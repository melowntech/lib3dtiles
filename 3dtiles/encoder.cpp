/**
 * Copyright (c) 2019 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/optional/optional_io.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "utility/cppversion.hpp"

#include "geo/srsdef.hpp"

#include "encoder.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace threedtiles {

namespace {

/** WGS84 in radians
 */
const auto Wgs84Rad(geo::setAngularUnit
                    (geo::SrsDefinition(4326), geo::AngularUnit::radian));

} // namespace

void Encoder::Config::configuration(po::options_description &config)
{
    config.add_options()
        ("geometricErrorFactor", po::value(&geometricErrorFactor)
         ->default_value(geometricErrorFactor)->required()
         , "Factor applied to pixelSize to obtain geometric error.")

        ("tilesetLimit", po::value(&tilesetLimit)
         ->default_value(tilesetLimit)->required()
         , "Limit for tiles in single tileset.json file. "
         "If generated tileset is larger it is split into multiple "
         "sub-tilesets.")

        ("parallel"
         , po::value(&parallel)->default_value(true)->required()
         , "Process in parallel.")
        ;
}

void Encoder::Config::configure(const po::variables_map &vars)
{
    (void) vars;
}

namespace {

double texelSize(const vts::Mesh &mesh, const vts::Atlas &atlas)
{
    const auto ma(area(mesh));
    double meshArea(ma.mesh);

    double textureArea(0.0);

    // mesh and texture area -> texelSize
    auto ita(ma.submeshes.begin());
    for (std::size_t i(0), e(atlas.size()); i != e; ++i, ++ita) {
        textureArea += ita->internalTexture * atlas.area(i);
    }

    return std::sqrt(meshArea / textureArea);
}

void updateBoundingVolumes(const Tile::pointer &tile
                           , const Tile::pointer &parent)
{
    if (!tile) { return; }

    // descend
    for (const auto &child : tile->children) {
        updateBoundingVolumes(child, tile);
    }

    if (parent) {
        update(parent->boundingVolume, tile->boundingVolume);
    }
}

double computeGeometricError(const Tile::pointer &tile
                             , double geometricErrorFactor)
{
    if (!tile) { return 0.0; }

    const auto ge(tile->geometricError * geometricErrorFactor);

    tile->geometricError = 0.0;

    // get maximum child geometric error
    for (const auto &child : tile->children) {
        tile->geometricError
            = std::max(tile->geometricError
                       , computeGeometricError(child, geometricErrorFactor));
    }

    // forward this error
    return ge ? ge : tile->geometricError;
}

struct Done {
    Done(std::size_t count, std::size_t total)
        : count(count), total(total)
    {}

    std::size_t count;
    std::size_t total;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Done &d)
{
    if (d.total) {
        double percentage((100.0 * d.count) / d.total);
        boost::io::ios_precision_saver ps(os);
        return os << '#' << d.count << " of " << d.total << " ("
                  << std::fixed << std::setprecision(2)
                  << std::setw(6) << percentage
                  << " % done)";
    }
    return os << '#' << d.count;
}

} // namespace

class Encoder::Detail {
public:
    Detail(Encoder &owner)
        : owner_(owner), config_(owner_.config_)
        , tileset_(owner_.tileset_), output_(owner_.output_)
        , ti_(owner_.validTiles_), fullTree_(ti_)
        , generated_(), total_(ti_.count())
        , srs2rad_(config_.srs, Wgs84Rad)
    {
        fullTree_.makeAbsolute().complete();
    }

    void generate() {
        if (config_.parallel) {
            UTILITY_OMP(parallel)
            UTILITY_OMP(single)
            {
                process({}, {});
            }
        } else {
            process({}, {});
        }
    }

private:
    void process(const vts::TileId &tileId, Tile *parent);

    Region tileVolume(const vts::Mesh &mesh) {
        Region region;
        for (const auto &sm : mesh) {
            for (const auto &v : sm.vertices) {
                math::update(region.extents, srs2rad_(v));
            }
        }
        return region;
    }

    Encoder &owner_;
    const Config &config_;
    Tileset &tileset_;
    const fs::path output_;

    const vts::TileIndex &ti_;
    vts::TileIndex fullTree_;
    std::atomic<std::size_t> generated_;
    std::size_t total_;

    geo::CsConvertor srs2rad_;
};

void Encoder::Detail::process(const vts::TileId &tileId, Tile *parent)
{
    struct TIDGuard {
        TIDGuard(const std::string &id)
            : old(dbglog::thread_id())
        {
            dbglog::thread_id(id);
        }
        ~TIDGuard() { try { dbglog::thread_id(old); } catch (...) {} }

        const std::string old;
    };

    if (!fullTree_.get(tileId)) { return; }

    TIDGuard tg(str(boost::format("tile:%s") % tileId));

    // hold tile in unique_ptr
    auto tileHolder(std::make_unique<Tile>());

    // borrow tile so it can be reference even after it is stored in the parent
    auto *tile(tileHolder.get());

    if (ti_.get(tileId)) {
        LOG(info2)
            << "Generating 3D tile from tile " << tileId << ".";

        auto texturedMesh(owner_.generate(tileId));

        auto &t(*tile);

        // store this tile's texel size in geometric error
        t.geometricError = texelSize(*texturedMesh.mesh, *texturedMesh.atlas);

        t.content = boost::in_place();

        t.boundingVolume = t.content->boundingVolume
            = tileVolume(*texturedMesh.mesh);

        t.content->uri = saveTile
            (output_, tileId, *texturedMesh.mesh, *texturedMesh.atlas)
            .string();

        Done done(++generated_, total_);

        LOG(info3)
            << "Generated 3D tile " << done
            << " from tile " << tileId << ".";
    }

    // store tile in the parent
    UTILITY_OMP(critical(threedtiles_encoder_process_1))
    {
        if (parent) {
            parent->children.push_back(std::move(tileHolder));
        } else {
            // parent is the Tileset itself
            tileset_.root = std::move(tileHolder);
            tile->refine = Refinement::replace;
        }
    }

    // proces children -> go down
    for (auto child : vts::children(tileId)) {
        UTILITY_OMP(task)
            process(child, tile);
    }
}

void Encoder::run()
{
    Detail(*this).generate();

    updateBoundingVolumes(tileset_.root, {});
    tileset_.geometricError
        = computeGeometricError(tileset_.root, config_.geometricErrorFactor);

    // split tileset into subtrees
    const auto subtrees(split(tileset_, config_.tilesetLimit));

    write(output_ / "tileset.json", tileset_);
    for (const auto &subtree : subtrees) {
        write(output_ / subtree.uri, subtree.tileset);
    }
}

} // namespace threedtiles
