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

#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/uri.hpp"
#include "utility/openmp.hpp"

#include "reader.hpp"
#include "b3dm.hpp"

namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace threedtiles {

namespace {

namespace constants {
const std::string TilesetJson("tileset.json");
} // namespace constants

inline const fs::path& coalesce(const boost::optional<fs::path> &opt
                                , const fs::path &dflt)
{
    return opt ? *opt : dflt;
}

} // namespace

Archive::Archive(const fs::path &root, const std::string &mime
                 , bool includeExternal)
    : archive_(root
               , roarchive::OpenOptions().setHint(constants::TilesetJson)
               .setInlineHint('#')
               .setMime(mime))
    , tileset_(tileset(coalesce(archive_.usedHint(), constants::TilesetJson)
                       , includeExternal))
{}

Archive::Archive(roarchive::RoArchive &archive, bool includeExternal)
    : archive_(archive.applyHint(constants::TilesetJson))
    , tileset_(tileset(constants::TilesetJson, includeExternal))
{
}

roarchive::IStream::pointer Archive::istream(const fs::path &path) const
{
    return archive_.istream(path);
}

namespace {

void include(const Archive &archive, Tile::pointer &root)
{
    if (!root->children.empty()) {
        // non-empty children
        for (auto &child : root->children) {
            include(archive, child);
        }
    } else if (root->content) {
        if (utility::Uri(root->content->uri).absolute()) {
            // we are unable to include non-local data
            return;
        }

        // some content, recurse to external tileset
        if (ba::iends_with(root->content->uri, ".json")) {
            UTILITY_OMP(task shared(archive, root))
            {
                // TODO: merge tileset metadata
                root = archive.tileset(root->content->uri).root;
                include(archive, root);
            }
        }
    }
}

} // namespace

Tileset Archive::tileset(const fs::path &path, bool includeExternal) const
{
    Tileset ts;
    if (!includeExternal) {
        read(*istream(path), ts, path);
        return ts;
    }

    UTILITY_OMP(parallel shared(ts, path))
    {
        UTILITY_OMP(single)
        {
            read(*istream(path), ts, path);
            include(*this, ts.root);
        }
    }

    return ts;
}

void Archive::loadMesh(gltf::MeshLoader &loader
                       , const boost::filesystem::path &path) const
{
    auto model(b3dm(*istream(path), path));
    // TODO: add RTC center shift to trafo
    auto trafo(gltf::yup2zup());
    decodeMesh(loader, model.model, trafo);
}

} // namespace threedtiles
