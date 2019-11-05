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

#ifndef threedtiles_encoder_hpp_included_
#define threedtiles_encoder_hpp_included_

#include <atomic>

#include <boost/filesystem/path.hpp>

#include "utility/gccversion.hpp"

#include "vts-libs/vts/tileindex.hpp"

#include "3dtiles.hpp"
#include "mesh.hpp"

namespace vts = vtslibs::vts;

namespace threedtiles {

class Encoder {
public:
    struct Config {
        double geometricErrorFactor;
        std::size_t tilesetLimit;
        bool parallel;

        Config()
            : geometricErrorFactor(16.0)
            , tilesetLimit(1000)
            , parallel(true)
        {}

        void configuration(boost::program_options::options_description
                           &config);

        void configure(const boost::program_options::variables_map &vars);
    };

    Encoder(const Config &config, const boost::filesystem::path &output
            , const vts::TileIndex &validTiles)
        : config_(config), output_(output)
        , ti_(validTiles)
        , fullTree_(ti_), generated_(), total_(ti_.count())
    {
        fullTree_.makeAbsolute().complete();
    }

    virtual ~Encoder() {}

    void run();

    struct TexturedMesh {
        vts::Mesh::pointer mesh;
        vts::Atlas::pointer atlas;

        template <typename AtlasType>
        AtlasType& initialize() {
            mesh = std::make_shared<vts::Mesh>();
            auto atlas(std::make_shared<AtlasType>());
            this->atlas = atlas;
            return *atlas;
        }
    };

private:
    virtual TexturedMesh generate(const vts::TileId &tileId) = 0;

    void process(const vts::TileId &tileId, Tile *parent);

    double texelSize(const vts::Mesh &mesh, const vts::Atlas &atlas) const;

    const Config &config_;
    const boost::filesystem::path output_;

    vts::TileIndex ti_;
    vts::TileIndex fullTree_;

    Tileset tileset_;

    std::atomic<std::size_t> generated_;
    std::size_t total_;
};

} // namespace threedtiles

#endif // threedtiles_encoder_hpp_included_
