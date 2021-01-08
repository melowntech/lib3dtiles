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

#ifndef threedtiles_mesh_hpp_included_
#define threedtiles_mesh_hpp_included_

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "vts-libs/vts/basetypes3.hpp"
#include "vts-libs/vts/mesh.hpp"
#include "vts-libs/vts/atlas.hpp"

#include "gltf/gltf.hpp"

namespace threedtiles {

/** Image source.
 */
class ImageSource {
public:
    virtual ~ImageSource() {}
    virtual gltf::Image image(int idx) const = 0;
    virtual std::string info(int idx) const = 0;
};

/** Image source-based savers.
 */
void saveTile(std::ostream &os, const boost::filesystem::path &path
              , const vtslibs::vts::TileId &tileId
              , const vtslibs::vts::ConstSubMeshRange &submeshes
              , const ImageSource &images);

/** Image source-based savers.
 */
boost::filesystem::path
saveTile(const boost::filesystem::path &root
         , const vtslibs::vts::TileId &tileId
         , const vtslibs::vts::ConstSubMeshRange &submeshes
         , const ImageSource &images
         , const boost::optional<vtslibs::vts::TileId::index_type>
         &z = boost::none);

/** Atlas based savers.
 */
boost::filesystem::path
saveTile(const boost::filesystem::path &root
         , const vtslibs::vts::TileId &tileId
         , const vtslibs::vts::Mesh &mesh
         , const vtslibs::vts::Atlas &atlas
         , const boost::optional<vtslibs::vts::TileId::index_type>
         &z = boost::none);

boost::filesystem::path
saveTile(const boost::filesystem::path &root
         , const vtslibs::vts::TileId &tileId
         , const vtslibs::vts::ConstSubMeshRange &submeshes
         , const vtslibs::vts::Atlas &atlas
         , const boost::optional<vtslibs::vts::TileId::index_type>
         &z = boost::none);

boost::filesystem::path saveTile(const boost::filesystem::path &root
                                 , const vtslibs::vts::TileId3 &tileId
                                 , const vtslibs::vts::Mesh &mesh
                                 , const vtslibs::vts::Atlas &atlas);

boost::filesystem::path
saveTile(const boost::filesystem::path &root
         , const vtslibs::vts::TileId3 &tileId
         , const vtslibs::vts::ConstSubMeshRange &submeshes
         , const vtslibs::vts::Atlas &atlas);


boost::filesystem::path
tilePath(const vtslibs::vts::TileId &tileId
         , const std::string &extension
         , const boost::optional<vtslibs::vts::TileId::index_type>
         &z = boost::none);

boost::filesystem::path tilePath(const vtslibs::vts::TileId3 &tileId
                                 , const std::string &extension);

// inlines

/** Atlas image source.
 */
namespace detail {

class AtlasSource : public ImageSource {
public:
    AtlasSource(const vtslibs::vts::Atlas &atlas)
        : atlas_(atlas)
    {}

    gltf::Image image(int idx) const override;
    std::string info(int idx) const override;

private:
    const vtslibs::vts::Atlas &atlas_;
};

} // namespace detail

inline boost::filesystem::path
saveTile(const boost::filesystem::path &root
         , const vtslibs::vts::TileId &tileId
         , const vtslibs::vts::Mesh &mesh
         , const vtslibs::vts::Atlas &atlas
         , const boost::optional<vtslibs::vts::TileId::index_type> &z)
{
    return saveTile(root, tileId, vtslibs::vts::submeshRange(mesh)
                    , detail::AtlasSource(atlas), z);
}

inline boost::filesystem::path
saveTile(const boost::filesystem::path &root
         , const vtslibs::vts::TileId &tileId
         , const vtslibs::vts::ConstSubMeshRange &submeshes
         , const vtslibs::vts::Atlas &atlas
         , const boost::optional<vtslibs::vts::TileId::index_type> &z)
{
    return saveTile(root, tileId, submeshes, detail::AtlasSource(atlas), z);
}

inline boost::filesystem::path saveTile(const boost::filesystem::path &root
                                        , const vtslibs::vts::TileId3 &tileId
                                        , const vtslibs::vts::Mesh &mesh
                                        , const vtslibs::vts::Atlas &atlas)
{
    return saveTile(root, tileId.tileId(), vtslibs::vts::submeshRange(mesh)
                    , detail::AtlasSource(atlas), tileId.z);
}

inline boost::filesystem::path
saveTile(const boost::filesystem::path &root
         , const vtslibs::vts::TileId3 &tileId
         , const vtslibs::vts::ConstSubMeshRange &submeshes
         , const vtslibs::vts::Atlas &atlas)
{
    return saveTile(root, tileId.tileId(), submeshes
                    , detail::AtlasSource(atlas), tileId.z);
}

inline boost::filesystem::path tilePath(const vtslibs::vts::TileId3 &tileId
                                        , const std::string &extension)
{
    return tilePath(tileId.tileId(), extension, tileId.z);
}

} // namespace threedtiles

#endif // threedtiles_mesh_hpp_included_
