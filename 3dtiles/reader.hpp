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

#ifndef threedtiles_reader_hpp_included_
#define threedtiles_reader_hpp_included_

#include "roarchive/roarchive.hpp"

#include "gltf/meshloader.hpp"

#include "3dtiles.hpp"

namespace threedtiles {

/** 3D Tiles archive reader
 */
class Archive {
public:
    Archive(const boost::filesystem::path &root, const std::string &mime = ""
            , bool includeExternal = false);
    Archive(roarchive::RoArchive &archive, bool includeExternal = false);

    /** Generic I/O.
     */
    roarchive::IStream::pointer
    istream(const boost::filesystem::path &path) const;

    /** Feed triangle mesh into parser from a b3dm file.
     */
    void loadMesh(gltf::MeshLoader &loader
                  , const boost::filesystem::path &path
                  , const gltf::MeshLoader::DecodeOptions &options
                  = gltf::MeshLoader::DecodeOptions()) const;

    /** Root tileset.
     */
    const Tileset& tileset() const { return tileset_; }

    const boost::filesystem::path& tilesetPath() const {
        return tilesetPath_;
    }

    const boost::filesystem::path path(const boost::filesystem::path &path)
        const
    {
        return archive_.path(path);
    }

    bool remote() const;

    /** Number of tiles in root tileset.
     */
    std::size_t treeSize() const { return treeSize_; }

    /** Read additional tileset file.
     */
    Tileset tileset(const boost::filesystem::path &path
                    , bool includeExternal = false
                    , bool relaxed = false) const;

private:
    const roarchive::RoArchive archive_;
    const boost::filesystem::path tilesetPath_;
    const Tileset tileset_;
    const std::size_t treeSize_;
};

} // namespace threedtiles

#endif // threedtiles_reader_hpp_included_
