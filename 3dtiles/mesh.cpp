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

#include <map>

#include <boost/crc.hpp>

#include "utility/stl-helpers.hpp"
#include "utility/format.hpp"
#include "utility/binaryio.hpp"

#include "gltf/gltf.hpp"
#include "3dtiles/b3dm.hpp"

#include "./mesh.hpp"

namespace fs = boost::filesystem;
namespace vts = vtslibs::vts;
namespace bin = utility::binaryio;

namespace threedtiles {

std::uint32_t calculateHash(const std::string &data)
{
    boost::crc_32_type crc;
    crc.process_bytes(data.data(), data.size());
    return crc.checksum();
}

fs::path dir(const fs::path &filename)
{
    const auto hash(calculateHash(filename.string()));
    return str(boost::format("%02x") % ((hash >> 24) & 0xff));
}

template <typename T>
void writeFaceIndices(std::ostream &os, const vts::SubMesh &mesh)
{
    for (const auto &face : mesh.faces) {
        bin::write(os, T(face(0)));
        bin::write(os, T(face(1)));
        bin::write(os, T(face(2)));
    }
}

gltf::Index serializeMesh(gltf::GLTF &ga, const std::string &name
                          , const vts::SubMesh &sm
                          , const math::Point3 &center
                          , const gltf::Index &materialId)
{
    const gltf::Index nodeId(ga.nodes.size());
    auto &node(utility::append(ga.nodes));

    const gltf::Index omeshId(ga.meshes.size());
    auto &omesh(utility::append(ga.meshes));
    node.mesh = omeshId;

    const gltf::Index bufferId(ga.buffers.size());
    auto &buffer(boost::get<gltf::InlineBuffer>
                 (utility::append(ga.buffers, gltf::InlineBuffer())));

    // make room for all 3 buffer views to prevent reallocation
    ga.bufferViews.reserve(ga.bufferViews.size() + 3);
    const gltf::Index facesId(ga.bufferViews.size());
    auto &faces(utility::append(ga.bufferViews, bufferId));

    const gltf::Index verticesId(ga.bufferViews.size());
    auto &vertices(utility::append(ga.bufferViews, bufferId));

    const gltf::Index tcId(ga.bufferViews.size());
    auto &tc(utility::append(ga.bufferViews, bufferId));

    node.name = name;
    omesh.name = name;
    buffer.name = name;

    // use ubyte, ushort or uint depending on the number of vertices
    const auto vertexIndexSize([&]() -> std::size_t
    {
        if (sm.vertices.size() < (1 << 8)) { return 1; }
        if (sm.vertices.size() < (1 << 16)) { return 2; }
        return 4;
    }());

    const auto &extents(math::computeExtents(sm.vertices));
    const math::Extents3 localExtents
        (extents.ll - center, extents.ur - center);

    {
        std::ostringstream os;
        auto currentPosition([&]() -> std::size_t { return os.tellp(); });

        // well, not nice, but does the job...
        auto align([&](std::size_t alignment)
        {
            auto current(currentPosition());
            while (current % alignment) {
                bin::write(os, std::uint8_t());
                ++current;
            }
            return current;
        });

        faces.byteOffset = currentPosition();

        // 1) write faces
        switch (vertexIndexSize) {
        case 1: writeFaceIndices<std::uint8_t>(os, sm); break;
        case 2: writeFaceIndices<std::uint16_t>(os, sm); break;
        case 4: writeFaceIndices<std::uint32_t>(os, sm); break;
        }

        faces.byteLength = currentPosition() - faces.byteOffset;
        vertices.byteOffset = align(sizeof(float));

        // 2) write vertices
        for (const auto &v : sm.vertices) {
            // translate to origin
            const math::Point3 p(v - center);

            bin::write(os, float(p(0)));
            bin::write(os, float(p(1)));
            bin::write(os, float(p(2)));
        }

        vertices.byteLength = currentPosition() - vertices.byteOffset;
        tc.byteOffset = align(sizeof(float));

        // 3) write texture coordinates
        for (const auto &tc : sm.tc) {
            bin::write(os, float(tc(0)));
            bin::write(os, float(1.0 - tc(1)));
        }

        {
            const auto data(os.str());
            buffer.data.assign(data.begin(), data.end());
        }

        tc.byteLength = buffer.data.size() - tc.byteOffset;
    }

    // mesh primitives
    auto &primitive(utility::append(omesh.primitives));
    primitive.mode = gltf::PrimitiveMode::triangles;
    primitive.material = materialId;

    // create buffer views and their accessors
    {
        auto &bv(faces);
        bv.target = gltf::Target::elementArrayBuffer;
        bv.name = utility::concat(name, ".faces");

        const gltf::Index aId(ga.accessors.size());
        primitive.indices = aId;

        auto &a(utility::append(ga.accessors));
        a.bufferView = facesId;

        switch (vertexIndexSize) {
        case 1: a.componentType = gltf::ComponentType::ubyte; break;
        case 2: a.componentType = gltf::ComponentType::ushort; break;
        case 4: a.componentType = gltf::ComponentType::uint; break;
        }

        a.count = 3 * sm.faces.size();
        a.type = gltf::AttributeType::scalar;
        a.name = bv.name;

    }

    {
        auto &bv(vertices);
        bv.buffer = bufferId;
        bv.target = gltf::Target::arrayBuffer;
        bv.name = utility::concat(name, ".vertices");

        const gltf::Index aId(ga.accessors.size());

        auto &a(utility::append(ga.accessors));
        a.bufferView = verticesId;

        a.componentType = gltf::ComponentType::float_;
        a.count = sm.vertices.size();
        a.type = gltf::AttributeType::vec3;
        a.max = { localExtents.ur(0), localExtents.ur(1)
                  , localExtents.ur(2) };
        a.min = { localExtents.ll(0), localExtents.ll(1)
                  , localExtents.ll(2) };
        a.name = bv.name;

        primitive.attributes
            [gltf::AttributeSemantic::position] = aId;
    }

    {
        auto &bv(tc);
        bv.buffer = bufferId;
        bv.target = gltf::Target::arrayBuffer;
        bv.name = utility::concat(name, ".tc");

        const gltf::Index aId(ga.accessors.size());
        auto &a(utility::append(ga.accessors));
        a.bufferView = tcId;

        a.componentType = gltf::ComponentType::float_;
        a.count = sm.tc.size();
        a.type = gltf::AttributeType::vec2;
        a.name = bv.name;

        primitive.attributes
            [gltf::AttributeSemantic::texCoord0] = aId;
    }

    return nodeId;
}

gltf::Index serializeTexture(gltf::GLTF &ga, const std::string &name
                             , const vts::Atlas &atlas, int idx
                             , gltf::Index simpleSamplerId)
{
    // serialize image into memory
    gltf::InlineImage image;
    {
        std::ostringstream os;
        atlas.write(os, idx);
        const auto str(os.str());
        image.data.assign(str.data(), str.data() + str.size());
    }
    // TODO: get mime-type from atlas
    image.mimeType = "image/jpeg";
    const gltf::Index imageId(ga.images.size());
    ga.images.push_back(std::move(image));

    const gltf::Index textureId(ga.textures.size());
    const gltf::Index materialId(ga.materials.size());

    utility::append(ga.textures, simpleSamplerId, imageId);
    auto &material(utility::append(ga.materials));

    material.name = name;
    material.pbrMetallicRoughness = boost::in_place();
    {
        auto &pbr(*material.pbrMetallicRoughness);

        // non-metalic behaviour if unlit extension doesn't kick in
        pbr.metallicFactor = 0.0;

        pbr.baseColorTexture = boost::in_place();
        auto &ti(*pbr.baseColorTexture);
        ti.index = textureId;
    }

    // make it unlit
    material.extensions["KHR_materials_unlit"] = gltf::emptyObject();

    return materialId;
}

fs::path saveTile(const fs::path &root, const vts::TileId &tileId
                  , const vts::Mesh &mesh, const vts::Atlas &atlas)
{
    LOG(info2) << "Saving tile " << tileId
               << ", submeshes: " << mesh.submeshes.size();

    const fs::path fname(utility::format
                         ("%s-%s-%s.b3dm"
                          , tileId.lod, tileId.x, tileId.y));

    const fs::path path(dir(fname) / fname);

    const auto fullPath(root / path);
    fs::create_directories(fullPath.parent_path());

    gltf::GLTF ga;
    const auto simpleSamplerId(gltf::add(ga.samplers));
    ga.extensionsUsed.push_back("KHR_materials_unlit");

    const auto me(vts::extents(mesh));
    const auto mc(math::center(me));

    int idx(0);
    gltf::Indices nodes;
    for (const auto &sm : mesh.submeshes) {
        LOG(info2) << "Saving submesh " << idx << " of tile " << tileId
                   << ", vertices: " << sm.vertices.size()
                   << ", faces: " << sm.faces.size()
                   << ", texture: " << atlas.imageSize(idx);

        const auto name(utility::format("%s.%s", tileId, idx));
        const auto materialId
            (serializeTexture(ga, name, atlas, idx, simpleSamplerId));
        const auto nodeId(serializeMesh(ga, name, sm, mc, materialId));
        nodes.push_back(nodeId);

        ++idx;
    }

    {
        const gltf::Index rootId(ga.nodes.size());
        ga.defaultScene().nodes.push_back(rootId);
        auto &rootNode(utility::append(ga.nodes));
        rootNode.matrix = gltf::zup2yup();
        rootNode.name = utility::format("%s", tileId);
        rootNode.children = nodes;
    }

    b3dm(fullPath, ga, {}, mc);

    return path;
}

} // namespace threedtiles
