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

#include <sstream>
#include <algorithm>

#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/array.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/binaryio.hpp"

#include "jsoncpp/io.hpp"

#include "gltf/meshloader.hpp"

#include "b3dm.hpp"

namespace bin = utility::binaryio;
namespace bio = boost::iostreams;

namespace threedtiles {

namespace detail {

const char MAGIC[4] = { 'b', '3', 'd', 'm' };

struct Header {
    std::uint32_t version = 1;
    std::uint32_t byteLength = size();
    std::uint32_t featureTableJSONByteLength = 0;
    std::uint32_t featureTableBinaryByteLength = 0;
    std::uint32_t batchTableJSONByteLength = 0;
    std::uint32_t batchTableBinaryByteLength = 0;

    static std::size_t size() {
        return 7 * sizeof(std::uint32_t);
    }
};

void write(std::ostream &os, const Header &header)
{
    bin::write(os, MAGIC);
    bin::write<std::uint32_t>(os, header.version);
    bin::write<std::uint32_t>(os, header.byteLength);
    bin::write<std::uint32_t>(os, header.featureTableJSONByteLength);
    bin::write<std::uint32_t>(os, header.featureTableBinaryByteLength);
    bin::write<std::uint32_t>(os, header.batchTableJSONByteLength);
    bin::write<std::uint32_t>(os, header.batchTableBinaryByteLength);
}

template <int alignment = 8>
void pad(std::ostream &os, std::size_t offset = 0, char chr = ' ')
{
    offset += os.tellp();
    if (const auto used = (offset % alignment)) {
        const auto padding(alignment - used);
        std::fill_n(std::ostream_iterator<char>(os), padding, chr);
    }
}

void featureTable(std::ostream &os
                  , const math::Point3 &rtcCenter
                  , std::size_t offset = 0)
{
    Json::Value ft(Json::objectValue);
    ft["BATCH_LENGTH"] = 0;
    if (rtcCenter != math::Point3()) {
        auto &c(ft["RTC_CENTER"] = Json::arrayValue);
        c.append(rtcCenter(0));
        c.append(rtcCenter(1));
        c.append(rtcCenter(2));
    }

    Json::write(os, ft, false);
    pad(os, offset, ' ');
}

} // namespace detail


/** Write a glTF archive as a b3dm file.
 */
void b3dm(std::ostream &os, const gltf::Model &model
          , const boost::filesystem::path &srcDir
          , const math::Point3 &rtcCenter)
{
    detail::Header header;

    // serialize and measure feature table.
    std::stringstream fs;
    detail::featureTable(fs, rtcCenter, header.byteLength);
    header.featureTableJSONByteLength = fs.tellp();
    header.byteLength += header.featureTableJSONByteLength;

    // serialize and measure GLB archive
    std::stringstream gs;
    gltf::glb(gs, model, srcDir);
    detail::pad(gs, header.byteLength, 0);
    header.byteLength += gs.tellp();

    // paste together
    detail::write(os, header);
    os << fs.rdbuf();
    os << gs.rdbuf();
}

/** Write a glTF archive as a b3dm file.
 */
void b3dm(const boost::filesystem::path &path, const gltf::Model &model
          , const boost::filesystem::path &srcDir
          , const math::Point3 &rtcCenter)
{
    LOG(info1) << "Generating b3dm in " << path  << ".";
    utility::ofstreambuf os(path.string());
    b3dm(os, model, srcDir, rtcCenter);
    os.close();
}

namespace detail {

typedef std::array<std::uint8_t, 4> Uint32Buffer;

struct Legacy {
    enum class Type { none, json, gltf };
    Type type = Type::none;
    Uint32Buffer buffer;
};

/** Reads b3dm header.
 */
Legacy read(std::istream &is, Header &header
            , const boost::filesystem::path &path)
{
    char magic[4];
    bin::read(is, magic);
    if (std::memcmp(magic, MAGIC, sizeof(MAGIC))) {
        LOGTHROW(err2, std::runtime_error)
            << "File " << path << " is not a Batched 3D Model file.";
    }

    header.version = bin::read<std::uint32_t>(is);
    header.byteLength = bin::read<std::uint32_t>(is);
    header.featureTableJSONByteLength = bin::read<std::uint32_t>(is);
    header.featureTableBinaryByteLength = bin::read<std::uint32_t>(is);

    // OK, legacy format ends here, we have to deal with old data
    // TODO: detect old format
    Legacy legacy;

    header.batchTableJSONByteLength = bin::read<std::uint32_t>(is);
    header.batchTableBinaryByteLength = bin::read<std::uint32_t>(is);

    return legacy;
}

void readFeatureTable(std::istream &is, const Header &header
                      , const boost::filesystem::path &path
                      , math::Point3 &rtcCenter)
{
    if (!header.featureTableJSONByteLength) { return; }

    Json::Value ft;
    {
        // read feature table to temporary buffer
        std::vector<char> buf(header.featureTableJSONByteLength);
        bin::read(is, buf);

        // wrap buffer to a stream
        bio::stream_buffer<bio::array_source>
            buffer(buf.data(), buf.data() + buf.size());
        std::istream s(&buffer);
        s.exceptions(std::ios::badbit | std::ios::failbit);

        // and read
        if (!read(s, ft)) {
            LOGTHROW(err2, std::runtime_error)
                << "Unable to read JSON feature table from file "
                << path << ".";
        }
    }

    if (ft.isMember("RTC_CENTER")) {
        const auto &rc(ft["RTC_CENTER"]);
        if (rc.size() != 3) {
            LOGTHROW(err2, std::runtime_error)
                << "Feature table from file "
                << path << " has malformed RTC_CENTER.";
        }
        rtcCenter(0) = rc[0].asDouble();
        rtcCenter(1) = rc[1].asDouble();
        rtcCenter(2) = rc[2].asDouble();
    }
}

} // namespace detail

BatchedModel b3dm(std::istream &is, const boost::filesystem::path &path)
{
    BatchedModel model;

    detail::Header header;
    const auto legacy(read(is, header, path));

    readFeatureTable(is, header, path, model.rtcCenter);

    // ignore rest of tables
    is.ignore(header.featureTableBinaryByteLength
              + header.batchTableJSONByteLength
              + header.batchTableBinaryByteLength);

    model.model = gltf::glb
        (is, path, (legacy.type == detail::Legacy::Type::gltf));

    return model;
}

BatchedModel b3dm(const boost::filesystem::path &path)
{
    utility::ifstreambuf f(path.string());
    auto model(b3dm(f, path));
    f.close();
    return model;
}

void loadMesh(gltf::MeshLoader &loader, const BatchedModel &model
              , gltf::MeshLoader::DecodeOptions options)
{
    // add rtc and Y-up to Z-up switch
    options.trafo = prod(options.trafo, math::translate(model.rtcCenter));
    options.trafo = prod(options.trafo, gltf::yup2zup());

    decodeMesh(loader, model.model, options);
}

} // namespace threedtiles
