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

template <int padding = 8>
void pad(std::ostream &os, char chr = ' ')
{
    if (const auto rem = std::size_t(os.tellp()) % padding) {
        std::fill_n(std::ostream_iterator<char>(os), padding - rem, chr);
    }
}

void featureTable(std::ostream &os
                  , const math::Point3 &rtcCenter)
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
    pad(os, ' ');
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
    detail::featureTable(fs, rtcCenter);
    header.featureTableJSONByteLength = fs.tellp();
    header.byteLength += header.featureTableJSONByteLength;

    // serialize and measure GLB archive
    std::stringstream gs;
    gltf::glb(gs, model, srcDir);
    header.byteLength += gs.tellp();

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

void read(std::istream &is, Header &header
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
    header.batchTableJSONByteLength = bin::read<std::uint32_t>(is);
    header.batchTableBinaryByteLength = bin::read<std::uint32_t>(is);
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
    read(is, header, path);

    readFeatureTable(is, header, path, model.rtcCenter);

    // ignore rest of tables
    is.ignore(header.featureTableBinaryByteLength
              + header.batchTableJSONByteLength
              + header.batchTableBinaryByteLength);

    model.model = gltf::glb(is, path);

    return model;
}

} // namespace threedtiles
