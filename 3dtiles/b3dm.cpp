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

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/binaryio.hpp"

#include "jsoncpp/io.hpp"

#include "./b3dm.hpp"

namespace bin = utility::binaryio;

namespace threedtiles {

namespace detail {

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
    os << "b3dm";
    bin::write<std::uint32_t>(os, header.version);
    bin::write<std::uint32_t>(os, header.byteLength);
    bin::write<std::uint32_t>(os, header.featureTableJSONByteLength);
    bin::write<std::uint32_t>(os, header.featureTableBinaryByteLength);
    bin::write<std::uint32_t>(os, header.batchTableJSONByteLength);
    bin::write<std::uint32_t>(os, header.batchTableBinaryByteLength);
}

void pad(std::ostream &os, char chr, int padding = 8)
{
    const std::size_t size(os.tellp());
    const auto rem(size % padding);
    if (!rem) { return; }

    padding -= rem;
    while (padding--) { os << chr; }
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
void b3dm(std::ostream &os, const gltf::GLTF &ga
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
    gltf::glb(gs, ga, srcDir);
    header.byteLength += gs.tellp();

    detail::write(os, header);

    os << fs.rdbuf();
    os << gs.rdbuf();
}

/** Write a glTF archive as a b3dm file.
 */
void b3dm(const boost::filesystem::path &path, const gltf::GLTF &ga
          , const boost::filesystem::path &srcDir
          , const math::Point3 &rtcCenter)
{
    LOG(info1) << "Generating b3dm in " << path  << ".";
    utility::ofstreambuf os(path.string());
    b3dm(os, ga, srcDir, rtcCenter);
    os.close();
}

} // namespace threedtiles
