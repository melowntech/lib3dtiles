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

#include <set>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/limits.hpp"
#include "utility/path.hpp"
#include "utility/openmp.hpp"
#include "utility/format.hpp"
#include "utility/streams.hpp"
#include "utility/uri.hpp"

#include "service/cmdline.hpp"

#include "geometry/meshop.hpp"

#include "imgproc/imagesize.cpp"

#include "3dtiles/reader.hpp"

namespace po = boost::program_options;
namespace bio = boost::iostreams;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace tdt = threedtiles;

namespace {

class Clone3dTiles : public service::Cmdline
{
public:
    Clone3dTiles()
        : service::Cmdline("clone-3dtiles", BUILD_TARGET_VERSION)
    {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        override;

    virtual void configure(const po::variables_map &vars)
        override;

    virtual bool help(std::ostream &out, const std::string &what) const
        override;

    virtual int run() override;

    void process(const tdt::Archive &archive, tdt::Tileset &&ts
                 , std::string &tsPath, const fs::path &parentTsPath);

    fs::path output_;
    fs::path input_;
};

void Clone3dTiles::configuration(po::options_description &cmdline
                                , po::options_description &config
                                , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("output", po::value(&output_)->required()
         , "Path to output converted input.")
        ("input", po::value(&input_)->required()
         , "Path to input SLPK archive.")
        ;

    pd
        .add("input", 1)
        .add("output", 1);

    (void) config;
}

void Clone3dTiles::configure(const po::variables_map&) {}

bool Clone3dTiles::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(clone-3dtiles

    Clones 3D TIles archive into local disk.

usage
    clone-3dtiles INPUT OUTPUT [OPTIONS]
)RAW";
    }
    return false;
}

bool isJson(const std::string &uri)
{
    return ba::iends_with(uri, ".json");
}

fs::path uri2path(const utility::Uri &uri)
{
    if (uri.scheme().empty()) { return str(uri); }

    fs::path p;
    std::ostringstream os;
    os << uri.scheme() << '_' << uri.components().host;
    if (uri.components().port > -1) {
        os << '_' << uri.components().port;
    }
    return p / os.str() / uri.path();
}

void Clone3dTiles::process(const tdt::Archive &archive, tdt::Tileset &&ts
                           , std::string &tsPath, const fs::path &parentTsPath)
{
    LOG(info3) << "Processing tileset <" << tsPath << ">.";

    fs::path tsOutputPath(uri2path(tsPath));

    traverse(*ts.root, [&](tdt::Tile &tile, const tdt::TilePath&)
             -> void
    {
        if (!tile.content) { return; }

        if (isJson(tile.content->uri)) {
            return process(archive, archive.tileset(tile.content->uri)
                           , tile.content->uri, tsOutputPath);
        }

        fs::path outputPath(uri2path(tile.content->uri));

        LOG(info3) << "Saving <" << tile.content->uri << "> in "
                   << outputPath << ".";

        // load and save
        {
            const auto path(output_ / outputPath);
            fs::create_directories(path.parent_path());
            copy(archive.istream(tile.content->uri), path);
        }

        // replace path (make relative)
        tile.content->uri
            = utility::lexically_relative
            (outputPath, tsOutputPath.parent_path()).string();
    });

    // build path and store
    LOG(info3) << "Saving <" << tsPath << "> in " << tsOutputPath << ".";
    {
        const auto path(output_ / tsOutputPath);
        fs::create_directories(path.parent_path());
        write(path, ts);
    }

    // replace path
    LOG(info4) << "tsOutputPath: " << tsOutputPath;
    LOG(info4) << "parentTsPath: " << parentTsPath;
    LOG(info4) << "parentTsPath.parent_path(): " << parentTsPath.parent_path();
    tsPath = utility::lexically_relative
        (tsOutputPath, parentTsPath.parent_path()).string();
    LOG(info4) << "tsPath: " << tsPath;
}

int Clone3dTiles::run()
{
    tdt::Archive archive(input_);

    auto tsPath((archive.remote() ? archive.path(archive.tilesetPath())
                 : archive.tilesetPath()).string());
    process(archive, clone(archive.tileset()), tsPath, {});

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    utility::unlimitedCoredump();
    return Clone3dTiles()(argc, argv);
}
