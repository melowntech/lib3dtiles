/**
 * Copyright (c) 2021 Melown Technologies SE
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
 * ARE DISCLAIMED.  IN NO EVaENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
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

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/limits.hpp"
#include "utility/path.hpp"
#include "utility/openmp.hpp"
#include "utility/format.hpp"
#include "utility/streams.hpp"

#include "service/cmdline.hpp"

#include "3dtiles/reader.hpp"

namespace po = boost::program_options;
namespace bio = boost::iostreams;
namespace fs = boost::filesystem;
namespace tdt = threedtiles;

namespace {

class Metadata : public service::Cmdline
{
public:
    Metadata()
        : service::Cmdline("3dtiles-metadata", BUILD_TARGET_VERSION)
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

    fs::path input_;
    bool includeExternal_ = true;
};

void Metadata::configuration(po::options_description &cmdline
                             , po::options_description &config
                             , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to input SLPK archive.")
        ("includeExternal", po::value(&includeExternal_)
         ->default_value(includeExternal_)
         , "Read external references into one metadata tree.")
        ;

    pd
        .add("input", 1)
        ;

    (void) config;
}

void Metadata::configure(const po::variables_map&) {}

bool Metadata::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(3dtils-metadata

    Dumps 3D Tiles metadata in more readable way.

usage
    3dtiles-metadata INPUT [OPTIONS]
)RAW";
    }
    return false;
}

int Metadata::run()
{
    tdt::Archive ar(input_, {}, includeExternal_);

    const auto &ts(ar.tileset());

    std::cout << std::fixed << std::setprecision(10);
    dumpMetadata(std::cout, ts);
    std::cout << std::flush;

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    utility::unlimitedCoredump();
    return Metadata()(argc, argv);
}
