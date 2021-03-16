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

#include "geometry/meshop.hpp"

#include "imgproc/imagesize.cpp"

#include "3dtiles/b3dm.hpp"
#include "gltf/meshloader.hpp"

namespace po = boost::program_options;
namespace bio = boost::iostreams;
namespace fs = boost::filesystem;
namespace tdt = threedtiles;

namespace {

class B3dm2Obj : public service::Cmdline
{
public:
    B3dm2Obj()
        : service::Cmdline("b3dm2obj", BUILD_TARGET_VERSION)
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

    fs::path output_;
    fs::path input_;
};

void B3dm2Obj::configuration(po::options_description &cmdline
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

void B3dm2Obj::configure(const po::variables_map&) {}

bool B3dm2Obj::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(b3dm2obj

    Converts batched 3D model into textured meshes in OBJ format.

usage
    b3dm2obj INPUT OUTPUT [OPTIONS]
)RAW";
    }
    return false;
}

struct MeshLoader : gltf::MeshLoader
{
    MeshLoader(const fs::path &output)
        : output(output), smIndex(-1), smStart()
    {}

    /** New mesh has been encountered.
     */
    void mesh() override {
        smStart = m.vertices.size();
        ++smIndex;
    }

    /** Mesh vertices.
     */
    void vertices(math::Points3d &&v) override {
        m.vertices.insert(m.vertices.end()
                          , std::make_move_iterator(v.begin())
                          , std::make_move_iterator(v.end()));
    }

    /** Mesh texture coordinates.
     */
    void tc(math::Points2d &&tc) override {
        m.tCoords.insert(m.tCoords.end()
                         , std::make_move_iterator(tc.begin())
                         , std::make_move_iterator(tc.end()));
    }

    /** Mexh faces. Indices are valid for both 3D and 2D vertices (i.e. vertices
     *  and texture coordinates.
     */
    void faces(Faces &&faces) override {
        for (const auto &face : faces) {
            m.faces.emplace_back
                (face(0) + smStart, face(1) + smStart, face(2) + smStart
                 , face(0) + smStart, face(1) + smStart, face(2) + smStart
                 , smIndex);
        }
    }

    /** Image data.
     */
    void image(const DataView &imageData) override {
        const auto &type(imgproc::imageType
                         (imageData.first, gltf::size(imageData)));

        const auto name(utility::format("%d%s", smIndex, type));
        utility::write(output / name, imageData.first, gltf::size(imageData));
        imageNames.push_back(name);
    }

    /** Image reference.
     */
    void image(const fs::path &path) override {
        imageNames.push_back(path.string());
    }

    void finish() {
        {
            LOG(info1) << "Writing material file.";
            std::ofstream f((output / "mesh.mtl").string());

            int index(0);
            for (const auto &name : imageNames) {
                f << "newmtl " << index << "\n"
                  << "map_Kd " << name
                  << "\n";

                ++index;
            }
            f.close();
        }

        LOG(info3) << "Saving mesh to " << output << ".";
        saveAsObj(m, output / "mesh.obj", geometry::ObjMaterial("mesh.mtl"));
    }

    const fs::path output;
    geometry::Mesh m;
    int smIndex;
    geometry::Face::index_type smStart;
    std::vector<std::string> imageNames;
};

int B3dm2Obj::run()
{
    auto model(tdt::b3dm(input_));

    fs::create_directories(output_);
    MeshLoader loader(output_);
    MeshLoader::DecodeOptions options;
    options.flipTc = true;
    tdt::loadMesh(loader, model, options);
    loader.finish();

    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    utility::unlimitedCoredump();
    return B3dm2Obj()(argc, argv);
}
