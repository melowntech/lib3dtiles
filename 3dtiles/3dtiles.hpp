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

#ifndef threedtiles_3dtiles_hpp_included_
#define threedtiles_3dtiles_hpp_included_

#include <algorithm>
#include <iosfwd>

#include <vector>
#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include "utility/enum-io.hpp"
#include "utility/openmp.hpp"

#include "math/geometry_core.hpp"

namespace threedtiles {

using OptString = boost::optional<std::string>;
using ExtensionList = std::vector<std::string>;
using Extension = boost::any;
using Extensions = std::map<std::string, Extension>;

UTILITY_GENERATE_ENUM_CI(Refinement,
                         ((replace)("REPLACE"))
                         ((add)("ADD"))
                         )

struct CommonBase {
    Extensions extensions;
    boost::any extras;
};

struct Box : CommonBase {
    math::Point3 center;
    math::Point3 x;
    math::Point3 y;
    math::Point3 z;

    void update(const Box &other);
};

struct Region : CommonBase {
    math::Extents3 extents;

    Region() : extents(math::InvalidExtents{}) {}

    void update(const Region &other);
};

struct Sphere : CommonBase {
    math::Point3 center;
    double radius = 0.0;

    void update(const Sphere &other);
};

/** Bounding volume. Blank when not filled yet. Tile with blank bounding volume
 *  cannot be serialized.
 *
 *  boost::blank must stay first template argument otherwise
 *  valid(BoundingVolume) would stop working
 */
using BoundingVolume = boost::variant<boost::blank, Box, Region, Sphere>;

inline bool valid(const BoundingVolume &bv) { return bv.which(); }

struct Property : CommonBase {
    double minimum;
    double maximum;

    Property() : minimum(), maximum() {}
};

using Properties = std::map<std::string, Property>;

struct TileContent : CommonBase {
    /** Bounding volume can be invalid.
     */
    BoundingVolume boundingVolume;
    std::string uri;
};

struct Tile : CommonBase {
    typedef std::unique_ptr<Tile> pointer;
    typedef std::vector<pointer> list;

    BoundingVolume boundingVolume;
    BoundingVolume viewerRequestVolume;
    double geometricError = 0.0;
    boost::optional<Refinement> refine;
    boost::optional<math::Matrix4> transform; // serialize as column major!
    boost::optional<TileContent> content;
    Tile::list children;

    /** Return number of tiles in subtree rooted in this tile.
     */
    std::size_t subtreeSize() const;

    /** Return depth of subtree. No children -> 0.
     */
    std::size_t subtreeDepth() const;
};

struct Asset : CommonBase {
    std::string version;
    OptString tilesetVersion;

    Asset() : version("1.0") {}
};

struct Tileset : CommonBase {
    Asset asset;
    Properties properties;
    double geometricError = 0.0;
    Tile::pointer root;

    ExtensionList extensionsUsed;
    ExtensionList extensionsRequired;
};

/** Write a tileset JSON file to an output stream.
 */
void write(std::ostream &os, const Tileset &tileset);

/** Write a tileset JSON file to an output file.
 */
void write(const boost::filesystem::path &path, const Tileset &tileset);

struct TilesetWithUri {
    typedef std::vector<TilesetWithUri> list;

    std::string uri;
    Tileset tileset;

    TilesetWithUri(const std::string &uri) : uri(uri) {}
};

/** Splits tileset tree into multiple sub-trees.
 *
 * Tile limit is only indicative, subtree is always full pyramid unless there
 * are leaf nodes.
 *
 * NB: original tileset tree is modified!
 *
 * \param tileset tileset to split
 * \param tileLimit (soft) tile limit
 * \returns list of sub trees
 */
TilesetWithUri::list split(Tileset &tileset, std::size_t tileLimit);

/** Updates one volume from the other.
 *  If updated is valid then it has to be of the same type as the updater.
 */
void update(BoundingVolume &updated, const BoundingVolume &updater);

/** Read tileset JSON file from an output stream.
 */
void read(std::istream &is, Tileset &tileset
          , const boost::filesystem::path &path = "");

/** Resolves all relative URIs using provided base URI and distributes various
 * inherited data to all tiles. Used to make tileset consistent after load.
 *
 *  \param tileset tileset to update
 *  \param baseUri base URI
 *  \param relaxed relaxed validity check
 *  \return tileset itself, to allow call chaining
 */
Tileset& absolutize(Tileset &ts, const std::string &baseUri
                    , bool relaxed = false);

/** Path in tile tree.
 */
struct TilePath {
    std::vector<int> path;
    TilePath() { path.reserve(32); /* guesstimage */ }
    int depth() const { return path.size(); }
};

/** Recursively traverses the tile tree starting at root and calls
 *      op(const Tile &tile, const TilePath &path)
 *  for every encountered tile.
 */
template <typename Op>
void traverse(const Tile &root, Op op);

/** Recursively traverses the tile tree starting at root and calls
 *      op(const Tile &tile, const TilePath &path)
 *  for every encountered tile.
 *
 * Explicitly const version.
 */
template <typename Op>
void ctraverse(const Tile &root, Op op);

/** Recursively traverses the tile tree starting at root and calls
 *      op(Tile &tile, int depth)
 *  for every encountered tile.
 */
template <typename Op>
void traverse(Tile &root, Op op);

/** Deep copy, when needed.
 */
Tileset clone(const Tileset &ts);

// inlines

namespace detail {

template <typename TileT, typename Op>
void traverse(TileT &root, Op op) {
    struct Traverser {
        Op op;
        Traverser(Op op) : op(op) {}
        void process(TileT &tile, TilePath &path) {
            op(tile, path);

            int i(0);
            for (const auto &child : tile.children) {
                path.path.push_back(i++);
                process(*child, path);
                path.path.pop_back();
            }
        }
    } t(op);

    TilePath path;
    t.process(root, path);
}

} // namespace detail

template <typename Op> void traverse(const Tile &root, Op op)
{
    detail::traverse(root, op);
}

template <typename Op> void ctraverse(const Tile &root, Op op)
{
    detail::traverse(root, op);
}

template <typename Op> void traverse(Tile &root, Op op)
{
    detail::traverse(root, op);
}

inline std::size_t Tile::subtreeSize() const {
    std::size_t count(0);
    traverse(*this, [&count](const Tile&, const TilePath&) { ++count; });
    return count;
}

inline std::size_t Tile::subtreeDepth() const {
    std::size_t depth(0);
    traverse(*this, [&depth](const Tile&, const TilePath &path) {
                        depth = std::max(depth, std::size_t(path.depth()));
                    });
    return depth;
}

} // namespace threedtiles

#endif // 3dtiles_3dtiles_hpp_included_
