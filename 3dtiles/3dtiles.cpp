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

#include <iterator>

#include <boost/lexical_cast.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/format.hpp"

#include "math/transform.hpp"

#include "utility/cppversion.hpp"
#include "utility/streams.hpp"
#include "utility/format.hpp"
#include "utility/uri.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "3dtiles.hpp"
#include "io.hpp"

namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace threedtiles {

namespace detail {

void build(Json::Value &value, const Tile &tile);

template <typename T>
typename std::enable_if<!std::is_enum<T>::value, void>::type
build(Json::Value &object, const T &value);

template <typename T>
void build(Json::Value &object, const char *name
           , const boost::optional<T> &value);

/** We do not support optional BoundingVolume
 */
template <>
void build<BoundingVolume>(Json::Value &object, const char *name
                           , const boost::optional<BoundingVolume> &value)
= delete;

template <typename T>
void build(Json::Value &object, const char *name
           , const std::unique_ptr<T> &value);

template <typename T>
void build(Json::Value &value, const std::vector<T> &list);

template <typename T>
void build(Json::Value &object, const char *name, const std::vector<T> &list);

template <typename T>
void build(Json::Value &object, const char *name
           , const std::map<std::string, T> &map);


/** Any is expected to be Json::Value instance.
 */
void build(Json::Value &object, const char *name, const boost::any &value)
{
    if (const auto *valueObject = boost::any_cast<Json::Value>(&value)) {
        object[name] = *valueObject;
    }
}

void build(Json::Value &object, const math::Matrix4 &matrix)
{
    // dump matrix as a column major
    object = Json::arrayValue;
    for (int column(0); column < 4; ++column) {
        for (int row(0); row < 4; ++row) {
            object.append(matrix(row, column));
        }
    }
}

void build(Json::Value &object, const math::Point2d &p)
{
    object = Json::arrayValue;
    object.append(p(0));
    object.append(p(1));
}

void build(Json::Value &object, const math::Point3d &p)
{
    object = Json::arrayValue;
    object.append(p(0));
    object.append(p(1));
    object.append(p(2));
}

void build(Json::Value &object, const math::Point4d &p)
{
    object = Json::arrayValue;
    object.append(p(0));
    object.append(p(1));
    object.append(p(2));
    object.append(p(3));
}

void common(Json::Value &value, const CommonBase &cb)
{
    value = Json::objectValue;
    build(value, "extensions", cb.extensions);
    build(value, "extras", cb.extras);
}

void build(Json::Value &object, Refinement refinement)
{
    object = boost::lexical_cast<std::string>(refinement);
}

void build(Json::Value &value, const BoundingVolume &bv)
{
    struct Builder : public boost::static_visitor<void> {
        Json::Value &value;
        Builder(Json::Value &value) : value(value) {
            value = Json::objectValue;
        }

        void operator()(const Box &b) {
            auto &array(value["box"] = Json::arrayValue);

            array.append(b.center(0));
            array.append(b.center(1));
            array.append(b.center(2));
            array.append(b.x(0));
            array.append(b.x(1));
            array.append(b.x(2));
            array.append(b.y(0));
            array.append(b.y(1));
            array.append(b.y(2));
            array.append(b.z(0));
            array.append(b.z(1));
            array.append(b.z(2));
        }

        void operator()(const Region &r) {
            auto &array(value["region"] = Json::arrayValue);

            array.append(r.extents.ll(0));
            array.append(r.extents.ll(1));
            array.append(r.extents.ur(0));
            array.append(r.extents.ur(1));
            array.append(r.extents.ll(2));
            array.append(r.extents.ur(2));
        }

        void operator()(const Sphere &s) {
            auto &array(value["sphere"] = Json::arrayValue);

            array.append(s.center(0));
            array.append(s.center(1));
            array.append(s.center(2));
            array.append(s.radius);
        }

        void operator()(const boost::blank&) {
            LOGTHROW(err2, std::runtime_error)
                << "Invalid bounding volume cannot be serialized.";
        }
    } builder(value);
    boost::apply_visitor(builder, bv);
}

void build(Json::Value &value, const char *name, const BoundingVolume &bv)
{
    if (valid(bv)) { build(value[name], bv); }
}

void build(Json::Value &value, const TileContent &content)
{
    common(value, content);
    build(value["uri"], content.uri);
    build(value, "boundingVolume", content.boundingVolume);
}

template<class T>
typename std::enable_if<std::is_enum<T>::value, void>::type
build(Json::Value &object, const T &value)
{
    object = static_cast<int>(value);
}

template <typename T>
typename std::enable_if<!std::is_enum<T>::value, void>::type
build(Json::Value &object, const T &value)
{
    object = value;
}

template <>
void build<std::size_t>(Json::Value &object, const std::size_t &value)
{
    object = Json::UInt64(value);
}

template <typename T>
void build(Json::Value &object, const char *name
           , const boost::optional<T> &value)
{
    if (value) { build(object[name], *value); }
}

template <typename T>
void build(Json::Value &object, const char *name
           , const std::unique_ptr<T> &value)
{
    if (value) { build(object[name], *value); }
}

template <typename T>
void build(Json::Value &object, const std::unique_ptr<T> &value)
{
    build(object, *value);
}

template <typename T>
void build(Json::Value &value, const std::vector<T> &list)
{
    value = Json::arrayValue;
    for (const auto &element : list) {
        build(value.append({}), element);
    }
}

template <typename T>
void build(Json::Value &object, const char *name, const std::vector<T> &list)
{
    if (list.empty()) { return; }

    auto &value(object[name] = Json::arrayValue);
    for (const auto &element : list) {
        build(value.append({}), element);
    }
}

template <typename T>
void build(Json::Value &object, const char *name
           , const std::map<std::string, T> &map)
{
    if (map.empty()) { return; }

    auto &value(object[name] = Json::objectValue);
    for (const auto &element : map) {
        build(value, element.first.c_str(), element.second);
    }
}

void build(Json::Value &value, const Tile &tile)
{
    common(value, tile);
    build(value["boundingVolume"], tile.boundingVolume);
    build(value, "viewerRequestVolume", tile.viewerRequestVolume);
    build(value["geometricError"], tile.geometricError);
    build(value, "refine", tile.refine);
    build(value, "transform", tile.transform);

    build(value, "content", tile.content);
    build(value, "children", tile.children);
}

void build(Json::Value &value, const Asset &asset)
{
    build(value["version"], asset.version);
    build(value, "tilesetVersion", asset.tilesetVersion);
}

void build(Json::Value &value, const Tileset &tileset)
{
    common(value, tileset);

    build(value["asset"], tileset.asset);
    build(value, "root", tileset.root);
    build(value["geometricError"], tileset.geometricError);

    build(value, "extensionsUsed", tileset.extensionsUsed);
    build(value, "extensionsRequired", tileset.extensionsRequired);
}

template <typename T>
bool update(BoundingVolume &updated, const BoundingVolume &updater)
{
    if (auto *dst = boost::get<T>(&updated)) {
        if (auto *src = boost::get<T>(&updater)) {
            dst->update(*src);
            return true;
        }
        LOGTHROW(err2, std::runtime_error)
            << "Bounding volume can be updated only with another bounding "
            "volume of the same type.";
    }
    return false;
}

} // namespace detail

void write(std::ostream &os, const Tileset &tileset)
{
    Json::Value value;
    detail::build(value, tileset);
    Json::write(os, value, false);
}

void write(const fs::path &path, const Tileset &tileset)
{
    LOG(info1) << "Saving Tileset to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    write(f, tileset);
    f.close();
}

void update(BoundingVolume &updated, const BoundingVolume &updater)
{
    if (!valid(updater)) {
        // invalid updater -> no-op
        return;
    }

    if (!valid(updated)) {
        // invalid updated -> copy
        updated = updater;
        return;
    }

    if (detail::update<Box>(updated, updater)) { return; }
    if (detail::update<Region>(updated, updater)) { return; }
    if (detail::update<Sphere>(updated, updater)) { return; }
}

void Box::update(const Box&)
{
    LOGTHROW(err2, std::runtime_error)
        << "Update of box bounding volume not implemented yet.";
}

void Region::update(const Region &other)
{
    math::update(extents, other.extents.ll);
    math::update(extents, other.extents.ur);
}

void Sphere::update(const Sphere&)
{
    LOGTHROW(err2, std::runtime_error)
        << "Update of sphere bounding volume not implemented yet.";
}

namespace detail {

void common(CommonBase &cb, const Json::Value &value)
{
    if (value.isMember("extensions")) {
        const auto &extensions(value["extensions"]);
        for (const auto &name : extensions.getMemberNames()) {
            cb.extensions.insert(Extensions::value_type
                                 (name, extensions[name]));
        }
    }
    if (value.isMember("extras")) {
        cb.extras = value["extras"];
    }
}

template <typename T>
void parse(boost::optional<T> &dst, const Json::Value &value
           , const char *member);

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const char *member);

void parse(std::vector<std::string> &list, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Expected JSON array.";
    }

    for (const auto &item : value) {
        if (!item.isString()) {
            LOGTHROW(err1, Json::Error)
                << "Expected JSON array of strings.";
        }
        list.push_back(item.asString());
    }
}

void parse(std::vector<std::string> &list, const Json::Value &value
           , const char *member)
{
    if (!value.isMember(member)) { return; }
    parse(list, value["member"]);
}

void parse(Box &box, const Json::Value &value)
{
    if ((value.type() != Json::arrayValue) || (value.size() != 12)) {
        LOGTHROW(err1, Json::Error)
            << "Box doesn't have 12 elements (but " << value.size() << ").";
    }

    Json::get(box.center(0), value[0]);
    Json::get(box.center(1), value[1]);
    Json::get(box.center(2), value[2]);
    Json::get(box.x(0), value[3]);
    Json::get(box.x(1), value[4]);
    Json::get(box.x(2), value[5]);
    Json::get(box.y(0), value[6]);
    Json::get(box.y(1), value[7]);
    Json::get(box.y(2), value[8]);
    Json::get(box.z(0), value[9]);
    Json::get(box.z(1), value[10]);
    Json::get(box.z(2), value[11]);
}

void parse(Region &region, const Json::Value &value)
{
    if ((value.type() != Json::arrayValue) || (value.size() != 6)) {
        LOGTHROW(err1, Json::Error)
            << "Region doesn't have 6 elements (but " << value.size() << ").";
    }

    Json::get(region.extents.ll(0), value[0]);
    Json::get(region.extents.ll(1), value[1]);
    Json::get(region.extents.ur(0), value[2]);
    Json::get(region.extents.ur(1), value[3]);
    Json::get(region.extents.ll(2), value[4]);
    Json::get(region.extents.ur(2), value[5]);
}

void parse(Sphere &sphere, const Json::Value &value)
{
    if ((value.type() != Json::arrayValue) || (value.size() != 4)) {
        LOGTHROW(err1, Json::Error)
            << "Sphere doesn't have 4 elements (but " << value.size() << ").";
    }

    Json::get(sphere.center(0), value[0]);
    Json::get(sphere.center(1), value[1]);
    Json::get(sphere.center(2), value[2]);
    Json::get(sphere.radius, value[3]);
}

void parse(BoundingVolume &bv, const Json::Value &value)
{
    if (value.isMember("box")) {
        Box box;
        parse(box, value["box"]);
        bv = box;
    } else if (value.isMember("region")) {
        Region region;
        parse(region, value["region"]);
        bv = region;
    } else if (value.isMember("sphere")) {
        Sphere sphere;
        parse(sphere, value["sphere"]);
        bv = sphere;
    } else {
        LOGTHROW(err1, Json::Error)
            << "Invalid boundingVolume.";
    }
}

/** Bounding volume may be (by default) invalid, so here comes special
 *  "optionsl" handling.
 */
void parse(BoundingVolume &bv, const Json::Value &value, const char *member)
{
    if (value.isMember(member)) {
        parse(bv, value[member]);
    }
}

void parse(Refinement &refinement, const Json::Value &value)
{
    refinement = boost::lexical_cast<Refinement>(value.asString());
}

void parse(math::Matrix4 &matrix, const Json::Value &value)
{
    if ((value.type() != Json::arrayValue) || (value.size() != 16)) {
        LOGTHROW(err1, Json::Error)
            << "Transformation matrix doesn't have 16 elements (but"
            << value.size() << ").";
    }

    matrix = boost::numeric::ublas::zero_matrix<double>(4, 4);

    // read matrix as a column major order
    for (int i(0), column(0); column < 4; ++column) {
        for (int row(0); row < 4; ++row, ++i) {
            matrix(row, column) = value[i].asDouble();
        }
    }
}

void parse(TileContent &content, const Json::Value &value)
{
    common(content, value);
    // support both 1.0 uri and 0.0 url
    Json::get(content.uri, value, { "uri", "url" });
    parse(content.boundingVolume, value, "boundingVolume");
}

void parse(Tile &tile, const Json::Value &value)
{
    common(tile, value);
    parse(tile.boundingVolume, value["boundingVolume"]);

    parse(tile.viewerRequestVolume, value, "viewerRequestVolume");
    parse(tile.content, value, "content");

    // treat geometric-error as optional, do not warn if it seams that tile is
    // an external tileset
    if (!Json::getOpt(tile.geometricError, value, "geometricError")) {
        if (!tile.content || !ba::iends_with(tile.content->uri, ".json")) {
            LOG(warn2) << "Missing geometricError, skipping.";
        }
    }

    parse(tile.refine, value, "refine");
    parse(tile.transform, value, "transform");
    parse(tile.children, value, "children");
}

void parse(std::vector<Tile::pointer> &tiles, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Expected JSON array.";
    }

    for (const auto &item : value) {
        tiles.push_back(std::make_unique<Tile>());
        parse(*tiles.back(), item);
    }
}

void parse(Asset &asset, const Json::Value &value)
{
    Json::get(asset.version, value, "version");
    Json::get(asset.tilesetVersion, value, "tilesetVersion");
    // TODO: gltfUpAxis (version 0.0)
}

void parse(Tileset &tileset, const Json::Value &value)
{
    common(tileset, value);

    parse(tileset.asset, value["asset"]);
    // TODO: properties
    parse(*(tileset.root = std::make_unique<Tile>()), value["root"]);
    Json::get(tileset.geometricError, value, "geometricError");

    parse(tileset.extensionsUsed, value, "extensionsUsed");
    parse(tileset.extensionsRequired, value, "extensionsRequired");
}

template <typename T>
void parse(boost::optional<T> &dst, const Json::Value &value
           , const char *member)
{
    if (value.isMember(member)) {
        parse(*(dst = boost::in_place()), value[member]);
    }
}

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const char *member)
{
    if (value.isMember(member)) {
        parse(dst, value[member]);
    }
}

} // namespace detail

void read(std::istream &is, Tileset &tileset, const fs::path &path)
{
    auto content(Json::read(is, path, "3D Tileset"));
    detail::parse(tileset, content);

}

Tileset& absolutize(Tileset &ts, const std::string &baseUri, bool relaxed)
{
    auto &root(*ts.root);

    // sanity check
    if (!relaxed && !root.refine) {
        LOGTHROW(err2, std::runtime_error)
            << "Root tile in tileset <" << baseUri
            << "> has no refinement defined.";
    }

    const utility::Uri base(baseUri);

    // make sure root tile has a valid transformation matrix
    if (!root.transform) {
        root.transform = math::identity4();
    }

    traverse(*ts.root, [&](Tile &tile, const TilePath&)
    {
        // resolve URI
        if (auto &content = tile.content) {
            content->uri = str(base.resolve(utility::Uri(content->uri)));
        }

        // distribute refinement and transformation down the tree
        for (auto &child : tile.children) {
            if (!child->refine) { child->refine = tile.refine; }

            if (child->transform) {
                // compose transformation (child's trafor is applied first)
                child->transform = prod(*tile.transform, *child->transform);
            } else {
                // distribute
                child->transform = tile.transform;
            }
        }
    });

    return ts;
}

namespace {

void cloneInto(CommonBase &dst, const CommonBase &src)
{
    dst = src;
}

Tile::pointer clone(const Tile &tile)
{
    auto ntile(std::make_unique<Tile>());
    cloneInto(*ntile, tile);

    ntile->boundingVolume = tile.boundingVolume;
    ntile->viewerRequestVolume = tile.viewerRequestVolume;
    ntile->geometricError = tile.geometricError;
    ntile->refine = tile.refine;
    ntile->transform = tile.transform;
    ntile->content = tile.content;

    for (const auto &child : tile.children) {
        ntile->children.push_back(clone(*child));
    }

    return ntile;
}

} // namespace

Tileset clone(const Tileset &ts)
{
    Tileset nts;
    nts.asset = ts.asset;
    nts.properties = ts.properties;
    nts.geometricError = ts.geometricError;
    nts.root = clone(*ts.root);
    nts.extensionsUsed = ts.extensionsUsed;
    nts.extensionsRequired = ts.extensionsRequired;
    return nts;
}

TilesetWithUri::list split(Tileset &tileset, std::size_t tileLimit)
{
    class Splitter {
    public:
        Splitter(Tileset &tileset, std::size_t tileLimit)
            : tileset_(tileset), tileLimit_(tileLimit)
        {
            process(tileset_.root
                    , boost::get_optional_value_or
                    (tileset_.root->refine, Refinement::replace));
        }

        TilesetWithUri::list subtrees() { return std::move(subtrees_); }

    private:
        std::size_t process(const Tile::pointer &tile, Refinement refine)
        {
            int i(0);

            std::size_t size(1);
            for (auto &child : tile->children) {
                path_.path.push_back(i++);

                auto childRefine(boost::get_optional_value_or
                                 (child->refine, refine));

                auto stSize(process(child, childRefine));
                if (stSize >= tileLimit_) {
                    LOG(info2)
                        << "Will split at <" << path_
                        << ">, subtree size: " << stSize;

                    cut(child, path_, childRefine);
                    // count in replaced child
                    stSize = 1;
                }

                size += stSize;
                path_.path.pop_back();
            }
            return size;
        }

        void cut(Tile::pointer &root, const TilePath &path
                 , Refinement refine)
        {
            const auto uri(utility::format("tileset-%s.json", path));

            subtrees_.emplace_back(uri);
            auto &ts(subtrees_.back().tileset);

            // copy data from original tileset, set root
            ts.asset = tileset_.asset;
            ts.properties = tileset_.properties;
            ts.geometricError = 2 * root->geometricError;
            ts.root = std::move(root);
            ts.root->refine = refine;
            ts.extensionsUsed = tileset_.extensionsUsed;
            ts.extensionsRequired = tileset_.extensionsRequired;

            // regenerate root
            root = std::make_unique<Tile>();
            root->geometricError = ts.root->geometricError;
            root->boundingVolume = ts.root->boundingVolume;
            root->content = boost::in_place();
            root->content->boundingVolume = ts.root->boundingVolume;
            root->content->uri = uri;

            // copy extensions and extras
            root->extensions = ts.root->extensions;
            root->extras = ts.root->extras;
        }

        Tileset &tileset_;
        std::size_t tileLimit_;

        TilePath path_;

        TilesetWithUri::list subtrees_;
    };

    return Splitter(tileset, tileLimit).subtrees();
}

namespace {

bool inside(const Box &inner, const Box &outer)
{
    LOG(warn2)
        << "Matching bounding-box bounding volumes not implemented. Yet.";
    return false;
    (void) inner;
    (void) outer;
}

bool inside(const Region &inner, const Region &outer)
{
    return (math::inside(outer.extents, inner.extents.ll)
            && math::inside(outer.extents, inner.extents.ur));
}

bool inside(const Sphere &inner, const Sphere &outer)
{
    LOG(warn2)
        << "Matching sphere bounding volumes not implemented. Yet.";

    return false;
    (void) inner;
    (void) outer;
}

} // namespace

bool inside(const BoundingVolume &inner, const BoundingVolume &outer)
{
    if (inner.which() != outer.which()) {
        LOG(warn2)
            << "Matching different bounding volume types "
            "not implemented. Yet.";
        return false;
    }

    struct Matcher : public boost::static_visitor<bool> {
        const BoundingVolume &outer;

        Matcher(const BoundingVolume &outer) : outer(outer) { }

        bool operator()(const Box &b) const {
            return inside(b, boost::get<const Box&>(outer));
        }

        bool operator()(const Region &r) const {
            return inside(r, boost::get<const Region&>(outer));
        }

        bool operator()(const Sphere &s) const {
            return inside(s, boost::get<const Sphere&>(outer));
        }

        bool operator()(const boost::blank&) {
            return true;
        }
    } matcher(outer);
    return boost::apply_visitor(matcher, inner);
}

std::ostream& operator<<(std::ostream &os, const Box &b)
{
    return os << "Box[center=" << b.center
       << ", x=" << b.x
       << ", y=" << b.y
       << ", z=" << b.z
       << "]";
}

std::ostream& operator<<(std::ostream &os, const Region &r)
{
    return os << "Region[" << r.extents << "]";
}

std::ostream& operator<<(std::ostream &os, const Sphere &s)
{
    return os << "Sphere[center=" << s.center
              << ", radius=" << s.radius
              << "]";
}

std::ostream& operator<<(std::ostream &os, const BoundingVolume &bv)
{
    struct Printer : public boost::static_visitor<void> {
        std::ostream &os;
        Printer(std::ostream &os) : os(os) {}

        void operator()(const Box &b) {
            os << b;
        }

        void operator()(const Region &r) {
            os << r;
        }

        void operator()(const Sphere &s) {
            os << s;
        }

        void operator()(const boost::blank&) {
            os << "[Invalid Bounding Volume]";
        }
    } printer(os);
    boost::apply_visitor(printer, bv);
    return os;
}

namespace dump {

template <typename TileT, typename Op>
void traverseTiles(TileT &root, Op op) {
    struct Traverser {
        Op op;
        Traverser(Op op) : op(op) {}
        void process(TileT &tile, TilePath &path, TileT *parent = nullptr)
        {
            op(tile, path, parent);

            int i(0);
            for (const auto &child : tile.children) {
                path.path.push_back(i++);
                process(*child, path, &tile);
                path.path.pop_back();
            }
        }
    } t(op);

    TilePath path;
    t.process(root, path);
}

bool checkExtents(const Tile &tile, const Tile *parent)
{
    if (!parent) { return true; }

    return inside(tile.boundingVolume, parent->boundingVolume);
}

} // namespace dump

void dumpMetadata(std::ostream &os, const Tileset &ts)
{
    dump::traverseTiles(*ts.root, [&os](const Tile &tile, const TilePath &path
                                        , const Tile *parent)
    {
        os
            << path.depth() << "\t"
            << utility::join(path.path, "-", "root") << "\textents: "
            << tile.boundingVolume;

        if (!dump::checkExtents(tile, parent)) {
            os << " out-of-bounds!";
        }

        if (tile.content) {
            os << "\tcontent: " << tile.content->uri;
        } else {
            os << "\tno content";
        }

        os << "\n";
    });
}

} // namespace threedtiles
