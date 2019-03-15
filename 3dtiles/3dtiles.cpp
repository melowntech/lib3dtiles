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

#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/base64.hpp"
#include "utility/streams.hpp"
#include "utility/format.hpp"

#include "jsoncpp/io.hpp"

#include "3dtiles.hpp"

namespace threedtiles {

namespace detail {

void build(Json::Value &value, const Tile &tile);

template <typename T>
typename std::enable_if<!std::is_enum<T>::value, void>::type
build(Json::Value &object, const T &value);

template <typename T>
void build(Json::Value &object, const char *name
           , const boost::optional<T> &value);

template <typename T>
void build(Json::Value &object, const char *name
           , const std::shared_ptr<T> &value);

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
    } builder(value);
    boost::apply_visitor(builder, bv);
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
           , const std::shared_ptr<T> &value)
{
    if (value) { build(object[name], *value); }
}

template <typename T>
void build(Json::Value &object, const std::shared_ptr<T> &value)
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
    build(value, "boundingVolume", tile.boundingVolume);
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

void write(const boost::filesystem::path &path, const Tileset &tileset)
{
    LOG(info1) << "Saving Tileset to " << path  << ".";
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(path.string(), std::ios_base::out);
    write(f, tileset);
    f.close();
}

void update(boost::optional<BoundingVolume> &updated
            , const boost::optional<BoundingVolume> &updater)
{
    if (!updater) { return; }

    if (!updated) {
        updated = updater;
        return;
    }

    if (detail::update<Box>(*updated, *updater)) { return; }
    if (detail::update<Region>(*updated, *updater)) { return; }
    if (detail::update<Sphere>(*updated, *updater)) { return; }
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

} // namespace threedtiles

