#include "map_asset_manager/asset_set.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>

#include "map_asset_manager/polygon_utils.hpp"

namespace map_asset_manager {

using json = nlohmann::json;

std::vector<RouteJunction> AssetSet::junctionList() const {
    std::vector<RouteJunction> v;
    v.reserve(junctions.size());
    for (const auto& kv : junctions) v.push_back(kv.second);
    std::sort(v.begin(), v.end(), [](const RouteJunction& a, const RouteJunction& b) { return a.id < b.id; });
    return v;
}

std::vector<RouteEdge> AssetSet::edgeList() const {
    std::vector<RouteEdge> v;
    v.reserve(edges.size());
    for (const auto& kv : edges) v.push_back(kv.second);
    std::sort(v.begin(), v.end(), [](const RouteEdge& a, const RouteEdge& b) { return a.id < b.id; });
    return v;
}

bool LoadAssetSet(FileManager& fm, const std::string& assets_file, AssetSet& out) {
    out = AssetSet{};
    std::vector<RouteJunction> jcts;
    std::vector<RouteEdge> edges;
    if (!fm.loadAllAssetsFromFile(assets_file, out.points, out.routes, out.areas, jcts, edges, out.rooms)) {
        return false;
    }
    for (auto& j : jcts) out.junctions[j.id] = std::move(j);
    for (auto& e : edges) out.edges[e.id] = std::move(e);
    return true;
}

bool SaveAssetSet(FileManager& fm, const std::string& assets_file, const AssetSet& set) {
    const std::string rev = ComputeAssetsRevision(set);
    std::error_code ec;
    if (std::filesystem::exists(assets_file, ec)) {
        AssetSet disk;
        if (LoadAssetSet(fm, assets_file, disk) && ComputeAssetsRevision(disk) == rev) return true;
    } else if (rev == ComputeAssetsRevision(AssetSet{})) {
        return true;
    }
    return fm.saveAllAssetsToFile(assets_file, set.points, set.routes, set.areas, set.junctionList(), set.edgeList(),
                                  set.rooms);
}

namespace {

void SortArrayBy(json& arr, const char* key) {
    if (!arr.is_array()) return;
    std::sort(arr.begin(), arr.end(), [key](const json& a, const json& b) {
        return a.value(key, std::string()) < b.value(key, std::string());
    });
}

}  // namespace

std::string ComputeAssetsRevision(const AssetSet& set) {
    json j = FileManager::buildAssetsJson(set.points, set.routes, set.areas, set.junctionList(), set.edgeList(),
                                          set.rooms);
    SortArrayBy(j["points"], "name");
    SortArrayBy(j["routes"], "name");
    SortArrayBy(j["areas"], "name");
    SortArrayBy(j["route_graph"]["junctions"], "id");
    SortArrayBy(j["route_graph"]["edges"], "id");
    SortArrayBy(j["dsg"]["layers"]["ROOMS"], "name");
    const std::string text = j.dump();
    uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : text) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(h));
    return buf;
}

void RecomputeRelated(AssetSet& set) {
    auto inside = [&](const std::vector<geometry_msgs::msg::Point>& poly, std::vector<std::string>& names,
                      std::vector<std::string>& jct_ids) {
        names.clear();
        jct_ids.clear();
        if (poly.size() < 3) return;
        for (const auto& kv : set.points)
            if (polygon_utils::isPointInPolygon(kv.second.x, kv.second.y, poly)) names.push_back(kv.first);
        for (const auto& kv : set.junctions)
            if (polygon_utils::isPointInPolygon(kv.second.x, kv.second.y, poly)) jct_ids.push_back(kv.first);
        std::sort(names.begin(), names.end());
        std::sort(jct_ids.begin(), jct_ids.end());
    };
    for (auto& kv : set.areas) inside(kv.second.polygon, kv.second.related_nav_point_names, kv.second.related_junction_ids);
    for (auto& kv : set.rooms) inside(kv.second.polygon, kv.second.related_nav_point_names, kv.second.related_junction_ids);
}

}  // namespace map_asset_manager
