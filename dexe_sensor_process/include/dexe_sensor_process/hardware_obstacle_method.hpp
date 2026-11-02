#pragma once

#include <regex>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace dexe_sensor_process {

// 按整数版本分量比较，不能用浮点数比较硬件版本（例如 v0.100 > v0.24）。
// 硬件文件约定采用 v<主版本>.<次版本>，不接受残缺版本或其他后缀。
inline std::string obstacleMethodForHardware(const YAML::Node& hardware) {
    const std::string revision = hardware["hardware_revision"].as<std::string>();
    static const std::regex pattern(R"(^v([0-9]+)\.([0-9]+)$)");
    std::smatch match;
    if (!std::regex_match(revision, match, pattern)) {
        throw std::invalid_argument("Invalid hardware_revision: " + revision);
    }
    const auto major = std::stoull(match[1].str());
    const auto minor = std::stoull(match[2].str());
    return major == 0 && minor <= 24 ? "height_filter" : "uv_disparity";
}

struct ObstacleMethodSelection {
    std::string method;
    std::string revision;
    std::string error;
};

// 启动路径与测试共用同一选择逻辑；路径可注入，测试无需修改系统硬件文件。
// 显式指定算法不读取文件；auto 读取失败时保留诊断原因并回退高度法。
inline ObstacleMethodSelection selectObstacleMethod(
    const std::string& requested_method,
    const std::string& hardware_path) {
    if (requested_method != "auto") {
        return {requested_method, "", ""};
    }
    try {
        const auto hardware = YAML::LoadFile(hardware_path);
        return {obstacleMethodForHardware(hardware),
                hardware["hardware_revision"].as<std::string>(), ""};
    } catch (const std::exception& ex) {
        return {"height_filter", "", ex.what()};
    }
}

}  // namespace dexe_sensor_process
