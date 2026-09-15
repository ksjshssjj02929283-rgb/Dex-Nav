#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <utility>
#include "hardware_obstacle_method.hpp"

using dexe_sensor_process::obstacleMethodForHardware;

TEST(HardwareObstacleMethod, OldHardwareUsesHeightFilter) {
    for (const auto* revision : {"v0.0", "v0.21", "v0.22", "v0.23", "v0.24"}) {
        YAML::Node hardware;
        hardware["hardware_revision"] = revision;
        EXPECT_EQ(obstacleMethodForHardware(hardware), "height_filter") << revision;
    }
}

TEST(HardwareObstacleMethod, NewHardwareUsesUV) {
    for (const auto* revision : {"v0.25", "v0.30", "v0.100", "v1.0"}) {
        YAML::Node hardware;
        hardware["hardware_revision"] = revision;
        EXPECT_EQ(obstacleMethodForHardware(hardware), "uv_disparity") << revision;
    }
}

TEST(HardwareObstacleMethod, ParsesQuotedYamlWithComment) {
    EXPECT_EQ(obstacleMethodForHardware(YAML::Load(
        "hardware_revision: \"v0.30\" # 整机硬件版本\n")), "uv_disparity");
}

TEST(HardwareObstacleMethod, RejectsMissingOrInvalidRevision) {
    for (const auto* yaml : {"{}", "hardware_revision: null", "hardware_revision: []",
                            "hardware_revision: v0", "hardware_revision: v0.24beta",
                            "hardware_revision: -0.24", "hardware_revision: 0.24",
                            "hardware_revision: v0.9999999999999999999999999999"}) {
        EXPECT_THROW(obstacleMethodForHardware(YAML::Load(yaml)), std::exception) << yaml;
    }
}

// 正常和异常配置读取 test/data 下的固定样例；临时目录仅用于文件缺失场景。
class HardwareObstacleFileTest : public ::testing::Test {
protected:
    void SetUp() override {
        char pattern[] = "/tmp/dexe_hardware_test_XXXXXX";
        const char* directory = mkdtemp(pattern);
        ASSERT_NE(directory, nullptr);
        directory_ = directory;
        path_ = directory_ / "hardware.yaml";
    }

    void TearDown() override {
        if (!directory_.empty()) {
            std::filesystem::remove_all(directory_);
        }
    }

    std::string fixturePath(const std::string& filename) const {
        return (std::filesystem::path(HARDWARE_TEST_DATA_DIR) / filename).string();
    }

    std::filesystem::path directory_;
    std::filesystem::path path_;
};

TEST_F(HardwareObstacleFileTest, AutoReadsHardwareVersionsFromFile) {
    const std::pair<const char*, const char*> cases[] = {
        {"v0.21", "height_filter"}, {"v0.22", "height_filter"},
        {"v0.23", "height_filter"}, {"v0.24", "height_filter"},
        {"v0.25", "uv_disparity"}, {"v0.30", "uv_disparity"},
        {"v0.100", "uv_disparity"}, {"v1.0", "uv_disparity"}
    };
    for (const auto& [revision, expected_method] : cases) {
        SCOPED_TRACE(revision);
        const std::string filename = std::string(revision) == "v0.30"
            ? "hardware.yaml" : std::string("hardware_") + revision + ".yaml";
        const auto result = dexe_sensor_process::selectObstacleMethod("auto", fixturePath(filename));
        EXPECT_EQ(result.method, expected_method);
        EXPECT_EQ(result.revision, revision);
        EXPECT_TRUE(result.error.empty());
    }
}

TEST_F(HardwareObstacleFileTest, MissingFileFallsBackWithDiagnostic) {
    const auto result = dexe_sensor_process::selectObstacleMethod("auto", path_.string());
    EXPECT_EQ(result.method, "height_filter");
    EXPECT_TRUE(result.revision.empty());
    EXPECT_FALSE(result.error.empty());
}

TEST_F(HardwareObstacleFileTest, InvalidFileFallsBackWithDiagnostic) {
    for (const auto* filename : {"hardware_empty.yaml", "hardware_malformed.yaml",
                                 "hardware_null.yaml", "hardware_missing_revision.yaml",
                                 "hardware_invalid_revision.yaml", "hardware_overflow.yaml"}) {
        SCOPED_TRACE(filename);
        ASSERT_TRUE(std::filesystem::is_regular_file(fixturePath(filename)));
        const auto result = dexe_sensor_process::selectObstacleMethod("auto", fixturePath(filename));
        EXPECT_EQ(result.method, "height_filter");
        EXPECT_TRUE(result.revision.empty());
        EXPECT_FALSE(result.error.empty());
    }
}

TEST_F(HardwareObstacleFileTest, ExplicitMethodWorksWithoutHardwareFile) {
    for (const auto* method : {"height_filter", "uv_disparity"}) {
        const auto result = dexe_sensor_process::selectObstacleMethod(method, path_.string());
        EXPECT_EQ(result.method, method);
        EXPECT_TRUE(result.revision.empty());
        EXPECT_TRUE(result.error.empty());
    }
}

TEST_F(HardwareObstacleFileTest, ExplicitMethodOverridesHardwareChoice) {
    EXPECT_EQ(dexe_sensor_process::selectObstacleMethod("height_filter", fixturePath("hardware.yaml")).method,
              "height_filter");
    EXPECT_EQ(dexe_sensor_process::selectObstacleMethod("uv_disparity", fixturePath("hardware_v0.24.yaml")).method,
              "uv_disparity");
}
