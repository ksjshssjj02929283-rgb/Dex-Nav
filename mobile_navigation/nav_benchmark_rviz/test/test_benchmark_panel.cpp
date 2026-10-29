// 面板离屏单测：不连 ROS，直接喂状态 JSON、点按钮，核对发出的命令和界面显示
#include <gtest/gtest.h>

#include <QApplication>
#include <QCheckBox>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QTableWidget>

#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>

#include "nav_benchmark_rviz/benchmark_panel.hpp"

using nav_benchmark_rviz::BenchmarkPanel;

namespace {

QJsonObject makeStatus(bool running, const QJsonArray& rows = QJsonArray()) {
    QJsonObject assets{{"map", "office_5f"}, {"points", QJsonArray{"P1", "P2"}}, {"routes", QJsonArray{"R1", "R2"}}};
    QJsonObject task{{"running", running}, {"index", 1}, {"total", 2}, {"current", "R1"}, {"state", "RUNNING"},
                     {"distance", 3.5}, {"rows", rows},
                     {"stats", QJsonObject{{"total", rows.size()}, {"ok", rows.size()}, {"fail", 0}}}};
    QJsonObject tiles{{"loc", QJsonObject{{"status", "4 LOC_SUCCESS"}, {"match_level", "1 MATCH_GOOD"}, {"score", "0.80"}}},
                      {"sys", QJsonObject{{"estop", false}, {"battery", 67}, {"map", "office_5f"},
                                          {"_sensors", QJsonArray{QJsonObject{{"name", "imu"}, {"state", 1}, {"rate_hz", 100.0}}}}}},
                      {"backend", QJsonObject{{"backend", "nav2"}}}};
    return QJsonObject{{"version", 1}, {"serve_version", "0.1.0"}, {"time", "2026-10-26 16:00:00.000"},
                       {"assets", assets}, {"tiles", tiles}, {"task", task}, {"nav_ready", true}};
}

struct Sink {
    QList<QJsonObject> cmds;
    std::function<void(const QString&)> fn() {
        return [this](const QString& s) { cmds << QJsonDocument::fromJson(s.toUtf8()).object(); };
    }
};

}  // namespace

TEST(BenchmarkPanel, TargetsFollowAssets) {
    BenchmarkPanel p;
    p.applyStatus(makeStatus(false));
    ASSERT_EQ(p.targetList()->count(), 2);   // 默认「路线」
    EXPECT_EQ(p.targetList()->item(0)->text(), "R1");
}

TEST(BenchmarkPanel, RunNeedsClearConfirmAndTargets) {
    BenchmarkPanel p;
    Sink sink;
    p.setCommandSink(sink.fn());
    p.applyStatus(makeStatus(false));
    p.runButton()->click();
    EXPECT_TRUE(sink.cmds.isEmpty());
    EXPECT_TRUE(p.hintLabel()->text().contains("清场"));
    p.clearCheck()->setChecked(true);
    p.runButton()->click();
    EXPECT_TRUE(sink.cmds.isEmpty());   // 没选目标
    p.targetList()->item(1)->setSelected(true);
    p.runButton()->click();
    ASSERT_EQ(sink.cmds.size(), 1);
    const QJsonObject c = sink.cmds[0];
    EXPECT_EQ(c.value("version").toInt(), 1);
    EXPECT_EQ(c.value("op").toString(), "nav_run");
    const QJsonObject a = c.value("args").toObject();
    EXPECT_EQ(a.value("kind").toString(), "route");
    EXPECT_EQ(a.value("targets").toArray().at(0).toString(), "R2");
    EXPECT_TRUE(a.value("confirm_clear").toBool());
}

TEST(BenchmarkPanel, ClearConfirmResetsAfterSequence) {
    BenchmarkPanel p;
    p.clearCheck()->setChecked(true);
    p.applyStatus(makeStatus(true));
    EXPECT_FALSE(p.runButton()->isEnabled());
    p.applyStatus(makeStatus(false, QJsonArray{QJsonObject{{"target", "R1"}, {"success", true}, {"elapsed_s", "12.3"}}}));
    EXPECT_TRUE(p.runButton()->isEnabled());
    EXPECT_FALSE(p.clearCheck()->isChecked());
}

TEST(BenchmarkPanel, HardStopAlwaysSends) {
    BenchmarkPanel p;
    Sink sink;
    p.setCommandSink(sink.fn());
    p.hardStopButton()->click();
    ASSERT_EQ(sink.cmds.size(), 1);
    EXPECT_EQ(sink.cmds[0].value("op").toString(), "hard_stop");
}

TEST(BenchmarkPanel, PreflightTable) {
    BenchmarkPanel p;
    QJsonObject st = makeStatus(false);
    st["preflight"] = QJsonObject{{"items", QJsonArray{QJsonObject{{"key", "localization"}, {"level", "fail"}, {"detail", "status=3"}},
                                                       QJsonObject{{"key", "battery"}, {"level", "ok"}, {"detail", "67%"}}}}};
    p.applyStatus(st);
    ASSERT_EQ(p.preflightTable()->rowCount(), 2);
    EXPECT_EQ(p.preflightTable()->item(0, 0)->text(), "定位");
    EXPECT_EQ(p.preflightTable()->item(0, 1)->text(), "不通过");
}

TEST(BenchmarkPanel, ProtocolMismatchIgnoresStatus) {
    BenchmarkPanel p;
    QJsonObject st = makeStatus(false);
    st["version"] = 2;
    p.applyStatus(st);
    EXPECT_EQ(p.targetList()->count(), 0);
}

TEST(BenchmarkPanel, EventLines) {
    BenchmarkPanel p;
    p.appendEvent(QJsonObject{{"t", "2026-10-26 16:00:01.000"}, {"type", "mark"}, {"text", "放障碍"}});
    p.appendEvent(QJsonObject{{"t", "2026-10-26 16:00:02.000"}, {"type", "state"}, {"group", "obs"},
                              {"changes", QJsonObject{{"status", QJsonArray{"0 FREE", "2 STOP"}}}}});
    ASSERT_EQ(p.eventList()->count(), 2);
    EXPECT_TRUE(p.eventList()->item(0)->text().contains("0 FREE → 2 STOP"));
    EXPECT_TRUE(p.eventList()->item(1)->text().contains("打点：放障碍"));
}

TEST(BenchmarkPanel, PluginLoadable) {
    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    const auto classes = loader.getDeclaredClasses();
    ASSERT_NE(std::find(classes.begin(), classes.end(), "nav_benchmark_rviz/BenchmarkPanel"), classes.end());
    auto panel = loader.createUniqueInstance("nav_benchmark_rviz/BenchmarkPanel");
    EXPECT_TRUE(panel != nullptr);
}

// 契约测试：用 nbt serve 实际发出的 /nbt/status（test/data/status_sample.json）驱动面板
TEST(BenchmarkPanel, RealServeStatusContract) {
    QFile f(QString(TEST_DATA_DIR) + "/status_sample.json");
    ASSERT_TRUE(f.open(QIODevice::ReadOnly));
    const QJsonObject st = QJsonDocument::fromJson(f.readAll()).object();
    ASSERT_EQ(st.value("version").toInt(), 1);
    BenchmarkPanel p;
    p.applyStatus(st);
    EXPECT_EQ(p.targetList()->count(), st.value("assets").toObject().value("routes").toArray().size());
    EXPECT_EQ(p.preflightTable()->rowCount(),
              st.value("preflight").toObject().value("items").toArray().size());
    EXPECT_GT(p.preflightTable()->rowCount(), 5);
    for (int i = 0; i < p.preflightTable()->rowCount(); ++i) {
        EXPECT_FALSE(p.preflightTable()->item(i, 0)->text().contains('_'));   // 每个检查项都有中文名
    }
}

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
