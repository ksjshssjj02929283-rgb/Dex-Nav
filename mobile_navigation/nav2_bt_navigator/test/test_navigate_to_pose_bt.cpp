// Copyright 2026 DEXE Robot Engineer
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <string>
#include <vector>

namespace
{

class NavigateToPoseBTTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
#ifdef BT_SOURCE_DIR
    xml_path_ = std::string(BT_SOURCE_DIR) +
      "/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml";
#else
    xml_path_ = "behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml";
#endif
    ASSERT_EQ(doc_.LoadFile(xml_path_.c_str()), tinyxml2::XML_SUCCESS)
      << "Failed to load behavior tree XML from: " << xml_path_;
  }

  // 递归遍历查找所有具有特定 tag 名的元素
  void findElementsByTagName(
    const tinyxml2::XMLElement * element,
    const std::string & tag_name,
    std::vector<const tinyxml2::XMLElement *> & results)
  {
    if (!element) {
      return;
    }
    if (std::string(element->Value()) == tag_name) {
      results.push_back(element);
    }
    for (const auto * child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
      findElementsByTagName(child, tag_name, results);
    }
  }

  // 递归遍历查找所有具有特定 name 属性的元素
  const tinyxml2::XMLElement * findElementByNameAttribute(
    const tinyxml2::XMLElement * element,
    const std::string & name_attr)
  {
    if (!element) {
      return nullptr;
    }
    const char * name = element->Attribute("name");
    if (name && std::string(name) == name_attr) {
      return element;
    }
    for (const auto * child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
      const auto * found = findElementByNameAttribute(child, name_attr);
      if (found) {
        return found;
      }
    }
    return nullptr;
  }

  std::string xml_path_;
  tinyxml2::XMLDocument doc_;
};

// 1. 验证全局路径规划外层重试次数与任务级 30s 规划期限配置一致
TEST_F(NavigateToPoseBTTest, GlobalPlanningOuterRetryCountIsHundred)
{
  const auto * root = doc_.RootElement();
  ASSERT_NE(root, nullptr);

  const auto * outer_retry = findElementByNameAttribute(root, "ComputePathToPoseOuterRetry");
  ASSERT_NE(outer_retry, nullptr)
    << "Could not find <RecoveryNode name=\"ComputePathToPoseOuterRetry\"> in behavior tree XML";

  EXPECT_STREQ(outer_retry->Value(), "RecoveryNode");

  const char * retries_attr = outer_retry->Attribute("number_of_retries");
  ASSERT_NE(retries_attr, nullptr)
    << "ComputePathToPoseOuterRetry must specify 'number_of_retries' attribute";

  EXPECT_STREQ(retries_attr, "100")
    << "Global planning outer retry count must be exactly 100";
}

// 2. 验证行为树中彻底不包含任何 BackUp 后退动作节点（杜绝倒车盲区风险）
TEST_F(NavigateToPoseBTTest, BackUpActionCompletelyRemoved)
{
  const auto * root = doc_.RootElement();
  ASSERT_NE(root, nullptr);

  std::vector<const tinyxml2::XMLElement *> backup_nodes;
  findElementsByTagName(root, "BackUp", backup_nodes);

  EXPECT_TRUE(backup_nodes.empty())
    << "Found " << backup_nodes.size()
    << " <BackUp> nodes in behavior tree XML! BackUp action must be completely removed.";

  const auto * backup_retry_node = findElementByNameAttribute(root, "FollowPathAfterBackUpRetry");
  EXPECT_EQ(backup_retry_node, nullptr)
    << "Found legacy <FollowPathAfterBackUpRetry> node! It should be removed.";

  const auto * backup_action = findElementByNameAttribute(root, "BackUpAfterLocalRetries");
  EXPECT_EQ(backup_action, nullptr)
    << "Found legacy <BackUpAfterLocalRetries> node! It should be removed.";
}

// 3. 验证局部控制保留 1 次原地恢复重规划
TEST_F(NavigateToPoseBTTest, LocalControlRecoveryUsesSingleReplan)
{
  const auto * root = doc_.RootElement();
  ASSERT_NE(root, nullptr);

  const auto * inner_retry = findElementByNameAttribute(root, "FollowPathInnerRetry");
  ASSERT_NE(inner_retry, nullptr)
    << "Could not find <RecoveryNode name=\"FollowPathInnerRetry\"> in behavior tree XML";

  EXPECT_STREQ(inner_retry->Value(), "RecoveryNode");
  EXPECT_STREQ(inner_retry->Attribute("number_of_retries"), "1")
    << "FollowPath failure must keep exactly one local recovery/replan attempt";

  // 验证恢复序列为 WaitStopBeforeFollowPathRetry
  const auto * wait_stop_seq = findElementByNameAttribute(inner_retry, "WaitStopBeforeFollowPathRetry");
  ASSERT_NE(wait_stop_seq, nullptr);
  EXPECT_STREQ(wait_stop_seq->Value(), "Sequence");

  // 验证包含停稳等待 Wait (wait_duration=0.5s)
  std::vector<const tinyxml2::XMLElement *> wait_nodes;
  findElementsByTagName(wait_stop_seq, "Wait", wait_nodes);
  ASSERT_FALSE(wait_nodes.empty());
  EXPECT_STREQ(wait_nodes[0]->Attribute("wait_duration"), "0.5");

  // 验证包含全局重规划请求 RequestAndWaitForGlobalReplan (reason=follow_path_failed)
  std::vector<const tinyxml2::XMLElement *> replan_nodes;
  findElementsByTagName(wait_stop_seq, "RequestAndWaitForGlobalReplan", replan_nodes);
  ASSERT_FALSE(replan_nodes.empty());
  EXPECT_STREQ(replan_nodes[0]->Attribute("reason"), "follow_path_failed");
}

}  // namespace
