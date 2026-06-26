#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <end_effector_interfaces/msg/ee_feedback.hpp>
#include <end_effector_interfaces/msg/ee_joint_control.hpp>
#include <end_effector_interfaces/msg/ee_joint_control_mode.hpp>

#include <nlohmann/json.hpp>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <memory>
#include <algorithm>

using json = nlohmann::json;
using namespace std::chrono_literals;

class JointConverter : public rclcpp::Node
{
public:
    JointConverter() : Node("joint_converter"), publish_rate_hz_(10)
    {
        // 声明参数
        this->declare_parameter("use_simulation", false);
        bool use_simulation = this->get_parameter("use_simulation").as_bool();
        
        // 根据仿真模式设置话题名
        std::string general_input_topic, left_hand_topic, right_hand_topic;
        if (use_simulation) {
            general_input_topic = "/feedback/robot_server_state_sim";
            left_hand_topic = "/feedback_sim/ee/left";
            right_hand_topic = "/feedback_sim/ee/right";
            RCLCPP_INFO(this->get_logger(), "启动仿真模式");
        } else {
            general_input_topic = "/feedback/robot_server_state";
            left_hand_topic = "/feedback/ee/left";
            right_hand_topic = "/feedback/ee/right";
            RCLCPP_INFO(this->get_logger(), "启动真实模式");
        }

        // 创建QoS配置
        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();

        // 订阅通用关节状态
        general_subscription_ = this->create_subscription<std_msgs::msg::String>(
            general_input_topic, qos_profile,
            std::bind(&JointConverter::general_callback, this, std::placeholders::_1));

        // 订阅左右手反馈
        left_hand_subscription_ = this->create_subscription<end_effector_interfaces::msg::EEFeedback>(
            left_hand_topic, 10,
            std::bind(&JointConverter::left_hand_callback, this, std::placeholders::_1));

        right_hand_subscription_ = this->create_subscription<end_effector_interfaces::msg::EEFeedback>(
            right_hand_topic, 10,
            std::bind(&JointConverter::right_hand_callback, this, std::placeholders::_1));

        // 发布合并后的 JointState
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 设置定时器，10Hz发布频率
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / publish_rate_hz_),
            std::bind(&JointConverter::timer_callback, this));

        // 初始化关节名称列表（按URDF顺序排列）
        initialize_joint_names();
        
        // 初始化关节消息
        joint_state_msg_.name = all_joint_names_;
        joint_state_msg_.position.resize(all_joint_names_.size(), 0.0);

        RCLCPP_INFO(this->get_logger(), "订阅话题: %s, %s, %s", 
                   general_input_topic.c_str(), left_hand_topic.c_str(), right_hand_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: /joint_states, 频率: %d Hz", publish_rate_hz_);
        RCLCPP_INFO(this->get_logger(), "JointConverter 节点初始化完成");
    }

private:
    void initialize_joint_names()
    {
        // 通用关节（按URDF顺序）
        std::vector<std::string> general_joints = {
            "ANKLE", "KNEE", "BUTTOCK", "WAIST", "NECK1", "NECK2",
            "LEFT_J1", "LEFT_J2", "LEFT_J3", "LEFT_J4", "LEFT_J5", "LEFT_J6", "LEFT_J7",
            "RIGHT_J1", "RIGHT_J2", "RIGHT_J3", "RIGHT_J4", "RIGHT_J5", "RIGHT_J6", "RIGHT_J7"
        };

        // 左手关节
        std::vector<std::string> left_hand_joints = {
            "LEFT_L_THUMBMCP2",   // 拇指弯曲
            "LEFT_L_THUMBMCP1",   // 拇指摆动
            "LEFT_L_INDEXMCP",    // 食指弯曲
            "LEFT_L_MIDDLEMCP",   // 中指弯曲
            "LEFT_L_RINGMCP",     // 无名指弯曲
            "LEFT_L_LITTLEMCP"    // 小指弯曲
        };

        // 右手关节
        std::vector<std::string> right_hand_joints = {
            "RIGHT_R_THUMBMCP2",  // 拇指弯曲
            "RIGHT_R_THUMBMCP1",  // 拇指摆动
            "RIGHT_R_INDEXMCP",   // 食指弯曲
            "RIGHT_R_MIDDLEMCP",  // 中指弯曲
            "RIGHT_R_RINGMCP",    // 无名指弯曲
            "RIGHT_R_LITTLEMCP"   // 小指弯曲
        };

        // 合并所有关节名称
        all_joint_names_.insert(all_joint_names_.end(), general_joints.begin(), general_joints.end());
        all_joint_names_.insert(all_joint_names_.end(), left_hand_joints.begin(), left_hand_joints.end());
        all_joint_names_.insert(all_joint_names_.end(), right_hand_joints.begin(), right_hand_joints.end());

        // 创建关节索引映射，便于快速查找和更新
        create_joint_index_maps(general_joints, left_hand_joints, right_hand_joints);
    }

    void create_joint_index_maps(
        const std::vector<std::string>& general_joints,
        const std::vector<std::string>& left_hand_joints,
        const std::vector<std::string>& right_hand_joints)
    {
        // 通用关节索引
        for (size_t i = 0; i < general_joints.size(); ++i) {
            joint_indices_[general_joints[i]] = i;
        }

        // 左手关节索引（从通用关节之后开始）
        int left_start_idx = general_joints.size();
        for (size_t i = 0; i < left_hand_joints.size(); ++i) {
            joint_indices_[left_hand_joints[i]] = left_start_idx + i;
        }

        // 右手关节索引（从左手关节之后开始）
        int right_start_idx = general_joints.size() + left_hand_joints.size();
        for (size_t i = 0; i < right_hand_joints.size(); ++i) {
            joint_indices_[right_hand_joints[i]] = right_start_idx + i;
        }
    }

    void general_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        try {
            // 解析 JSON
            auto data = json::parse(msg->data);
            auto positions = data["joint_position"];
            
            // 确认数据长度
            if (positions.size() != 20) { // 通用关节数量应该是20个
                RCLCPP_WARN(this->get_logger(), 
                           "通用关节数量不匹配: %zu vs 20", 
                           positions.size());
                return;
            }
            
            // 更新通用关节位置
            for (size_t i = 0; i < positions.size(); ++i) {
                std::string joint_name = all_joint_names_[i];
                int index = joint_indices_[joint_name];
                if (index >= 0 && index < static_cast<int>(all_joint_names_.size())) {
                    general_joint_data_[joint_name] = static_cast<double>(positions[i]);
                }
            }
            
            general_data_updated_ = true;
            
        } catch (const json::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON 解析失败: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理通用关节消息时出错: %s", e.what());
        }
    }

    void left_hand_callback(const end_effector_interfaces::msg::EEFeedback::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 更新左手关节位置（百分比转弧度）；新接口 position 在每个 joint_state 内
        std::vector<double> positions;
        positions.reserve(msg->joint_states.size());
        for (const auto & js : msg->joint_states) {
            positions.push_back(js.position);
        }
        auto left_positions = percent_to_rad(positions, left_joint_ranges_);
        for (size_t i = 0; i < left_positions.size(); ++i) {
            std::string joint_name = left_hand_joint_names_[i];
            int index = joint_indices_[joint_name];
            if (index >= 0 && index < static_cast<int>(all_joint_names_.size())) {
                hand_joint_data_[joint_name] = left_positions[i];
            }
        }
        
        left_hand_data_updated_ = true;
    }

    void right_hand_callback(const end_effector_interfaces::msg::EEFeedback::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 更新右手关节位置（百分比转弧度）；新接口 position 在每个 joint_state 内
        std::vector<double> positions;
        positions.reserve(msg->joint_states.size());
        for (const auto & js : msg->joint_states) {
            positions.push_back(js.position);
        }
        auto right_positions = percent_to_rad(positions, right_joint_ranges_);
        for (size_t i = 0; i < right_positions.size(); ++i) {
            std::string joint_name = right_hand_joint_names_[i];
            int index = joint_indices_[joint_name];
            if (index >= 0 && index < static_cast<int>(all_joint_names_.size())) {
                hand_joint_data_[joint_name] = right_positions[i];
            }
        }
        
        right_hand_data_updated_ = true;
    }

    std::vector<double> percent_to_rad(const std::vector<double>& percent_list, 
                                      const std::vector<std::pair<double, double>>& joint_ranges)
    {
        std::vector<double> rad_list;
        for (size_t i = 0; i < percent_list.size(); ++i) {
            double min_angle = joint_ranges[i].first;
            double max_angle = joint_ranges[i].second;
            // 按百分比线性插值
            double rad = min_angle + (max_angle - min_angle) * (percent_list[i] / 100.0);
            rad_list.push_back(rad);
        }
        return rad_list;
    }

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 更新时间戳
        joint_state_msg_.header.stamp = this->now();
        
        // 更新所有关节位置
        bool has_data = false;
        
        // 更新通用关节位置
        for (const auto& [joint_name, position] : general_joint_data_) {
            auto it = joint_indices_.find(joint_name);
            if (it != joint_indices_.end()) {
                int index = it->second;
                if (index >= 0 && index < static_cast<int>(joint_state_msg_.position.size())) {
                    joint_state_msg_.position[index] = position;
                    has_data = true;
                }
            }
        }
        
        // 更新手部关节位置
        for (const auto& [joint_name, position] : hand_joint_data_) {
            auto it = joint_indices_.find(joint_name);
            if (it != joint_indices_.end()) {
                int index = it->second;
                if (index >= 0 && index < static_cast<int>(joint_state_msg_.position.size())) {
                    joint_state_msg_.position[index] = position;
                    has_data = true;
                }
            }
        }
        
        // 如果有数据更新，则发布
        if (has_data) {
            joint_state_publisher_->publish(joint_state_msg_);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "没有关节数据可发布");
        }
    }

    // 成员变量
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr general_subscription_;
    rclcpp::Subscription<end_effector_interfaces::msg::EEFeedback>::SharedPtr left_hand_subscription_;
    rclcpp::Subscription<end_effector_interfaces::msg::EEFeedback>::SharedPtr right_hand_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int publish_rate_hz_;
    
    // 关节名称列表
    std::vector<std::string> all_joint_names_;
    std::vector<std::string> general_joint_names_;
    std::vector<std::string> left_hand_joint_names_ = {
        "LEFT_L_THUMBMCP2", "LEFT_L_THUMBMCP1", "LEFT_L_INDEXMCP", 
        "LEFT_L_MIDDLEMCP", "LEFT_L_RINGMCP", "LEFT_L_LITTLEMCP"
    };
    std::vector<std::string> right_hand_joint_names_ = {
        "RIGHT_R_THUMBMCP2", "RIGHT_R_THUMBMCP1", "RIGHT_R_INDEXMCP", 
        "RIGHT_R_MIDDLEMCP", "RIGHT_R_RINGMCP", "RIGHT_R_LITTLEMCP"
    };
    
    // 关节索引映射
    std::map<std::string, int> joint_indices_;
    
    // 关节数据
    std::map<std::string, double> general_joint_data_;
    std::map<std::string, double> hand_joint_data_;
    
    // 关节范围（弧度）
    std::vector<std::pair<double, double>> left_joint_ranges_ = {
        std::make_pair(0.0, M_PI_2),           // THUMBMCP1: 0-90度
        std::make_pair(0.0, M_PI / 180.0 * 50.0), // THUMBMCP2: 0-50度
        std::make_pair(0.0, M_PI / 180.0 * 75.0), // INDEXMCP: 0-75度
        std::make_pair(0.0, M_PI / 180.0 * 75.0), // MIDDLEMCP: 0-75度
        std::make_pair(0.0, M_PI / 180.0 * 75.0), // RINGMCP: 0-75度
        std::make_pair(0.0, M_PI / 180.0 * 75.0)  // LITTLEMCP: 0-75度
    };
    
    std::vector<std::pair<double, double>> right_joint_ranges_ = {
        std::make_pair(0.0, M_PI_2),           // THUMBMCP1: 0-90度
        std::make_pair(0.0, M_PI / 180.0 * 50.0), // THUMBMCP2: 0-50度
        std::make_pair(0.0, M_PI / 180.0 * 75.0), // INDEXMCP: 0-75度
        std::make_pair(0.0, M_PI / 180.0 * 75.0), // MIDDLEMCP: 0-75度
        std::make_pair(0.0, M_PI / 180.0 * 75.0), // RINGMCP: 0-75度
        std::make_pair(0.0, M_PI / 180.0 * 75.0)  // LITTLEMCP: 0-75度
    };
    
    // 发布的关节状态消息
    sensor_msgs::msg::JointState joint_state_msg_;
    
    // 数据更新标志
    bool general_data_updated_ = false;
    bool left_hand_data_updated_ = false;
    bool right_hand_data_updated_ = false;
    
    // 线程安全
    std::mutex data_mutex_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<JointConverter>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}