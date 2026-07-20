#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#ifdef HAVE_SDL2
#include <SDL2/SDL.h>
#endif

using std::placeholders::_1;
using namespace std;

class mobile_joy_log_control : public rclcpp::Node {
public:
    mobile_joy_log_control() : Node("mobile_joy_log_control") {
        RCLCPP_INFO(this->get_logger(), "mobile_joy_log_control");

        // 声明参数（使用嵌套结构支持logitech和beitong两组参数）
        // 注意：此时不读取参数，等检测到设备后再读取
        // Logitech参数（默认值）
        this->declare_parameter<int>("logitech.button_enable_remote", 11);
        this->declare_parameter<int>("logitech.button_medium_speed", 4);
        this->declare_parameter<int>("logitech.axis_left_right", 0);
        this->declare_parameter<int>("logitech.axis_forward_backward", 3);
        this->declare_parameter<int>("logitech.axis_fixed_speed", 5);
        this->declare_parameter<int>("logitech.axis_fixed_angular", 4);
        this->declare_parameter<double>("logitech.joystick_deadzone", 0.1);
        this->declare_parameter<double>("logitech.max_linear_velocity", 0.4);
        this->declare_parameter<double>("logitech.max_angular_velocity", 0.75);
        this->declare_parameter<double>("logitech.medium_linear_velocity", 0.6);
        this->declare_parameter<double>("logitech.medium_angular_velocity", 1.0);
        this->declare_parameter<double>("logitech.acc_filter_param", 0.45);
        this->declare_parameter<double>("logitech.dec_filter_param", 0.05);
        this->declare_parameter<double>("logitech.inactive_timeout", 30.0);
        this->declare_parameter<int>("logitech.button_run_param_0", 0);
        this->declare_parameter<int>("logitech.button_run_param_1", 1);
        this->declare_parameter<std::string>("logitech.script_path", "");
        this->declare_parameter<std::string>("logitech.script_param_0", "");
        this->declare_parameter<std::string>("logitech.script_param_1", "");
        this->declare_parameter<int>("logitech.button_run_action_0", 0);
        this->declare_parameter<int>("logitech.button_run_action_1", 1);
        this->declare_parameter<std::string>("logitech.script_path_action", "");
        this->declare_parameter<std::string>("logitech.script_action_0", "");
        this->declare_parameter<std::string>("logitech.script_action_1", "");
        this->declare_parameter<bool>("logitech.omni_mode", false);
        this->declare_parameter<int>("logitech.axis_linear_y", 0);
        this->declare_parameter<int>("logitech.axis_angular_z", 2);

        // BEITONG参数（默认值，与Logitech相同，但可以在yaml中覆盖）
        this->declare_parameter<int>("beitong.button_enable_remote", 11);
        this->declare_parameter<int>("beitong.button_medium_speed", 4);
        this->declare_parameter<int>("beitong.axis_left_right", 0);
        this->declare_parameter<int>("beitong.axis_forward_backward", 3);
        this->declare_parameter<int>("beitong.axis_fixed_speed", 5);
        this->declare_parameter<int>("beitong.axis_fixed_angular", 4);
        this->declare_parameter<double>("beitong.joystick_deadzone", 0.1);
        this->declare_parameter<double>("beitong.max_linear_velocity", 0.4);
        this->declare_parameter<double>("beitong.max_angular_velocity", 0.75);
        this->declare_parameter<double>("beitong.medium_linear_velocity", 0.6);
        this->declare_parameter<double>("beitong.medium_angular_velocity", 1.0);
        this->declare_parameter<double>("beitong.acc_filter_param", 0.45);
        this->declare_parameter<double>("beitong.dec_filter_param", 0.05);
        this->declare_parameter<double>("beitong.inactive_timeout", 30.0);
        this->declare_parameter<int>("beitong.button_run_param_0", 0);
        this->declare_parameter<int>("beitong.button_run_param_1", 1);
        this->declare_parameter<std::string>("beitong.script_path", "");
        this->declare_parameter<std::string>("beitong.script_param_0", "");
        this->declare_parameter<std::string>("beitong.script_param_1", "");
        this->declare_parameter<int>("beitong.button_run_action_0", 0);
        this->declare_parameter<int>("beitong.button_run_action_1", 1);
        this->declare_parameter<std::string>("beitong.script_path_action", "");
        this->declare_parameter<std::string>("beitong.script_action_0", "");
        this->declare_parameter<std::string>("beitong.script_action_1", "");
        this->declare_parameter<bool>("beitong.omni_mode", false);
        this->declare_parameter<int>("beitong.axis_linear_y", 0);
        this->declare_parameter<int>("beitong.axis_angular_z", 2);

        // 接收手柄话题
        sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 1,
                                                               std::bind(&mobile_joy_log_control::callback, this, _1));
        mobile_diff_control_pub_joy_ = this->create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel_joy", 1);
        mobile_omni_control_pub_joy_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/control/cmd_vel_omni_joy", 1);
        pub_joy_in_ = this->create_publisher<std_msgs::msg::Int16>("control/joy_in", 1);
        // 定时发布机器人速度话题
        pub_timer_ = this->create_wall_timer(20ms, std::bind(&mobile_joy_log_control::pub_timer_callback, this));

        // 定时检测设备（每1秒检测一次）
        device_detect_timer_ =
            this->create_wall_timer(1000ms, std::bind(&mobile_joy_log_control::device_detect_callback, this));

        RCLCPP_INFO(this->get_logger(), "mobile_joy_log_control initialized, waiting for joystick device...");
    }

private:
    // 加载参数
    void loadParameters() {
        if (parameters_loaded_) {
            RCLCPP_WARN(this->get_logger(), "Parameters already loaded, skipping parameter loading");
            return;  // 参数已加载，不需要重复加载
        }

        // 先检测是否有设备
#ifdef HAVE_SDL2
        // 如果SDL已经初始化，先清理并重新初始化，以便重新扫描设备（支持热插拔）
        bool sdl_was_initialized = (SDL_WasInit(SDL_INIT_JOYSTICK) != 0);
        if (sdl_was_initialized) {
            SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
        }

        // 重新初始化SDL，这样会重新扫描设备
        if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) < 0) {
            RCLCPP_WARN(this->get_logger(), "SDL初始化失败，无法检测设备，使用默认Logitech参数");
            joystick_type_ = "logitech";  // 使用默认参数
        } else {
            int num_joysticks = SDL_NumJoysticks();
            if (num_joysticks == 0) {
                RCLCPP_WARN(this->get_logger(), "No joystick device detected, skipping parameter loading");
                return;  // 没有检测到设备，不加载参数
            }
            // 检测手柄名称
            joystick_type_ = detectJoystickName();
        }
#else
        // 没有SDL2，无法检测设备类型，使用默认Logitech参数
        RCLCPP_WARN(this->get_logger(), "SDL2未找到，使用默认Logitech参数");
        joystick_type_ = "logitech";
#endif
        std::string param_prefix = joystick_type_ + ".";  // "logitech." 或 "beitong."
        RCLCPP_INFO(this->get_logger(), "检测到设备，使用参数组: %s", joystick_type_.c_str());

        // 根据检测到的手柄类型获取参数
        std::string button_enable_remote_param = param_prefix + "button_enable_remote";
        std::string button_medium_speed_param = param_prefix + "button_medium_speed";
        std::string axis_left_right_param = param_prefix + "axis_left_right";
        std::string axis_forward_backward_param = param_prefix + "axis_forward_backward";
        std::string axis_fixed_speed_param = param_prefix + "axis_fixed_speed";
        std::string axis_fixed_angular_param = param_prefix + "axis_fixed_angular";
        std::string joystick_deadzone_param = param_prefix + "joystick_deadzone";
        std::string max_linear_velocity_param = param_prefix + "max_linear_velocity";
        std::string max_angular_velocity_param = param_prefix + "max_angular_velocity";
        std::string medium_linear_velocity_param = param_prefix + "medium_linear_velocity";
        std::string medium_angular_velocity_param = param_prefix + "medium_angular_velocity";
        std::string acc_filter_param_param = param_prefix + "acc_filter_param";
        std::string dec_filter_param_param = param_prefix + "dec_filter_param";
        std::string inactive_timeout_param = param_prefix + "inactive_timeout";
        std::string button_run_param_0_param = param_prefix + "button_run_param_0";
        std::string button_run_param_1_param = param_prefix + "button_run_param_1";
        std::string script_path_param = param_prefix + "script_path";
        std::string script_param_0_param = param_prefix + "script_param_0";
        std::string script_param_1_param = param_prefix + "script_param_1";
        std::string button_run_action_0_param = param_prefix + "button_run_action_0";
        std::string button_run_action_1_param = param_prefix + "button_run_action_1";
        std::string script_path_action_param = param_prefix + "script_path_action";
        std::string script_action_0_param = param_prefix + "script_action_0";
        std::string script_action_1_param = param_prefix + "script_action_1";

        // 全向模式参数
        std::string omni_mode_param = param_prefix + "omni_mode";
        std::string axis_linear_y_param = param_prefix + "axis_linear_y";
        std::string axis_angular_z_param = param_prefix + "axis_angular_z";

        // 获取参数
        this->get_parameter(button_enable_remote_param, button_enable_remote);
        this->get_parameter(button_medium_speed_param, button_medium_speed);
        this->get_parameter(axis_left_right_param, axis_left_right);
        this->get_parameter(axis_forward_backward_param, axis_forward_backward);
        this->get_parameter(axis_fixed_speed_param, axis_fixed_speed);
        this->get_parameter(axis_fixed_angular_param, axis_fixed_angular);
        this->get_parameter(joystick_deadzone_param, joystick_deadzone);
        this->get_parameter(max_linear_velocity_param, max_linear_velocity);
        this->get_parameter(max_angular_velocity_param, max_angular_velocity);
        this->get_parameter(medium_linear_velocity_param, medium_linear_velocity);
        this->get_parameter(medium_angular_velocity_param, medium_angular_velocity);
        this->get_parameter(acc_filter_param_param, acc_filter_param);
        this->get_parameter(dec_filter_param_param, dec_filter_param);
        this->get_parameter(inactive_timeout_param, inactive_timeout);

        // 获取脚本控制参数
        this->get_parameter(button_run_param_0_param, button_run_param_0);
        this->get_parameter(button_run_param_1_param, button_run_param_1);
        this->get_parameter(script_path_param, script_path);
        this->get_parameter(script_param_0_param, script_param_0);
        this->get_parameter(script_param_1_param, script_param_1);

        this->get_parameter(button_run_action_0_param, button_run_action_0);
        this->get_parameter(button_run_action_1_param, button_run_action_1);
        this->get_parameter(script_path_action_param, script_path_action);
        this->get_parameter(script_action_0_param, script_action_0);
        this->get_parameter(script_action_1_param, script_action_1);

        // 获取全向模式参数
        this->get_parameter(omni_mode_param, is_omni_mode_);
        if (is_omni_mode_) {
            this->get_parameter(axis_linear_y_param, axis_linear_y);
            this->get_parameter(axis_angular_z_param, axis_angular_z);
        }

        parameters_loaded_ = true;

        // 打印初始化信息
        RCLCPP_INFO(this->get_logger(), "参数加载完成");
        RCLCPP_INFO(this->get_logger(), "Button mappings:");
        RCLCPP_INFO(this->get_logger(), "  Enable Remote (Hold): Button %d", button_enable_remote);
        RCLCPP_INFO(this->get_logger(), "  Medium Speed (Hold): Button %d", button_medium_speed);
        RCLCPP_INFO(this->get_logger(), "Axis mappings:");
        if (is_omni_mode_) {
            RCLCPP_INFO(this->get_logger(), "  --- OMNI MODE ENABLED ---");
            RCLCPP_INFO(this->get_logger(), "  Linear X (Forward/Backward): Axis %d", axis_forward_backward);
            RCLCPP_INFO(this->get_logger(), "  Linear Y (Left/Right): Axis %d", axis_linear_y);
            RCLCPP_INFO(this->get_logger(), "  Angular Z (Rotation): Axis %d", axis_angular_z);
            RCLCPP_INFO(this->get_logger(), "  Fixed Speed X: Axis %d (+/- 0.5 m/s)", axis_fixed_speed);
            RCLCPP_INFO(this->get_logger(), "  Fixed Speed Y: Axis %d (+/- 0.5 m/s)", axis_fixed_angular);
        } else {
            RCLCPP_INFO(this->get_logger(), "  --- DIFFERENTIAL MODE ENABLED ---");
            RCLCPP_INFO(this->get_logger(), "  Left-Right: Axis %d", axis_left_right);
            RCLCPP_INFO(this->get_logger(), "  Forward-Backward: Axis %d", axis_forward_backward);
            RCLCPP_INFO(this->get_logger(), "  Fixed Speed: Axis %d (0.5 or -0.5 m/s)", axis_fixed_speed);
            RCLCPP_INFO(this->get_logger(), "  Fixed Angular: Axis %d (0.5 or -0.5 rad/s)", axis_fixed_angular);
        }
        RCLCPP_INFO(this->get_logger(),
                    "Initialization: Push both left and right joysticks toward center (Axis 0 & 2)");
        RCLCPP_INFO(this->get_logger(), "Joystick deadzone: %.2f", joystick_deadzone);
        RCLCPP_INFO(this->get_logger(), "Default velocity: Linear=%.2f, Angular=%.2f", max_linear_velocity,
                    max_angular_velocity);
        RCLCPP_INFO(this->get_logger(), "Medium velocity: Linear=%.2f, Angular=%.2f", medium_linear_velocity,
                    medium_angular_velocity);
        RCLCPP_INFO(this->get_logger(), "acc_filter_param is: %0.2f", acc_filter_param);
        RCLCPP_INFO(this->get_logger(), "dec_filter_param is: %0.2f", dec_filter_param);
        RCLCPP_INFO(this->get_logger(), "inactive_timeout is: %0.1f seconds", inactive_timeout);
    }

    // 设备检测回调函数
    void device_detect_callback() {
        if (parameters_loaded_) {
            return;  // 参数已加载，不需要再检测
        }

#ifdef HAVE_SDL2
        // 如果SDL已经初始化，先清理并重新初始化，以便重新扫描设备（支持热插拔）
        bool sdl_was_initialized = (SDL_WasInit(SDL_INIT_JOYSTICK) != 0);
        if (sdl_was_initialized) {
            SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
        }

        // 重新初始化SDL，这样会重新扫描设备
        if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) < 0) {
            return;  // SDL初始化失败，无法检测设备
        }

        int num_joysticks = SDL_NumJoysticks();
        if (num_joysticks > 0) {
            // 检测到设备，加载参数
            loadParameters();
        }
#endif
    }

    // 检测手柄名称
    std::string detectJoystickName() {
        std::string joystick_name = "logitech";  // 默认使用Logitech参数

#ifdef HAVE_SDL2
        bool sdl_was_initialized = (SDL_WasInit(SDL_INIT_JOYSTICK) != 0);

        // 初始化SDL（如果尚未初始化）
        if (!sdl_was_initialized) {
            if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
                RCLCPP_WARN(this->get_logger(), "SDL初始化失败: %s，使用默认Logitech参数", SDL_GetError());
                return joystick_name;
            }
        }

        // 获取手柄数量
        int num_joysticks = SDL_NumJoysticks();
        RCLCPP_INFO(this->get_logger(), "检测到 %d 个手柄设备", num_joysticks);

        // 尝试打开第一个手柄
        if (num_joysticks > 0) {
            SDL_Joystick* joystick = SDL_JoystickOpen(0);
            if (joystick != nullptr) {
                const char* name = SDL_JoystickName(joystick);
                if (name != nullptr) {
                    std::string joystick_name_str(name);
                    RCLCPP_INFO(this->get_logger(), "Opened joystick: %s", joystick_name_str.c_str());

                    // 检查是否是BEITONG手柄（不区分大小写）
                    std::string joystick_name_upper = joystick_name_str;
                    std::transform(joystick_name_upper.begin(), joystick_name_upper.end(), joystick_name_upper.begin(),
                                   ::toupper);
                    if (joystick_name_upper.find("BEITONG") != std::string::npos) {
                        joystick_name = "beitong";
                        RCLCPP_INFO(this->get_logger(), "检测到BEITONG手柄，使用BEITONG参数");
                    } else {
                        RCLCPP_INFO(this->get_logger(), "检测到Logitech手柄，使用Logitech参数");
                    }
                }
                SDL_JoystickClose(joystick);
            } else {
                RCLCPP_WARN(this->get_logger(), "无法打开手柄设备，使用默认Logitech参数");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "未检测到手柄设备，使用默认Logitech参数");
        }

        // 如果SDL是我们初始化的，则清理（但通常不清理，因为可能其他程序也在使用）
        // if (!sdl_was_initialized) {
        //     SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
        // }
#else
        RCLCPP_WARN(this->get_logger(), "SDL2未找到，使用默认Logitech参数");
#endif

        return joystick_name;
    }

    // 应用死区函数
    double applyDeadzone(double value) {
        if (std::abs(value) < joystick_deadzone) {
            return 0.0;
        }
        return value;
    }

    double filterAxisCommand(double target, double previous, double alpha_acc, double alpha_dec) const {
        constexpr double reverse_stop_threshold = 0.005;

        // 正负方向切换不能直接从 previous 插值到反向 target，否则一次滤波就可能
        // 跨过零点，同时给底盘“刹车+反向驱动”。先只向零衰减，确认到零后，
        // 下一周期再按加速时间常数进入新方向。
        if (target * previous < 0.0) {
            const double braking_value = (1.0 - alpha_dec) * previous;
            return std::abs(braking_value) <= reverse_stop_threshold ? 0.0 : braking_value;
        }

        const bool increasing_magnitude = std::abs(target) >= std::abs(previous);
        const double alpha = increasing_magnitude ? alpha_acc : alpha_dec;
        return (1.0 - alpha) * previous + alpha * target;
    }

    void publishZeroCmd() {
        mobile_diff_control_command.linear.x = 0.0;
        mobile_diff_control_command.linear.y = 0.0;
        mobile_diff_control_command.angular.z = 0.0;

        // 根据当前模式发布到不同话题
        if (is_omni_mode_) {
            if (mobile_omni_control_pub_joy_) {
                mobile_omni_control_pub_joy_->publish(mobile_diff_control_command);
            }
        } else {
            if (mobile_diff_control_pub_joy_) {
                mobile_diff_control_pub_joy_->publish(mobile_diff_control_command);
            }
        }

        last_linear_x_ = 0.0;
        last_linear_y_ = 0.0;
        last_angular_z_ = 0.0;
    }

    // 执行脚本函数
    void runScript(const std::string& script_path, const std::string& param_path) {
        // 原子占位，避免同一帧多键触发的竞态
        bool expected = false;
        if (!script_running_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(this->get_logger(), "脚本正在运行中，请等待脚本执行完毕后再试！");
            return;
        }

        std::thread([this, script_path, param_path]() {
            {
                std::lock_guard<std::mutex> lg(script_mutex_);
                RCLCPP_INFO(this->get_logger(), "========== 脚本开始执行 ==========");
                RCLCPP_INFO(this->get_logger(), "脚本路径: %s", script_path.c_str());
                RCLCPP_INFO(this->get_logger(), "参数文件: %s", param_path.c_str());
                RCLCPP_INFO(this->get_logger(), "=====================================");
            }

            // 执行Python脚本，传递参数
            std::string cmd = "python3 " + script_path + " " + param_path;
            int result = std::system(cmd.c_str());

            {
                std::lock_guard<std::mutex> lg(script_mutex_);
                RCLCPP_INFO(this->get_logger(), "========== 脚本执行完成 ==========");
                RCLCPP_INFO(this->get_logger(), "返回值: %d", result);
                RCLCPP_INFO(this->get_logger(), "现在可以启动新的脚本了");
                RCLCPP_INFO(this->get_logger(), "=====================================");
            }

            // 脚本执行完成后更新活跃时间，避免长时间执行导致被判定为不活跃
            // 注意：由于我们在不活跃检查中已经增加了对script_running_的检查，
            // 脚本运行期间不会被判定为不活跃，所以这里只需要更新时间即可
            last_activity_time_ = this->now();
            script_running_.store(false);
        }).detach();
    }

    // 顺序执行多个脚本（先执行standby，再执行动作）
    void runScriptWithStandby(const std::string& action_script_path, const std::string& action_param_path) {
        // 原子占位，避免同一帧多键触发的竞态
        bool expected = false;
        if (!script_running_.compare_exchange_strong(expected, true)) {
            RCLCPP_WARN(this->get_logger(), "脚本正在运行中，请等待脚本执行完毕后再试！");
            return;
        }

        std::thread([this, action_script_path, action_param_path]() {
            // 第一步：先执行standby脚本
            {
                std::lock_guard<std::mutex> lg(script_mutex_);
                RCLCPP_INFO(this->get_logger(), "========== [1/2] 开始执行Standby脚本 ==========");
                RCLCPP_INFO(this->get_logger(), "脚本路径: %s", script_path.c_str());
                RCLCPP_INFO(this->get_logger(), "参数文件: %s", script_param_0.c_str());
                RCLCPP_INFO(this->get_logger(), "================================================");
            }

            std::string standby_cmd = "python3 " + script_path + " " + script_param_0;
            int standby_result = std::system(standby_cmd.c_str());

            {
                std::lock_guard<std::mutex> lg(script_mutex_);
                RCLCPP_INFO(this->get_logger(), "========== [1/2] Standby脚本执行完成 ==========");
                RCLCPP_INFO(this->get_logger(), "返回值: %d", standby_result);
                RCLCPP_INFO(this->get_logger(), "================================================");
            }

            // 检查standby脚本是否执行成功
            if (standby_result != 0) {
                std::lock_guard<std::mutex> lg(script_mutex_);
                RCLCPP_ERROR(this->get_logger(), "Standby脚本执行失败，取消后续动作执行");
                last_activity_time_ = this->now();
                script_running_.store(false);
                return;
            }

            // 第二步：执行动作脚本
            {
                std::lock_guard<std::mutex> lg(script_mutex_);
                RCLCPP_INFO(this->get_logger(), "========== [2/2] 开始执行动作脚本 ==========");
                RCLCPP_INFO(this->get_logger(), "脚本路径: %s", action_script_path.c_str());
                RCLCPP_INFO(this->get_logger(), "参数文件: %s", action_param_path.c_str());
                RCLCPP_INFO(this->get_logger(), "=============================================");
            }

            std::string action_cmd = "python3 " + action_script_path + " " + action_param_path;
            int action_result = std::system(action_cmd.c_str());

            {
                std::lock_guard<std::mutex> lg(script_mutex_);
                RCLCPP_INFO(this->get_logger(), "========== [2/2] 动作脚本执行完成 ==========");
                RCLCPP_INFO(this->get_logger(), "返回值: %d", action_result);
                RCLCPP_INFO(this->get_logger(), "========== 所有脚本执行完成 ==========");
                RCLCPP_INFO(this->get_logger(), "现在可以启动新的脚本了");
                RCLCPP_INFO(this->get_logger(), "=========================================");
            }

            last_activity_time_ = this->now();
            script_running_.store(false);
        }).detach();
    }

    void pub_timer_callback() {
        rclcpp::Time now = this->now();
        if ((now - last_joy_time_).seconds() > 0.5) {
            if (!prev_no_data_) {
                zero_pub_count_no_data_ = 5;
                prev_no_data_ = true;
            }
            // 间隔1s打印一次没有收到的消息
            if ((now - last_pub_msg_time_).seconds() > 1.0) {
                last_pub_msg_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "No joy data, all commands are zero");
            }
            // 热插拔防止串口有默认数据
            joy_init_flag = false;
            remote_enabled = false;
            joy_state = -2;
            init_print_count_ = 0;
        } else {
            if (prev_no_data_) {
                zero_pub_count_no_data_ = 5;  // 恢复正常也发 5 次 0
                RCLCPP_WARN(this->get_logger(), "Joy data recovered, publish 0 cmd 5 times");
            }
            prev_no_data_ = false;

            // 检查长时间不活跃，需要重新初始化
            // 如果脚本正在运行或使能状态下，认为手柄是活跃状态，不应该判定为不活跃
            if (joy_init_flag && (now - last_activity_time_).seconds() > inactive_timeout && !script_running_.load() &&
                !remote_enabled) {
                RCLCPP_WARN(this->get_logger(), "手柄长时间不活跃，需要重新初始化");
                joy_init_flag = false;
                remote_enabled = false;
                joy_state = -1;
                init_print_count_ = 0;
            }
        }

        if (!remote_enabled) {
            if (!prev_not_enabled_) {
                zero_pub_count_disabled_ = 5;
                prev_not_enabled_ = true;
            }
            if (joy_init_flag) joy_state = 0;
        } else {
            if (prev_not_enabled_) {
                zero_pub_count_disabled_ = 5;  // 恢复正常也发 5 次 0
                RCLCPP_WARN(this->get_logger(), "Remote enabled, publish 0 cmd 5 times");
            }
            prev_not_enabled_ = false;
        }
        joy_in.data = joy_state;
        pub_joy_in_->publish(joy_in);

        if (zero_pub_count_disabled_ > 0) {
            RCLCPP_INFO(this->get_logger(), "No enable, Pub zero velocity");
            publishZeroCmd();
            zero_pub_count_disabled_--;
            return;
        }
        if (zero_pub_count_no_data_ > 0) {
            publishZeroCmd();
            RCLCPP_INFO(this->get_logger(), "No Data, Pub zero velocity");
            zero_pub_count_no_data_--;
            return;
        }
        // 脚本执行期间不允许运动，即使使能按键被按下
        if (remote_enabled && isInitReady() && !script_running_.load()) {
            // 速度指令发布前限速（使用当前模式的速度限制）
            // 速度指令发布前限速
            mobile_diff_control_command.linear.x =
                std::clamp(mobile_diff_control_command.linear.x, -current_max_linear, current_max_linear);
            if (is_omni_mode_) {
                mobile_diff_control_command.linear.y =
                    std::clamp(mobile_diff_control_command.linear.y, -current_max_linear, current_max_linear);
            }
            mobile_diff_control_command.angular.z =
                std::clamp(mobile_diff_control_command.angular.z, -current_max_angular, current_max_angular);

            // 打印最终输出的控制命令
            if (is_omni_mode_) {
                RCLCPP_INFO(this->get_logger(), "Output command: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
                            mobile_diff_control_command.linear.x, mobile_diff_control_command.linear.y,
                            mobile_diff_control_command.angular.z);
            } else {
                RCLCPP_INFO(this->get_logger(), "Output command: linear.x=%.2f, angular.z=%.2f",
                            mobile_diff_control_command.linear.x, mobile_diff_control_command.angular.z);
            }

            // 根据当前模式选话题发布
            if (is_omni_mode_) {
                if (mobile_omni_control_pub_joy_) {
                    mobile_omni_control_pub_joy_->publish(mobile_diff_control_command);
                }
            } else {
                if (mobile_diff_control_pub_joy_) {
                    mobile_diff_control_pub_joy_->publish(mobile_diff_control_command);
                }
            }

            last_linear_x_ = mobile_diff_control_command.linear.x;
            last_linear_y_ = mobile_diff_control_command.linear.y;
            last_angular_z_ = mobile_diff_control_command.angular.z;
        }
    }

    void callback(const sensor_msgs::msg::Joy::SharedPtr Joy) {
        // 如果参数未加载，尝试加载参数（收到手柄数据说明设备已连接）
        if (!parameters_loaded_) {
            RCLCPP_WARN(this->get_logger(), "No parameters loaded, trying to load parameters");
            loadParameters();
            // 如果加载失败（没有检测到设备），直接返回
            if (!parameters_loaded_) {
                RCLCPP_WARN(this->get_logger(), "Failed to load parameters, no parameters loaded");
                return;
            }
        }

        last_joy_time_ = this->now();
        // 速度指令置0；
        mobile_diff_control_command.linear.x = 0.0;
        mobile_diff_control_command.angular.z = 0.0;

        // 打印所有按键和摇杆的原始值
        std::stringstream ss;
        ss << "Raw Joy Data - Buttons: [";
        for (size_t i = 0; i < Joy->buttons.size(); ++i) {
            ss << Joy->buttons[i] << " ";
        }
        ss << "] Axes: [";
        for (size_t i = 0; i < Joy->axes.size(); ++i) {
            ss << Joy->axes[i] << " ";
        }
        ss << "]";
        RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());

        // 检查遥控器启用/禁用按键的状态（长按使能）
        bool current_enable_button_state = (Joy->buttons[button_enable_remote] == 1);
        // 如果脚本正在运行，即使按下使能按键也不允许运动
        if (script_running_.load() && current_enable_button_state) {
            remote_enabled = false;
            RCLCPP_WARN(this->get_logger(), "Script is running, remote control disabled for safety");
        } else {
            remote_enabled = current_enable_button_state;  // 直接根据按键状态设置使能状态
            // 使能状态下认为活跃，更新活跃时间
            if (remote_enabled) {
                last_activity_time_ = this->now();
            }
            // 使能按键状态变化时也认为是活跃（按键触发）
            if (current_enable_button_state != last_enable_button_state_) {
                last_activity_time_ = this->now();
            }
            last_enable_button_state_ = current_enable_button_state;
        }

        // 检查中速模式按键状态
        bool current_medium_speed_state = (Joy->buttons[button_medium_speed] == 1);
        // 中速模式按键状态变化时认为是活跃（按键触发）
        if (current_medium_speed_state) {
            last_activity_time_ = this->now();
        }

        // 检查脚本运行按键（需要在未使能状态下才能执行，使能状态下机器人运动时不能执行脚本）
        bool current_script_0_button =
            (Joy->buttons.size() > button_run_param_0 && Joy->buttons[button_run_param_0] == 1);
        bool current_script_1_button =
            (Joy->buttons.size() > button_run_param_1 && Joy->buttons[button_run_param_1] == 1);

        // 按键0边缘触发检测（需要已初始化，且未使能状态下才能执行）
        if (current_script_0_button && !last_script_param_0_button_state_) {
            if (!isInitReady()) {
                RCLCPP_WARN(this->get_logger(), "Button 0 pressed - Not initialized, please initialize first.");
            } else if (remote_enabled) {
                RCLCPP_WARN(this->get_logger(),
                            "Button 0 pressed - Remote is enabled (robot is moving), cannot run script! Please disable "
                            "remote first.");
            } else if (script_running_.load()) {
                RCLCPP_WARN(this->get_logger(), "Button 0 pressed - Script is already running, please wait!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Button 0 pressed - Starting script with param 0");
                last_activity_time_ = this->now();  // 按键触发脚本也认为是活跃状态
                runScript(script_path, script_param_0);
            }
        }
        last_script_param_0_button_state_ = current_script_0_button;

        // 按键1边缘触发检测（需要已初始化，且未使能状态下才能执行）
        if (current_script_1_button && !last_script_param_1_button_state_) {
            if (!isInitReady()) {
                RCLCPP_WARN(this->get_logger(), "Button 1 pressed - Not initialized, please initialize first.");
            } else if (remote_enabled) {
                RCLCPP_WARN(this->get_logger(),
                            "Button 1 pressed - Remote is enabled (robot is moving), cannot run script! Please disable "
                            "remote first.");
            } else if (script_running_.load()) {
                RCLCPP_WARN(this->get_logger(), "Button 1 pressed - Script is already running, please wait!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Button 1 pressed - Starting script with param 1");
                last_activity_time_ = this->now();  // 按键触发脚本也认为是活跃状态
                runScript(script_path, script_param_1);
            }
        }
        last_script_param_1_button_state_ = current_script_1_button;

        // 检查动作脚本运行按键（需要在未使能状态下才能执行，使能状态下机器人运动时不能执行脚本）
        bool current_script_action_0_button =
            (Joy->buttons.size() > button_run_action_0 && Joy->buttons[button_run_action_0] == 1);
        bool current_script_action_1_button =
            (Joy->buttons.size() > button_run_action_1 && Joy->buttons[button_run_action_1] == 1);

        // 动作按键0边缘触发检测（需要已初始化，且未使能状态下才能执行）
        if (current_script_action_0_button && !last_script_action_0_button_state_) {
            if (!isInitReady()) {
                RCLCPP_WARN(this->get_logger(), "Action Button 0 pressed - Not initialized, please initialize first.");
            } else if (remote_enabled) {
                RCLCPP_WARN(this->get_logger(),
                            "Action Button 0 pressed - Remote is enabled (robot is moving), cannot run script! Please "
                            "disable remote first.");
            } else if (script_running_.load()) {
                RCLCPP_WARN(this->get_logger(), "Action Button 0 pressed - Script is already running, please wait!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Action Button 0 pressed - Starting action script with action 0");
                last_activity_time_ = this->now();  // 按键触发脚本也认为是活跃状态
                runScriptWithStandby(script_path_action, script_action_0);
            }
        }
        last_script_action_0_button_state_ = current_script_action_0_button;

        // 动作按键1边缘触发检测（需要已初始化，且未使能状态下才能执行）
        if (current_script_action_1_button && !last_script_action_1_button_state_) {
            if (!isInitReady()) {
                RCLCPP_WARN(this->get_logger(), "Action Button 1 pressed - Not initialized, please initialize first.");
            } else if (remote_enabled) {
                RCLCPP_WARN(this->get_logger(),
                            "Action Button 1 pressed - Remote is enabled (robot is moving), cannot run script! Please "
                            "disable remote first.");
            } else if (script_running_.load()) {
                RCLCPP_WARN(this->get_logger(), "Action Button 1 pressed - Script is already running, please wait!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Action Button 1 pressed - Starting action script with action 1");
                last_activity_time_ = this->now();  // 按键触发脚本也认为是活跃状态
                runScriptWithStandby(script_path_action, script_action_1);
            }
        }
        last_script_action_1_button_state_ = current_script_action_1_button;

        if (remote_enabled) {
            RCLCPP_DEBUG(this->get_logger(), "Remote control enabled (button held)");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Remote control disabled (button released)");
        }

        if (current_medium_speed_state) {
            RCLCPP_DEBUG(this->get_logger(), "Medium speed mode enabled");
        }

        // 获取摇杆值并应用死区
        // 获取原始值
        double raw_lr = Joy->axes[axis_left_right];
        double raw_fb = Joy->axes[axis_forward_backward];

        auto isValid = [&](double raw, double last_valid) {
            // 1. 合法范围 [-1, 1]
            if (!std::isfinite(raw) || raw < -1.01 || raw > 1.01) {
                return false;
            }

            // 2. 如果回到 0 附近（松手场景），直接认为合法
            if (std::fabs(raw) < 0.05) {  // 0 附近的容差
                return true;
            }
            return true;
        };

        rclcpp::Time now = this->now();
        if (!joy_init_flag) {
            // 阶段1：检测初始化拨动手势（轴0和轴2 相反方向拨动）
            double axis_0 = Joy->axes[0];  // 左摇杆左右
            double axis_1 = Joy->axes[1];  // 左摇杆前后
            double axis_2 = Joy->axes[2];  // 右摇杆左右
            double axis_3 = Joy->axes[3];  // 右摇杆前后
            // RCLCPP_INFO(this->get_logger(), "axis_0: %f, axis_1: %f, axis_2: %f, axis_3: %  f", axis_0, axis_1,
            // axis_2, axis_3);

            // if(!init_pending_ && axis_0 < -0.85 && axis_2 > 0.85)
            if (!init_pending_ && axis_0 < -0.25 && axis_1 > 0.25 && axis_2 > 0.25 && axis_3 > 0.25) {
                init_pending_ = true;
                RCLCPP_INFO(this->get_logger(), "检测到初始化拨动，请松开并将所有轴回零");
            }

            // 阶段2：等待回零后再判定成功
            if (init_pending_) {
                bool all_buttons_zero = true;
                for (size_t i = 0; i < Joy->buttons.size(); ++i) {
                    if (Joy->buttons[i] != 0) {
                        all_buttons_zero = false;
                        break;
                    }
                }
                bool all_axes_zero = true;
                const double zero_tol = 0.05;  // 轴回零容差
                // BEITONG手柄的轴4和轴5会一直是1，所以只检测前4个轴
                size_t max_axis_to_check = (joystick_type_ == "beitong") ? 4 : Joy->axes.size();
                for (size_t i = 0; i < max_axis_to_check; ++i) {
                    if (std::fabs(Joy->axes[i]) > zero_tol) {
                        all_axes_zero = false;
                        break;
                    }
                }

                if (all_buttons_zero && all_axes_zero) {
                    joy_init_flag = true;
                    init_pending_ = false;
                    init_complete_time_ = this->now();  // 记录初始化完成时间（用于保护期）
                    last_activity_time_ = this->now();  // 记录初始化时间
                    init_print_count_ = 0;
                    last_enable_button_state_ = Joy->buttons[button_enable_remote];
                    RCLCPP_INFO(this->get_logger(), "初始化成功，等待保护期 0.3s...");
                }
            }

            if ((now - last_joy_init_time_).seconds() > 0.5) {
                last_joy_init_time_ = this->now();
                if (init_print_count_ < 5) {
                    if (!init_pending_) {
                        RCLCPP_INFO(this->get_logger(), "请左右摇杆斜上推，初始化手柄");
                    } else {
                        RCLCPP_INFO(this->get_logger(), "请松开摇杆并回零，完成初始化");
                    }
                    init_print_count_++;
                }
            }
            joy_state = -1;
            return;
        }
        // 左右轴
        if (isValid(raw_lr, last_valid_lr_)) {
            left_right_value = applyDeadzone(raw_lr);
            last_valid_lr_ = left_right_value;
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid LR value: %f, use last=%f", raw_lr, last_valid_lr_);
            left_right_value = last_valid_lr_;
        }

        // 前后轴
        if (isValid(raw_fb, last_valid_fb_)) {
            forward_backward_value = applyDeadzone(raw_fb);
            last_valid_fb_ = forward_backward_value;
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid FB value: %f, use last=%f", raw_fb, last_valid_fb_);
            forward_backward_value = last_valid_fb_;
        }

        // 如果遥控器未启用，不处理任何输入,手柄接受为False
        if (!remote_enabled) {
            RCLCPP_DEBUG(this->get_logger(), "Remote control is disabled, ignoring input");
            joy_state = 0;
            return;
        }

        joy_state = 1;  // 使能状态下joy_in为1

        // 处理控制逻辑
        double target_linear_x = 0.0;
        double target_linear_y = 0.0;
        double target_angular_z = 0.0;

        // 根据中速模式选择速度限制
        current_max_linear = current_medium_speed_state ? medium_linear_velocity : max_linear_velocity;
        current_max_angular = current_medium_speed_state ? medium_angular_velocity : max_angular_velocity;

        if (is_omni_mode_) {
            // --- 全向模式控制逻辑 ---
            double linear_y_value = applyDeadzone(Joy->axes[axis_linear_y]);
            double angular_z_value = applyDeadzone(Joy->axes[axis_angular_z]);

            // 固定速度模式 (X和Y轴)
            double fixed_speed_x_val = Joy->axes[axis_fixed_speed];
            double fixed_speed_y_val = Joy->axes[axis_fixed_angular];
            bool using_fixed_speed_x = (std::abs(fixed_speed_x_val) > 0.5);
            bool using_fixed_speed_y = (std::abs(fixed_speed_y_val) > 0.5);

            if (using_fixed_speed_x) {
                target_linear_x = (fixed_speed_x_val > 0) ? 0.5 : -0.5;
                RCLCPP_INFO(this->get_logger(), "Fixed speed X: %.2f m/s", target_linear_x);
            } else {
                target_linear_x = forward_backward_value * current_max_linear;
            }

            if (using_fixed_speed_y) {
                target_linear_y = (fixed_speed_y_val > 0) ? 0.5 : -0.5;
                RCLCPP_INFO(this->get_logger(), "Fixed speed Y: %.2f m/s", target_linear_y);
            } else {
                target_linear_y = linear_y_value * current_max_linear;
            }

            // 正常角速度控制
            target_angular_z = angular_z_value * current_max_angular;

            if (using_fixed_speed_x || using_fixed_speed_y) {
                last_activity_time_ = this->now();
            }

        } else {
            // --- 差分模式控制逻辑 (保持原有逻辑) ---
            target_linear_x = forward_backward_value * current_max_linear;
            target_angular_z = left_right_value * current_max_angular;

            // 固定速度控制
            double fixed_speed_value = Joy->axes[axis_fixed_speed];
            if (std::abs(fixed_speed_value) > 0.5) {
                target_linear_x = (fixed_speed_value > 0) ? 0.5 : -0.5;
                RCLCPP_INFO(this->get_logger(), "Fixed speed mode: %.2f m/s", target_linear_x);
                last_activity_time_ = this->now();
            }

            // 固定角速度控制
            double fixed_angular_value = Joy->axes[axis_fixed_angular];
            if (std::abs(fixed_angular_value) > 0.5) {
                target_angular_z = (fixed_angular_value > 0) ? 0.75 : -0.75;
                RCLCPP_INFO(this->get_logger(), "Fixed angular mode: %.2f rad/s", target_angular_z);
                last_activity_time_ = this->now();
            }
        }
        // 时间常数化滤波；最终下发前 mobile_cmd_omni 还会统一执行加速度和
        // jerk 限制。这里主要处理摇杆噪声，并保证快速反向一定先经过零速。
        double dt = 0.02;

        double alpha_acc = 1.0 - std::exp(-dt / acc_filter_param);
        double alpha_dec = 1.0 - std::exp(-dt / dec_filter_param);

        // --- X轴线速度滤波 ---
        mobile_diff_control_command.linear.x = filterAxisCommand(
            target_linear_x, last_linear_x_, alpha_acc, alpha_dec);

        // --- Y轴线速度滤波 (仅在全向模式下) ---
        if (is_omni_mode_) {
            mobile_diff_control_command.linear.y = filterAxisCommand(
                target_linear_y, last_linear_y_, alpha_acc, alpha_dec);
        } else {
            mobile_diff_control_command.linear.y = 0.0;
        }

        // --- Z轴角速度滤波 ---
        mobile_diff_control_command.angular.z = filterAxisCommand(
            target_angular_z, last_angular_z_, alpha_acc, alpha_dec);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mobile_diff_control_pub_joy_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mobile_omni_control_pub_joy_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_joy_in_;

    // 是否进入手柄话题
    std_msgs::msg::Int16 joy_in;
    int joy_state = -1;

    // 手柄类型
    std::string joystick_type_ = "logitech";  // 手柄类型："logitech" 或 "beitong"

    // 运动模式相关变量
    bool remote_enabled = false;  // 遥控器是否启用，默认为禁用状态

    // 摇杆值
    double left_right_value = 0.0;        // 左推右推值
    double forward_backward_value = 0.0;  // 前推后推值

    // 配置参数
    int button_enable_remote;   // 启用/禁用遥控器的按键
    int button_medium_speed;    // 中速模式按键
    int axis_left_right;        // 左推右推摇杆
    int axis_forward_backward;  // 前推后推摇杆
    int axis_fixed_speed;       // 固定速度摇杆
    int axis_fixed_angular;     // 固定角速度摇杆

    // 全向模式新增参数
    bool is_omni_mode_ = false;  // 是否为全向模式
    int axis_linear_y;           // Y轴平移摇杆 (全向模式)
    int axis_angular_z;          // Z轴旋转摇杆 (全向模式)

    // 脚本控制参数
    int button_run_param_0;      // 运行脚本的按键0
    int button_run_param_1;      // 运行脚本的按键1
    std::string script_path;     // 共用脚本路径
    std::string script_param_0;  // 按键0的参数
    std::string script_param_1;  // 按键1的参数

    // 动作脚本控制参数
    int button_run_action_0;         // 运行动作脚本的按键0
    int button_run_action_1;         // 运行动作脚本的按键1
    std::string script_path_action;  // 共用动作脚本路径
    std::string script_action_0;     // 按键0的动作
    std::string script_action_1;     // 按键1的动作

    // 脚本运行状态
    std::atomic<bool> script_running_ = {false};
    std::mutex script_mutex_;

    // 按键状态
    bool last_script_param_0_button_state_ = false;
    bool last_script_param_1_button_state_ = false;
    bool last_script_action_0_button_state_ = false;
    bool last_script_action_1_button_state_ = false;

    // 运动参数
    geometry_msgs::msg::Twist mobile_diff_control_command;
    double max_linear_velocity = 0.4;      // 默认线速度
    double max_angular_velocity = 0.75;    // 默认角速度
    double medium_linear_velocity = 0.6;   // 中速线速度
    double medium_angular_velocity = 1.0;  // 中速角速度
    double current_max_linear = 0.4;       // 当前最大线速度
    double current_max_angular = 0.75;     // 当前最大角速度
    double max_linear_acc_ = 1.0;
    double max_angular_acc_ = 1.0;
    // 用于一阶低通滤波
    double acc_filter_param = 0.45;
    double dec_filter_param = 0.05;
    double last_linear_x_ = 0.0;
    double last_linear_y_ = 0.0;  // Y轴速度历史值 (全向模式)
    double last_angular_z_ = 0.0;

    // 摇杆死区参数
    double joystick_deadzone = 0.1;  // 摇杆死区值，默认为0.1
    double inactive_timeout = 30.0;  // 不活跃超时时间（秒）
    bool get_joy_data = false;
    bool joy_init_flag = false;
    bool parameters_loaded_ = false;  // 参数是否已加载
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr device_detect_timer_;  // 设备检测定时器
    rclcpp::Time last_joy_time_ = this->now();
    rclcpp::Time last_pub_msg_time_ = this->now();
    rclcpp::Time last_joy_init_time_ = this->now();
    rclcpp::Time last_activity_time_ = this->now();  // 最后活跃时间

    int zero_pub_count_no_data_ = 0;
    int zero_pub_count_disabled_ = 0;
    bool prev_no_data_ = false;      // 上一次是否是 no_data
    bool prev_not_enabled_ = false;  // 上一次是否是 not_enabled

    // 类内成员变量
    double last_valid_lr_ = 0.0;
    double last_valid_fb_ = 0.0;
    bool last_enable_button_state_ = false;        // 上次使能按键状态
    const double MAX_JUMP = 0.75;                  // 单帧允许的最大变化幅度（可调）
    bool init_pending_ = false;                    // 是否处于初始化待回零阶段
    rclcpp::Time init_complete_time_;              // 初始化完成时间（用于保护期检查）
    const double init_protection_duration_ = 0.5;  // 初始化保护期（秒）
    int init_print_count_ = 0;

    // 检查是否真正完成初始化（包括保护期）
    bool isInitReady() const {
        if (!joy_init_flag) return false;
        rclcpp::Time now = this->now();
        return (now - init_complete_time_).seconds() >= init_protection_duration_;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    printf("mobile_joy_log_control\n");
    auto node = std::make_shared<mobile_joy_log_control>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
