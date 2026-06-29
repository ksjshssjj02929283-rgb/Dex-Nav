#!/bin/bash

# 定位操作脚本 - Localization Operation Script
# 用于控制机器人的定位初始化流程

set -e

# 颜色输出定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${GREEN}===================================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}===================================================${NC}"
}

# 加载ROS2环境
source_ros2_environment() {
    local ros2_setup_files=(
        "/opt/ros/humble/setup.bash"
    )

    for setup_file in "${ros2_setup_files[@]}"; do
        if [ -f "$setup_file" ]; then
            source "$setup_file" >/dev/null 2>&1
            print_success "已加载ROS2环境: $setup_file"

            local workspace_setup="/home/dexforce/w1/install/setup.bash"
            if [ -f "$workspace_setup" ]; then
                source "$workspace_setup" >/dev/null 2>&1
                print_success "已加载工作空间环境: $workspace_setup"
            fi

            return 0
        fi
    done

    print_error "未找到ROS2安装，请确保ROS2已正确安装"
    return 1
}

# 步骤1: 关闭建图/标定服务，切换至自动定位模式
step1_switch_to_auto_mode() {
    print_step "步骤 1: 切换至自动定位模式"

    # ---- 提前触发一次 sudo 认证，后续所有 sudo 命令无需再次输入密码 ----
    print_info "需要 sudo 权限来管理系统服务，请输入密码："
    sudo -v || { print_error "sudo 认证失败"; exit 1; }

    print_info "正在停止 dexe-map.service、dexe-calib.service..."
    sudo systemctl stop dexe-map.service 2>/dev/null && print_success "dexe-map.service 已停止" \
        || print_warning "dexe-map.service 停止失败或不存在，继续..."

    sudo systemctl stop dexe-calib.service 2>/dev/null && print_success "dexe-calib.service 已停止" \
        || print_warning "dexe-calib.service 停止失败或不存在，继续..."

    print_info "正在重启 dexe-auto.service..."
    if sudo systemctl restart dexe-auto.service; then
        print_success "dexe-auto.service 已重启"
    else
        print_error "dexe-auto.service 重启失败"
        exit 1
    fi

    # ---- 等待 map_manager 节点上线 ----
    print_info "等待 map_manager 节点启动..."
    local node_timeout=60
    local node_elapsed=0
    while [ "$node_elapsed" -lt "$node_timeout" ]; do
        if ros2 node list 2>/dev/null | grep -q "map_manager"; then
            print_success "map_manager 节点已上线"
            break
        fi
        printf "\r  [%02ds] 等待节点..." "$node_elapsed"
        sleep 2
        node_elapsed=$((node_elapsed + 2))
    done
    echo ""
    if [ "$node_elapsed" -ge "$node_timeout" ]; then
        print_error "map_manager 节点未能在 ${node_timeout}s 内启动，请检查 dexe-auto.service 日志"
        print_info "  journalctl -u dexe-auto.service -n 50 --no-pager"
        exit 1
    fi

    # ---- 等待 /map_manager/get_map_list 服务就绪 ----
    print_info "等待 /map_manager/get_map_list 服务就绪..."
    local svc_timeout=60
    local svc_elapsed=0
    while [ "$svc_elapsed" -lt "$svc_timeout" ]; do
        if ros2 service list 2>/dev/null | grep -q "/map_manager/get_map_list"; then
            print_success "/map_manager/get_map_list 服务已就绪"
            break
        fi
        printf "\r  [%02ds] 等待服务..." "$svc_elapsed"
        sleep 2
        svc_elapsed=$((svc_elapsed + 2))
    done
    echo ""
    if [ "$svc_elapsed" -ge "$svc_timeout" ]; then
        print_error "/map_manager/get_map_list 服务未能在 ${svc_timeout}s 内就绪"
        exit 1
    fi

    print_success "自动定位模式就绪，所有节点和服务均已启动"
    echo ""
}

# 步骤2: 获取地图列表并显示默认地图名称
step2_get_map_list() {
    print_step "步骤 2: 获取地图列表"

    # 获取默认地图名称
    print_info "正在查询 map_manager 默认地图名称..."
    local map_info_output
    map_info_output=$(ros2 service call /map_manager/get_current_map_info nav_interfaces/srv/GetMapInfo 2>&1) || true

    local default_map_name=""
    # 响应格式为 Python 对象风格: map_name='xxx' 或 map_name="xxx"
    if echo "$map_info_output" | grep -qE "map_name="; then
        default_map_name=$(echo "$map_info_output" | grep -oP "map_name='[^']*'" | head -1 | sed "s/map_name='//;s/'//")
        # 兼容双引号格式
        if [ -z "$default_map_name" ]; then
            default_map_name=$(echo "$map_info_output" | grep -oP 'map_name="[^"]*"' | head -1 | sed 's/map_name="//;s/"//')
        fi
    fi

    if [ -n "$default_map_name" ] && [ "$default_map_name" != "" ]; then
        print_info "map_manager 当前/默认地图名称: ${YELLOW}${default_map_name}${NC}"
    else
        print_warning "未能获取到默认地图名称（map_manager 可能尚未完全启动）"
        default_map_name=""
    fi

    # 获取地图列表
    print_info "正在获取可用地图列表..."
    local map_list_output
    map_list_output=$(ros2 service call /map_manager/get_map_list nav_interfaces/srv/GetMapList 2>&1) || true

    if ! echo "$map_list_output" | grep -qE "map_list=\[|map_list:|success=True|success: [Tt]rue"; then
        print_error "获取地图列表失败，请检查 map_manager 是否正常运行"
        print_info "原始响应: ${map_list_output}"
        print_info "提示: 可以运行 'ros2 service list | grep map_manager' 查看可用服务"
        exit 1
    fi

    # 解析地图列表
    # 响应格式1(Python对象风格): map_list=['name1', 'name2', ...]
    # 响应格式2(YAML风格):       map_list:\n  - name1
    mapfile -t MAP_ARRAY < <(
        # 先提取 map_list=[...] 括号内的全部内容（用 sed 跨行也能处理）
        echo "$map_list_output" \
        | tr -d '\n' \
        | grep -oP "map_list=\[([^\]]*)\]" \
        | grep -oP "'[^']+'" \
        | tr -d "'"
    )
    # 兼容双引号格式: map_list=["name1", ...]
    if [ ${#MAP_ARRAY[@]} -eq 0 ]; then
        mapfile -t MAP_ARRAY < <(
            echo "$map_list_output" \
            | tr -d '\n' \
            | grep -oP 'map_list=\[([^\]]*)\]' \
            | grep -oP '"[^"]+"' \
            | tr -d '"'
        )
    fi
    # 兼容 YAML 列表格式: - name
    if [ ${#MAP_ARRAY[@]} -eq 0 ]; then
        mapfile -t MAP_ARRAY < <(
            echo "$map_list_output" \
            | grep -oP "(?<=- )['\"]?[a-zA-Z0-9_\-\.]+['\"]?" \
            | tr -d "'" | tr -d '"'
        )
    fi

    if [ ${#MAP_ARRAY[@]} -eq 0 ]; then
        print_warning "地图列表为空，将使用默认地图（如有）"
    else
        echo ""
        echo -e "${YELLOW}可用地图列表:${NC}"
        for i in "${!MAP_ARRAY[@]}"; do
            local marker=""
            if [ "${MAP_ARRAY[$i]}" = "$default_map_name" ]; then
                marker=" ${GREEN}(默认)${NC}"
            fi
            echo -e "  ${BLUE}[$((i+1))]${NC} ${MAP_ARRAY[$i]}${marker}"
        done
    fi

    echo ""
    # 将解析结果和默认地图名导出供后续步骤使用
    DEFAULT_MAP_NAME="$default_map_name"
    echo ""
}

# 步骤3: 选择地图并加载
step3_select_and_load_map() {
    print_step "步骤 3: 选择并加载地图"

    local selected_map=""

    if [ ${#MAP_ARRAY[@]} -eq 0 ]; then
        print_warning "无可用地图，跳过地图选择，使用 map_manager 默认配置"
        echo ""
        return 0
    fi

    echo -e "${YELLOW}请输入要加载的地图编号，或直接按回车键使用默认地图 [${DEFAULT_MAP_NAME:-无默认}]:${NC}"
    read -r -p "地图编号 (1-${#MAP_ARRAY[@]}): " user_choice

    if [ -z "$user_choice" ]; then
        if [ -n "$DEFAULT_MAP_NAME" ]; then
            selected_map="$DEFAULT_MAP_NAME"
            print_info "使用默认地图: ${selected_map}"
        else
            print_info "未指定地图，跳过加载，使用 map_manager 内置默认配置"
            echo ""
            return 0
        fi
    else
        # 验证输入是否为有效数字
        if ! [[ "$user_choice" =~ ^[0-9]+$ ]] || \
           [ "$user_choice" -lt 1 ] || \
           [ "$user_choice" -gt "${#MAP_ARRAY[@]}" ]; then
            print_error "无效的选择: ${user_choice}，请输入 1 到 ${#MAP_ARRAY[@]} 之间的数字"
            exit 1
        fi
        selected_map="${MAP_ARRAY[$((user_choice-1))]}"
        print_info "已选择地图: ${selected_map}"
    fi

    # 调用 map_manager/load_map 服务加载所选地图
    print_info "正在加载地图: ${selected_map}..."
    local load_output
    load_output=$(ros2 service call /map_manager/load_map nav_interfaces/srv/LoadMap \
        "{map_name: '${selected_map}'}" 2>&1) || true

    if echo "$load_output" | grep -qE "success=True|success: [Tt]rue"; then
        print_success "地图 '${selected_map}' 加载成功"
    elif echo "$load_output" | grep -qE "success=False|success: [Ff]alse"; then
        print_error "地图加载失败（map_manager 返回 success=False）"
        print_info "原始响应: ${load_output}"
        exit 1
    else
        print_error "地图加载失败，未收到有效响应"
        print_info "原始响应: ${load_output}"
        exit 1
    fi

    LOADED_MAP_NAME="$selected_map"
    echo ""
}

# 步骤3b: 可选 — 构建节点图（distance map + nodemap）
step3b_build_node_map() {
    if [ -z "${LOADED_MAP_NAME:-}" ]; then
        return 0
    fi

    print_step "步骤 3b: 节点图构建（可选）"

    local map_yaml="${HOME}/.dexe/data/maps/${LOADED_MAP_NAME}/map.yaml"
    if [ ! -f "$map_yaml" ]; then
        print_warning "未找到地图文件: ${map_yaml}，跳过节点图构建"
        echo ""
        return 0
    fi

    local preprocessed_dir="${HOME}/.dexe/data/maps/${LOADED_MAP_NAME}/preprocessed"
    local global_nodemap="${preprocessed_dir}/nodemap_global.yaml"

    if [ -f "$global_nodemap" ]; then
        print_info "已存在节点图: ${global_nodemap}"
        echo -e "${YELLOW}是否重新构建节点图? [y/N]:${NC}"
        read -r rebuild_choice
        if [[ ! "$rebuild_choice" =~ ^[Yy]$ ]]; then
            print_info "使用已有节点图"
            USE_NODE_MAP=true
            NODE_MAP_YAML="$global_nodemap"
            echo ""
            return 0
        fi
    else
        echo -e "${YELLOW}是否为该地图构建节点图（distance map + 双层 nodemap）? [Y/n]:${NC}"
        read -r build_choice
        if [[ "$build_choice" =~ ^[Nn]$ ]]; then
            print_info "跳过节点图构建，规划将使用原始栅格 Theta*"
            USE_NODE_MAP=false
            echo ""
            return 0
        fi
    fi

    print_info "正在构建节点图: ${map_yaml}"
    if ros2 run nav2_map_server map_preprocess --map-yaml "$map_yaml"; then
        print_success "节点图构建完成: ${preprocessed_dir}"
        USE_NODE_MAP=true
        NODE_MAP_YAML="$global_nodemap"
    else
        print_error "节点图构建失败，规划将回退到栅格 Theta*"
        USE_NODE_MAP=false
    fi
    echo ""
}

# 步骤3c: 将节点图参数写入 planner（若已构建）
step3c_apply_planner_node_map() {
    if [ "${USE_NODE_MAP:-false}" != "true" ] || [ -z "${NODE_MAP_YAML:-}" ]; then
        return 0
    fi

    print_info "配置 Theta* 使用节点图: ${NODE_MAP_YAML}"

    local dist_yaml
    dist_yaml="$(dirname "$NODE_MAP_YAML")/distance_map.yaml"

    ros2 param set /planner_server ThetaStarBased.use_node_map true 2>/dev/null || \
        print_warning "无法设置 use_node_map（planner_server 可能未启动，请稍后在 nav 参数中配置）"

    ros2 param set /planner_server ThetaStarBased.node_map_yaml "$NODE_MAP_YAML" 2>/dev/null || true
    ros2 param set /planner_server ThetaStarBased.distance_map_yaml "$dist_yaml" 2>/dev/null || true

    print_success "已请求 planner 使用节点图（若 nav 已运行则立即生效）"
    echo ""
}

# 步骤4: 等待定位成功
# 说明：LocalizationState.msg 中 LOC_INITIALIZED=2（初始化成功），LOC_SUCCESS=4（定位成功）
# 本脚本等待 status >= 2（初始化成功），同时提示最终定位状态
step4_wait_for_localization() {
    print_step "步骤 4: 等待定位成功"

    local timeout=120      # 最长等待时间（秒）
    local poll_interval=2  # 轮询间隔（秒）
    local elapsed=0
    local loc_status=-99

    print_info "开始等待定位，超时时间: ${timeout}s"
    print_info "定位状态说明:"
    echo -e "  ${BLUE}0${NC} - 未初始化"
    echo -e "  ${BLUE}1${NC} - 初始化中"
    echo -e "  ${YELLOW}2${NC} - 初始化成功 (LOC_INITIALIZED)"
    echo -e "  ${YELLOW}3${NC} - 重定位中"
    echo -e "  ${GREEN}4${NC} - 定位成功 (LOC_SUCCESS) ✓"
    echo -e "  ${RED}-1${NC} - 初始化失败"
    echo -e "  ${RED}-2${NC} - 定位失败"
    echo ""

    while [ "$elapsed" -lt "$timeout" ]; do
        local topic_output
        topic_output=$(timeout 3 ros2 topic echo /localization_state nav_interfaces/msg/LocalizationState \
            --once 2>/dev/null) || true

        if [ -n "$topic_output" ]; then
            # 兼容两种格式:
            # Python对象风格: LocalizationState(status=4, ...)
            # YAML风格:       status: 4
            loc_status=$(echo "$topic_output" \
                | grep -oP "(?<=status=)-?[0-9]+" | head -1)
            if [ -z "$loc_status" ]; then
                loc_status=$(echo "$topic_output" \
                    | grep -oP "(?<=status: )-?[0-9]+" | head -1)
            fi
        fi

        if [ -n "$loc_status" ] && [ "$loc_status" != "-99" ]; then
            case "$loc_status" in
                4)
                    echo ""
                    print_success "定位成功！(LOC_SUCCESS, status=4)"
                    return 0
                    ;;
                2)
                    print_info "[${elapsed}s] 定位初始化成功 (LOC_INITIALIZED, status=2)，等待完全定位..."
                    ;;
                3)
                    print_info "[${elapsed}s] 重定位中 (LOC_RELOCALIZING, status=3)..."
                    ;;
                1)
                    print_info "[${elapsed}s] 初始化中 (LOC_INITIALIZING, status=1)..."
                    ;;
                0)
                    print_info "[${elapsed}s] 等待初始化 (LOC_NOT_INIT, status=0)..."
                    ;;
                -1)
                    echo ""
                    print_error "定位初始化失败 (LOC_INIT_FAILED, status=-1)"
                    exit 1
                    ;;
                -2)
                    echo ""
                    print_error "定位失败 (LOC_FAILED, status=-2)"
                    exit 1
                    ;;
                *)
                    print_info "[${elapsed}s] 当前定位状态: status=${loc_status}..."
                    ;;
            esac
        else
            print_info "[${elapsed}s] 等待 /localization_state 话题消息..."
        fi

        sleep "$poll_interval"
        elapsed=$((elapsed + poll_interval))
    done

    echo ""
    print_error "等待定位超时（已等待 ${timeout}s），最后状态: status=${loc_status}"
    print_info "提示: 可运行 'ros2 topic echo /localization_state' 手动查看定位状态"
    exit 1
}

# 主函数
main() {
    clear
    echo -e "${BLUE}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║              DEXE 机器人定位操作脚本                     ║${NC}"
    echo -e "${BLUE}║         DEXE Robot Localization Operation Script         ║${NC}"
    echo -e "${BLUE}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""

    # 加载ROS2环境
    print_info "正在加载ROS2环境..."
    if ! source_ros2_environment; then
        exit 1
    fi
    echo ""

    # 步骤1: 关闭建图/标定服务，启动自动模式
    step1_switch_to_auto_mode

    # 步骤2: 获取地图列表（结果存入 MAP_ARRAY / DEFAULT_MAP_NAME）
    step2_get_map_list

    # 步骤3: 选择并加载地图
    step3_select_and_load_map

    # 步骤3b/3c: 可选节点图构建 + 配置 planner
    step3b_build_node_map
    step3c_apply_planner_node_map

    # 步骤4: 等待定位成功
    step4_wait_for_localization

    # 完成提示
    echo -e "${GREEN}╔═══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                   定位流程已完成！                        ║${NC}"
    echo -e "${GREEN}║                                                           ║${NC}"
    echo -e "${GREEN}║  当前地图: ${LOADED_MAP_NAME:-默认地图}${NC}"
    if [ "${USE_NODE_MAP:-false}" = "true" ]; then
        echo -e "${GREEN}║  节点图:   ${NODE_MAP_YAML}${NC}"
    fi
    echo -e "${GREEN}║                                                           ║${NC}"
    echo -e "${GREEN}║  后续操作:                                                ║${NC}"
    echo -e "${GREEN}║  - 可以开始导航任务                                       ║${NC}"
    echo -e "${GREEN}║  - 如需重新定位，请再次运行此脚本                         ║${NC}"
    echo -e "${GREEN}║  - 如需建图，请运行 mapping_operation.sh                  ║${NC}"
    echo -e "${GREEN}╚═══════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# 运行主函数
main
