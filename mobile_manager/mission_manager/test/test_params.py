#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试参数文件加载
"""

import os
import sys
import yaml


def test_param_file():
    """测试参数文件格式和内容"""
    # 获取参数文件路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    param_file = os.path.join(package_dir, 'params',
                              'mission_manager_params.yaml')

    print(f"测试参数文件: {param_file}")

    # 检查文件是否存在
    if not os.path.exists(param_file):
        print(f"❌ 参数文件不存在: {param_file}")
        return False

    print(f"✅ 参数文件存在")

    # 加载YAML文件
    try:
        with open(param_file, 'r', encoding='utf-8') as f:
            params = yaml.safe_load(f)
    except Exception as e:
        print(f"❌ 加载参数文件失败: {e}")
        return False

    print(f"✅ 参数文件加载成功")

    # 检查参数结构
    if '/**' not in params or 'ros__parameters' not in params['/**']:
        print(f"❌ 参数文件格式不正确")
        return False

    print(f"✅ 参数文件格式正确")

    # 获取参数
    ros_params = params['/**']['ros__parameters']

    # 检查mission_manager参数
    if 'mission_manager' not in ros_params:
        print(f"❌ 缺少mission_manager参数组")
        return False

    print(f"✅ mission_manager参数组存在")

    mm_params = ros_params['mission_manager']

    # 检查必需参数
    required_params = {
        'auto_charge_threshold': float,
        'force_charge_threshold': float,
        'respawn_task': bool
    }

    for param_name, param_type in required_params.items():
        if param_name not in mm_params:
            print(f"❌ 缺少参数: {param_name}")
            return False

        value = mm_params[param_name]
        if not isinstance(value, param_type):
            print(f"❌ 参数类型错误: {param_name}, 期望{param_type}, 实际{type(value)}")
            return False

        print(f"✅ 参数 {param_name} = {value} ({param_type.__name__})")

    # 检查参数值的合理性
    if mm_params['auto_charge_threshold'] <= 0 or mm_params[
            'auto_charge_threshold'] > 100:
        print(
            f"⚠️  警告: auto_charge_threshold值可能不合理: {mm_params['auto_charge_threshold']}"
        )

    if mm_params['force_charge_threshold'] <= 0 or mm_params[
            'force_charge_threshold'] > 100:
        print(
            f"⚠️  警告: force_charge_threshold值可能不合理: {mm_params['force_charge_threshold']}"
        )

    if mm_params['force_charge_threshold'] >= mm_params[
            'auto_charge_threshold']:
        print(f"⚠️  警告: force_charge_threshold应该小于auto_charge_threshold")

    print("\n" + "=" * 50)
    print("参数文件测试完成")
    print("=" * 50)
    print(f"\n参数总结:")
    print(f"  自动充电阈值: {mm_params['auto_charge_threshold']}%")
    print(f"  强制充电阈值: {mm_params['force_charge_threshold']}%")
    print(f"  任务重启: {mm_params['respawn_task']}")

    return True


if __name__ == '__main__':
    success = test_param_file()
    sys.exit(0 if success else 1)
