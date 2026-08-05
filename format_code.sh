#!/usr/bin/env bash
set -e

# 获取脚本所在根目录
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 检查 clang-format
if ! command -v clang-format &> /dev/null; then
    echo "❌ 未找到 clang-format 工具，请执行: sudo apt install clang-format"
    exit 1
fi

if [ -n "$1" ]; then
    # 格式化指定的单个文件
    TARGET_FILE="$1"
    if [ ! -f "$TARGET_FILE" ]; then
        echo "❌ 错误: 文件 '$TARGET_FILE' 不存在！"
        exit 1
    fi
    echo "🎨 正在按 .clang-format 规则格式化单个文件: $TARGET_FILE"
    clang-format -i --style=file "$TARGET_FILE"
    echo "✅ 文件 $TARGET_FILE 格式化完成！"
else
    # 格式化整个项目
    echo "🎨 正在按 .clang-format 规则格式化项目所有 C++/CUDA 代码..."
    find "${WORKSPACE_DIR}" \
      \( -path "*/build*" -o -path "*/.git*" -o -path "*/install*" -o -path "*/release*" \) -prune \
      -o \( -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" -o -name "*.cu" \) -type f -print0 | xargs -0 clang-format -i --style=file
    echo "✅ 所有 C++/CUDA 文件格式化完成！"
fi
