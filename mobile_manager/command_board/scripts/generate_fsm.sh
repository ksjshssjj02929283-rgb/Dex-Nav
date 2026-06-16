#!/usr/bin/env bash


# 检查Java环境
if ! command -v java &> /dev/null; then
    echo "错误: 未找到Java运行时环境。请先安装Java。" >&2
    exit 1
fi

# 检查Java版本
java_version=$(java -version 2>&1 | head -n 1 | cut -d'"' -f2)
if [ -z "$java_version" ]; then
    echo "警告: 无法确定Java版本" >&2
else
    echo "检测到Java版本: $java_version"
    # 检查是否为合适的Java版本（例如至少为1.8）
    if [[ "$java_version" < "1.8" ]]; then
        echo "警告: 建议使用Java 1.8或更高版本，当前版本为: $java_version" >&2
    fi
fi


OUT_DIR_H=../include/fsm/
OUT_DIR_SRC=../src/fsm/
SMC=Smc.jar



# 查找当前目录下的所有 .sm 文件并处理
for sm_file in ./*.sm; do
    # 检查是否存在 .sm 文件
    if [ ! -e "$sm_file" ]; then
        echo "警告: 当前目录下没有找到 .sm 文件" >&2
        continue
    fi
    
    echo "正在处理状态机文件: $sm_file"
    
    # 检查 .sm 文件是否存在
    if [ ! -f "$sm_file" ]; then
        echo "跳过: $sm_file 不是一个有效的文件" >&2
        continue
    fi
    
    # 使用 SMC 生成代码
    java -jar "./$SMC" -c++ "$sm_file"
    
    # 移动生成的文件到相应目录
    if ls ./*.h >/dev/null 2>&1; then
        echo "移动头文件到: $OUT_DIR_H"
        mv ./*.h "$OUT_DIR_H"
    fi
    
    if ls ./*.cpp >/dev/null 2>&1; then
        echo "移动源文件到: $OUT_DIR_SRC"
        mv ./*.cpp "$OUT_DIR_SRC"
    fi
done

echo "状态机代码生成完成"