#!/usr/bin/env bash
set -e

WORKSPACE_DIR=$(pwd)
IMAGE_NAME="dexe_mobile_nav_builder:arm64"
DOCKERFILE_PATH="${WORKSPACE_DIR}/Dockerfile.arm64"
PASSWORD="dex@1234"

# === 1. 确保 binfmt 注册成功（支持多架构运行） ===
echo "⚙️ 检查并启用多架构支持..."
docker run --privileged --rm tonistiigi/binfmt --install all > /dev/null

# === 2. 检查并同步编译镜像 ===
echo "🐳 检查编译镜像 ${IMAGE_NAME}..."

# 计算 Dockerfile 和 package.xml 的 MD5 校验和，用于检测包配置和依赖文件的变化
CALC_HASH() {
    (cat "${DOCKERFILE_PATH}"; find "${WORKSPACE_DIR}" -maxdepth 4 -name "package.xml" -type f -exec cat {} +) | md5sum | cut -d' ' -f1
}

REBUILD_REQUIRED=false
HASH_FILE="${WORKSPACE_DIR}/.build_image_hash"

# 检查命令行参数是否有 --rebuild 或 -r
for arg in "$@"; do
    if [ "$arg" == "--rebuild" ] || [ "$arg" == "-r" ]; then
        echo "🔄 收到强制重建参数，开始重新构建镜像..."
        REBUILD_REQUIRED=true
    fi
done

if ! docker image inspect "${IMAGE_NAME}" > /dev/null 2>&1; then
    echo "🔨 本地未找到镜像，开始构建镜像..."
    REBUILD_REQUIRED=true
else
    # 镜像已存在，但检查依赖文件是否有更新
    if [ -f "${HASH_FILE}" ]; then
        CURRENT_HASH=$(CALC_HASH)
        SAVED_HASH=$(cat "${HASH_FILE}")
        if [ "${CURRENT_HASH}" != "${SAVED_HASH}" ]; then
            echo "🔄 检测到 Dockerfile.arm64 或 package.xml 依赖有更新，重新构建镜像..."
            REBUILD_REQUIRED=true
        fi
    else
        # 找不到哈希记录，但镜像已存在，为安全起见计算一次哈希以备后用
        CALC_HASH > "${HASH_FILE}"
    fi
fi

CONTAINER_NAME="dexe_mobile_nav_builder"

if [ "${REBUILD_REQUIRED}" = true ]; then
    echo "🔨 开始构建镜像 ${IMAGE_NAME}..."
    DOCKER_BUILDKIT=1 docker build --platform linux/arm64 -t "${IMAGE_NAME}" -f "${DOCKERFILE_PATH}" .
    CALC_HASH > "${HASH_FILE}"
    
    # 镜像重建后，必须销毁旧的持久化容器，保证下次启动时基于新镜像重建
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "🧹 检测到镜像已重建，正在销毁并移除旧的持久化容器..."
        docker rm -f "${CONTAINER_NAME}" > /dev/null
    fi
else
    echo "⚡ 检测到本地已存在镜像 ${IMAGE_NAME} 且依赖无变化，跳过构建直接开始编译！"
    echo "💡 提示：如果需要强制重建镜像并重新安装包，请运行: ./build_arm64.sh --rebuild"
fi
echo "✅ 编译镜像准备就绪！"

# === 3. 管理并运行持久化 Docker 容器进行 ARM64 交叉编译与打包 ===
echo "🚀 准备编译容器环境..."
# 赋予容器内编译脚本执行权限
chmod +x "${WORKSPACE_DIR}/build_internal.sh"

# 1. 检查持久化容器是否存在
if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "🆕 未找到持久化容器，正在创建并启动后台挂起容器 ${CONTAINER_NAME}..."
    # 使用 -d (后台运行) 和 --restart always (重启自启)
    docker run -d --privileged \
        --platform linux/arm64 \
        --restart always \
        --name "${CONTAINER_NAME}" \
        -v "${WORKSPACE_DIR}":/workspace \
        "${IMAGE_NAME}" \
        tail -f /dev/null
else
    # 2. 容器存在，检查是否运行。如果没运行则启动它
    if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}")" != "true" ]; then
        echo "🚀 正在启动已有的持久化编译容器 ${CONTAINER_NAME}..."
        docker start "${CONTAINER_NAME}"
    fi
fi

echo "⚙️ 正在持久化容器中执行交叉编译..."
# 检查是否是交互式终端环境，选择合适的 docker exec 参数
INTERACTIVE_FLAG="-i"
if [ -t 0 ]; then
    INTERACTIVE_FLAG="-it"
fi

docker exec ${INTERACTIVE_FLAG} "${CONTAINER_NAME}" /bin/bash /workspace/build_internal.sh

# === 4. 修复文件所有权 (Docker 内部以 root 运行，需将生成文件改回当前用户) ===
echo "🧹 正在修正生成的文件所有权..."
CURRENT_UID=$(id -u)
CURRENT_GID=$(id -g)
echo "${PASSWORD}" | sudo -S chown -R ${CURRENT_UID}:${CURRENT_GID} \
    "${WORKSPACE_DIR}/build_arm64" \
    "${WORKSPACE_DIR}/install_arm64" \
    "${WORKSPACE_DIR}/log_arm64" \
    "${WORKSPACE_DIR}/release" \
    2>/dev/null || true

echo "==============================================="
echo "🎉 ARM64 跨平台编译及部署包制作全部完成！"
echo "📂 您的部署包已保存在 ${WORKSPACE_DIR}/release 目录下："
echo "   - [install_arm64.tar.gz](file://${WORKSPACE_DIR}/release/install_arm64.tar.gz) (压缩包)"
echo "==============================================="
