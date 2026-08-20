      
#!/bin/bash

CURDIR=$(pwd)
PROJECT_DIR=$(basename "$PWD")
PACKAGE_NAME=${PROJECT_DIR//_/-}
echo "Running file installation for $PACKAGE_NAME..."


mkdir -p "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PACKAGE_NAME"
mkdir -p "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/install"

if [ ! -d "$CURDIR/install" ]; then
    echo "Error: install directory not found!"
    exit 1
fi

find "$CURDIR/install" -maxdepth 1 -type f -exec cp {} "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/$PACKAGE_NAME/" \;
find "$CURDIR/install" -maxdepth 1 -mindepth 1 -type d -exec cp -r {} "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/install/" \;

# --- 剔除 dexe_interfaces 接口包: 机上接口由 dexe-interfaces deb 铺设, 本包不重复发布 ---
# 包名以 package.xml 的 <name> 为准, 目录名与包名可能不一致(dexe_interfaces_meta -> dexe_interfaces)。
# colcon 的 install/setup.bash 动态枚举包目录, 删子目录不影响 source。
if [ -d "$CURDIR/dexe_interfaces" ]; then
    for xml in "$CURDIR"/dexe_interfaces/*/package.xml; do
        [ -f "$xml" ] || continue
        pkg=$(sed -n 's|.*<name>\([^<]*\)</name>.*|\1|p' "$xml" | head -n1)
        [ -n "$pkg" ] || continue
        echo "Excluding interface package from deb: $pkg"
        rm -rf "$CURDIR/debian/$PACKAGE_NAME/home/dexforce/w1/install/$pkg"
    done
fi

# --- 自动生成 postinst:整棵 /home/dexforce/w1 chown 回 dexforce ---
DEBIAN_DIR="$CURDIR/debian/$PACKAGE_NAME/DEBIAN"
mkdir -p "$DEBIAN_DIR"
cat > "$DEBIAN_DIR/postinst" <<'EOF'
#!/bin/sh
set -e
chown -R dexforce:dexforce /home/dexforce/w1
EOF
chmod 0755 "$DEBIAN_DIR/postinst"

# --- 自动生成 control:版本号从 dexe_mobile_nav_meta/package.xml 读取 ---
VERSION=$(grep -m1 '<version>' "${CURDIR}/dexe_mobile_nav_meta/package.xml" | sed 's/.*<version>\(.*\)<\/version>.*/\1/')
cat > "$DEBIAN_DIR/control" <<EOF
Package: $PACKAGE_NAME
Version: ${VERSION}
Architecture: arm64
Maintainer: Dexforce <dexforce@dexforce.com>
Description: $PACKAGE_NAME
EOF
