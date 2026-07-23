#!/usr/bin/env python3
"""从 dexforce_description 的可视化 URDF 生成 W1 差速仿真 URDF。

DexforceW1V020.urdf 是纯可视化模型（轮关节 fixed、无 gazebo 插件），本脚本做以下变换：
1. 用 config/real_robot_default_tf.json 覆盖全部机器人内部 TF；主体来自真机 bag 首帧，
   前后雷达使用设备配置中的未标定初始外参；odom→base_footprint 是运动 TF，明确排除
2. WHEEL_LEFT / WHEEL_RIGHT：fixed → continuous（axis 统一 0 1 0，diff 插件正速度=前进）
3. 其余所有非 fixed 关节（上身手臂/手指/腰）→ fixed：底盘仿真不控上身，
   revolute 关节没有控制器时在重力下会瘫倒
4. wheel_left / wheel_right link 补圆柱 collision + visual（原模型轮子无几何，
   半径按模型自身几何链推导：base_joint z 0.010 + WHEEL z 0.053 = 0.063）
5. 碰撞体极简化（防物理 NaN）：删除全部原始 collision（43 个 .obj 网格——含手指/手臂，
   数值不稳定且底盘仿真不需要）；仅保留：轮圆柱 ×2 + 底盘盒 ×1 + 无摩擦万向球 ×4（skid）。
   skid 底面比驱动轮底面高 5mm，避免辅助支撑卸载驱动轮
6. 追加 gazebo 插件：diff_drive（订 /control/cmd_vel，发 /feedback/odom + odom→base_footprint TF）、
   joint_state（轮关节 → rsp 发轮 TF）、前后激光（/laser_scan/front|rear）、IMU（/imu/chassis）、
   前后 RealSense 风格 RGBD（/orbbec/{front,rear}/{rgb,depth,camera_info}）
7. 摩擦：轮 mu 1.0；万向球/底盘盒 mu 0（滑动支撑）

话题契约与原 buddi 模型完全一致 → w1_nav2_params_diff_sim.yaml 无需改动。

用法：python3 gen_w1_diff_sim_urdf.py [源URDF] [输出URDF] [真机TF配置]
默认源：本包内置快照 urdf/source/DexforceW1V020.urdf（自包含，不依赖 dexe_mobile_application；
W1 模型更新时把新 URDF + meshes/Visual 拷入本包后重跑本脚本）。
mesh URI 统一重写为 package://w1_diff_gazebo/...（meshes/Visual 已随包内置）。
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

WHEEL_SEPARATION = 0.5078   # WHEEL_LEFT/RIGHT origin y=±0.2539
WHEEL_RADIUS = 0.063        # base_joint z 0.010 + WHEEL 关节 z 0.053（轮心离地高）
WHEEL_WIDTH = 0.045
SKID_CENTER_Z = 0.015       # r=0.02；底面比驱动轮底面高 5mm，确保驱动轮承受法向载荷

DEFAULT_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "urdf", "source", "DexforceW1V020.urdf")
DEFAULT_TF_PROFILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "..", "config", "real_robot_default_tf.json")

GAZEBO_EXTRA = f"""
<extra>
  <gazebo>
    <plugin name="w1_diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <!-- ROS2 版插件话题名固定为 cmd_vel/odom，只能用 remapping 改名；
             <command_topic>/<odometry_topic> 是 ROS1 标签会被静默忽略（bug-1766）-->
        <remapping>cmd_vel:=/control/cmd_vel</remapping>
        <remapping>odom:=/feedback/odom</remapping>
      </ros>
      <update_rate>100</update_rate>
      <left_joint>WHEEL_LEFT</left_joint>
      <right_joint>WHEEL_RIGHT</right_joint>
      <wheel_separation>{WHEEL_SEPARATION}</wheel_separation>
      <wheel_diameter>{2 * WHEEL_RADIUS}</wheel_diameter>
      <max_wheel_torque>30</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
    </plugin>
    <plugin name="w1_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
      <ros>
        <remapping>~/out:=joint_states</remapping>
      </ros>
      <update_rate>30</update_rate>
      <joint_name>WHEEL_LEFT</joint_name>
      <joint_name>WHEEL_RIGHT</joint_name>
    </plugin>
  </gazebo>
  <gazebo reference="wheel_left">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
  <gazebo reference="wheel_right">
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>
  <gazebo reference="base_link">
    <mu1>0.0</mu1>
    <mu2>0.0</mu2>
  </gazebo>
  <gazebo reference="chassis_base_link">
    <mu1>0.0</mu1>
    <mu2>0.0</mu2>
  </gazebo>
  <gazebo reference="laser_front_link">
    <sensor name="laser_front" type="ray">
      <!-- laser link 的 TF 已直接使用初始外参，传感器不再叠加内部位姿补偿。 -->
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>15</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1.0</resolution>
            <min_angle>-2.356</min_angle>
            <max_angle>2.356</max_angle>
          </horizontal>
        </scan>
        <range>
          <!-- 自扫防护靠"碰撞盒顶面(0.17) < 激光扫描平面(0.2097)"保证（bug-1764），
               min 保持真实雷达量程即可 -->
          <min>0.12</min>
          <max>25.0</max>
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="w1_laser_front" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=/laser_scan/front</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_front_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  <gazebo reference="laser_rear_link">
    <sensor name="laser_rear" type="ray">
      <!-- laser link 的 TF 已直接使用初始外参，传感器不再叠加内部位姿补偿。 -->
      <pose>0 0 0 0 0 0</pose>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>15</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1.0</resolution>
            <min_angle>-2.356</min_angle>
            <max_angle>2.356</max_angle>
          </horizontal>
        </scan>
        <range>
          <!-- 自扫防护靠"碰撞盒顶面(0.17) < 激光扫描平面(0.2097)"保证（bug-1764），
               min 保持真实雷达量程即可 -->
          <min>0.12</min>
          <max>25.0</max>
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="w1_laser_rear" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=/laser_scan/rear</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_rear_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  <gazebo reference="chassis_depth_front_link">
    <sensor name="realsense_front" type="depth">
      <!-- 真机相机 link 是 optical frame（+Z 朝前），Gazebo camera 以局部 +X 为光轴。 -->
      <pose>0 0 0 -2.356195 -1.569518 -2.356195</pose>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>15</update_rate>
      <camera name="realsense">
        <horizontal_fov>1.500983</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <depth_camera />
        <clip>
          <near>0.1</near>
          <far>5.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.005</stddev>
        </noise>
      </camera>
      <plugin name="w1_realsense_front" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/orbbec/front</namespace>
          <!-- Gazebo ROS camera remap 的源名必须包含 camera_name 前缀。 -->
          <remapping>realsense/image_raw:=rgb</remapping>
          <remapping>realsense/depth/image_raw:=depth</remapping>
          <remapping>realsense/camera_info:=camera_info</remapping>
          <remapping>realsense/depth/camera_info:=depth/camera_info</remapping>
          <remapping>realsense/points:=points</remapping>
        </ros>
        <camera_name>realsense</camera_name>
        <frame_name>chassis_depth_front_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>5.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>
  <gazebo reference="chassis_depth_rear_link">
    <sensor name="realsense_rear" type="depth">
      <pose>0 0 0 2.356195 -1.569518 -0.785398</pose>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>15</update_rate>
      <camera name="realsense">
        <horizontal_fov>1.500983</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <depth_camera />
        <clip>
          <near>0.1</near>
          <far>5.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.005</stddev>
        </noise>
      </camera>
      <plugin name="w1_realsense_rear" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/orbbec/rear</namespace>
          <remapping>realsense/image_raw:=rgb</remapping>
          <remapping>realsense/depth/image_raw:=depth</remapping>
          <remapping>realsense/camera_info:=camera_info</remapping>
          <remapping>realsense/depth/camera_info:=depth/camera_info</remapping>
          <remapping>realsense/points:=points</remapping>
        </ros>
        <camera_name>realsense</camera_name>
        <frame_name>chassis_depth_rear_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>5.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>
  <gazebo reference="imu_chassis_link">
    <sensor name="w1_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="w1_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <remapping>~/out:=/imu/chassis</remapping>
        </ros>
        <frame_name>imu_chassis_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</extra>
"""


def make_wheel_geometry(radius, width):
    geoms = []
    for tag in ("visual", "collision"):
        el = ET.Element(tag)
        origin = ET.SubElement(el, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", "1.5707963267948966 0 0")
        geometry = ET.SubElement(el, "geometry")
        cyl = ET.SubElement(geometry, "cylinder")
        cyl.set("radius", str(radius))
        cyl.set("length", str(width))
        geoms.append(el)
    return geoms


def quaternion_to_rpy(qx, qy, qz, qw):
    """Convert quaternion to URDF fixed-axis RPY, including gimbal-lock cases."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        raise ValueError("TF profile contains a zero quaternion")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))

    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r01 = 2.0 * (qx * qy - qz * qw)
    r02 = 2.0 * (qx * qz + qy * qw)
    r10 = 2.0 * (qx * qy + qz * qw)
    r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
    r12 = 2.0 * (qy * qz - qx * qw)
    r20 = 2.0 * (qx * qz - qy * qw)
    r21 = 2.0 * (qy * qz + qx * qw)
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

    horizontal = math.hypot(r00, r10)
    if horizontal > 1e-10:
        return (math.atan2(r21, r22), math.atan2(-r20, horizontal),
                math.atan2(r10, r00))
    if r20 < 0.0:
        return math.atan2(r01, r02), math.pi / 2.0, 0.0
    return math.atan2(-r01, -r02), -math.pi / 2.0, 0.0


def load_tf_profile(path):
    with open(path, "r", encoding="utf-8") as stream:
        document = json.load(stream)
    transforms = document.get("transforms", {})
    if not transforms:
        raise ValueError(f"TF profile has no transforms: {path}")
    for edge, values in transforms.items():
        if len(values) != 7 or not all(isinstance(value, (int, float)) for value in values):
            raise ValueError(f"Invalid TF profile entry {edge}: expected xyz+xyzw")
    return transforms


def apply_tf_profile(root, transforms):
    joints_by_edge = {}
    for joint in root.iter("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"Joint {joint.get('name')} lacks parent or child")
        edge = f"{parent.get('link')}->{child.get('link')}"
        if edge in joints_by_edge:
            raise ValueError(f"Duplicate URDF edge: {edge}")
        joints_by_edge[edge] = joint

    missing_from_profile = sorted(set(joints_by_edge) - set(transforms))
    missing_from_urdf = sorted(set(transforms) - set(joints_by_edge))
    if missing_from_profile or missing_from_urdf:
        raise ValueError(
            "Real-robot TF profile and URDF topology differ; "
            f"missing profile edges={missing_from_profile}, "
            f"missing URDF edges={missing_from_urdf}")

    for edge, joint in joints_by_edge.items():
        x, y, z, qx, qy, qz, qw = transforms[edge]
        roll, pitch, yaw = quaternion_to_rpy(qx, qy, qz, qw)
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
        origin.set("xyz", f"{x:.17g} {y:.17g} {z:.17g}")
        origin.set("rpy", f"{roll:.17g} {pitch:.17g} {yaw:.17g}")

    return len(joints_by_edge)


def main():
    src = (sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else DEFAULT_SRC)
    dst = (sys.argv[2] if len(sys.argv) > 2 and sys.argv[2]
           else os.path.join(os.path.dirname(DEFAULT_SRC), "..", "w1_diff_sim.urdf"))
    tf_profile = (sys.argv[3] if len(sys.argv) > 3 and sys.argv[3]
                  else DEFAULT_TF_PROFILE)
    tree = ET.parse(src)
    root = tree.getroot()
    root.set("name", "w1_diff_sim")

    n_profile = apply_tf_profile(root, load_tf_profile(tf_profile))

    # mesh URI 重写：description 已内置本包（meshes/Visual），不再依赖 dexforce_description
    n_uri = 0
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename", "")
        if fn.startswith("package://dexforce_description/"):
            mesh.set("filename", fn.replace(
                "package://dexforce_description/", "package://w1_diff_gazebo/"))
            n_uri += 1

    n_wheel, n_fixed = 0, 0
    for joint in root.iter("joint"):
        name = joint.get("name")
        if name in ("WHEEL_LEFT", "WHEEL_RIGHT"):
            joint.set("type", "continuous")
            axis = joint.find("axis")
            if axis is None:
                axis = ET.SubElement(joint, "axis")
            axis.set("xyz", "0 1 0")
            for tag in ("limit", "safety_controller", "dynamics"):
                el = joint.find(tag)
                if el is not None:
                    joint.remove(el)
            n_wheel += 1
        elif joint.get("type") not in ("fixed",):
            joint.set("type", "fixed")
            for tag in ("limit", "safety_controller", "dynamics", "mimic"):
                el = joint.find(tag)
                if el is not None:
                    joint.remove(el)
            n_fixed += 1

    # 碰撞体极简化：删除所有原始 collision（.obj 全身网格会导致物理数值发散/NaN）
    n_coll_removed = 0
    for link in root.iter("link"):
        for coll in list(link.findall("collision")):
            link.remove(coll)
            n_coll_removed += 1

    # 惯量极简化：SolidWorks 导出的 53 份惯量存在非正定风险（加驱动力即 NaN 的实测根因方向）。
    # 上身已全部 fixed 并被 gazebo lump，物理只需：轮 ×2（保留原惯量）+ 底盘一个干净的盒惯量。
    for link in root.iter("link"):
        name = link.get("name")
        if name in ("wheel_left", "wheel_right"):
            continue
        inr = link.find("inertial")
        if inr is not None:
            link.remove(inr)
        if name == "chassis_base_link":
            # 整机集中质量：60kg，com z 0.30，盒 0.7×0.5×1.2 惯量
            link.append(ET.fromstring(
                '<inertial><origin xyz="0 0 0.30" rpy="0 0 0"/><mass value="60.0"/>'
                '<inertia ixx="8.45" ixy="0" ixz="0" iyy="9.65" iyz="0" izz="3.70"/></inertial>'))

    for link in root.iter("link"):
        if link.get("name") in ("wheel_left", "wheel_right"):
            for el in make_wheel_geometry(WHEEL_RADIUS, WHEEL_WIDTH):
                link.append(el)
        elif link.get("name") == "chassis_base_link":
            # 底盘盒（对外碰撞用）：0.70 x 0.50 x 0.14，z∈[0.03,0.17]——顶面必须低于
            # 激光扫描平面 z=0.2097（激光倒装于四角），否则激光自扫碰撞盒把机器人
            # 自己标成 costmap 障碍（bug-1764，实测 78% 波束 <0.6m 全是盒内壁/外壁）
            box = ET.fromstring(
                '<collision><origin xyz="0 0 0.10" rpy="0 0 0"/>'
                '<geometry><box size="0.70 0.50 0.14"/></geometry></collision>')
            link.append(box)
            # 4 个无摩擦万向球（skid）：r 0.02，球底比轮底高 5mm，避免辅助支撑卸载驱动轮。
            for sx in (0.28, -0.28):
                for sy in (0.18, -0.18):
                    sph = ET.fromstring(
                        f'<collision><origin xyz="{sx} {sy} {SKID_CENTER_Z}" rpy="0 0 0"/>'
                        '<geometry><sphere radius="0.02"/></geometry></collision>')
                    link.append(sph)

    extra = ET.fromstring(GAZEBO_EXTRA)
    for el in list(extra):
        root.append(el)

    ET.indent(tree, space="  ")
    tree.write(dst, encoding="unicode", xml_declaration=False)
    print(f"生成 {dst}：真机 TF ×{n_profile}，轮关节 continuous ×{n_wheel}，"
          f"上身关节转 fixed ×{n_fixed}，"
          f"mesh URI 重写 ×{n_uri}，separation={WHEEL_SEPARATION} radius={WHEEL_RADIUS}")


if __name__ == "__main__":
    main()
