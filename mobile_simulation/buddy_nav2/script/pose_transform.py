import numpy as np
import math

def euler_to_rotation_matrix(roll, pitch, yaw):
    # 将欧拉角转换为旋转矩阵
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]])

    R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                    [0, 1, 0],
                    [-math.sin(pitch), 0, math.cos(pitch)]])

    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])

    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def pose_transform(initial_pose, relative_pose):
    # 将初始位姿和相对位姿变换为新的位姿
    x, y, z, roll, pitch, yaw = initial_pose
    dx, dy, dz, droll, dpitch, dyaw = relative_pose

    # 计算旋转矩阵
    R = euler_to_rotation_matrix(roll, pitch, yaw)

    # 计算位姿变换
    translation_vector = np.array([dx, dy, dz])
    rotated_translation = np.dot(R, translation_vector)

    new_x = x + rotated_translation[0]
    new_y = y + rotated_translation[1]
    new_z = z + rotated_translation[2]

    new_roll = roll + droll
    new_pitch = pitch + dpitch
    new_yaw = yaw + dyaw

    return new_x, new_y, new_z, new_roll, new_pitch, new_yaw

# 示例
initial_pose = (0, 0, 0, 0, 0, 0)  # 初始位姿
relative_forward_lidar_pose = (0.47503, 0.27579, 0.2097, 3.14159, 0, 0.7854)  # 相对位姿变化
relative_back_lidar_pose = (-0.47656, -0.27579, 0.2097, -3.14159, 0, -2.3562)

forward_lidar_pose = pose_transform(initial_pose, relative_forward_lidar_pose)
back_lidar_pose = pose_transform(initial_pose, relative_back_lidar_pose)
print("forward_lidar_pose：", forward_lidar_pose)
print("back_lidar_pose：", back_lidar_pose)
