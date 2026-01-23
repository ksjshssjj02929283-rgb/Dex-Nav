from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
import numpy as np
from math import pi, sqrt, sin, cos, atan2


class PoseModifier:

    def add_yaw(self, orin_pose, yaw, order='zyx'):
        # Create a rotation representing the yaw
        q_yaw = R.from_euler(order, [yaw, 0, 0], degrees=False)

        # Create a rotation from the pose's orientation quaternion
        q1 = R.from_quat(
            [orin_pose.orientation.x, orin_pose.orientation.y, orin_pose.orientation.z, orin_pose.orientation.w])

        # Apply the yaw rotation to the pose's orientation
        q_new = q_yaw * q1

        # Normalize the resulting quaternion
        q_new = R.from_quat(q_new.as_quat() / np.linalg.norm(q_new.as_quat()))

        # Update the pose's orientation
        tar_pose = Pose()
        tar_pose.orientation.x = q_new.as_quat()[0]
        tar_pose.orientation.y = q_new.as_quat()[1]
        tar_pose.orientation.z = q_new.as_quat()[2]
        tar_pose.orientation.w = q_new.as_quat()[3]

        return tar_pose

    def compute_rotation_error(self, tar_pose, orin_pose, order='zyx'):
        # Create rotations from the poses' orientation quaternions
        q_tar = R.from_quat(
            [tar_pose.orientation.x, tar_pose.orientation.y, tar_pose.orientation.z, tar_pose.orientation.w])
        q_orin = R.from_quat(
            [orin_pose.orientation.x, orin_pose.orientation.y, orin_pose.orientation.z, orin_pose.orientation.w])

        # Compute the rotation error
        q_error = q_tar * q_orin.inv()

        # Convert the rotation error to Euler angles
        yaw, pitch, roll = q_error.as_euler(order, degrees=False)

        return yaw, pitch, roll

    def get_pose_yaw(self, pose, order='zyx'):
        q = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        yaw, pitch, roll = q.as_euler(order, degrees=False)
        return yaw

    def normalize_pi(self, alpha):
        while alpha > pi:
            alpha -= 2 * pi
        while alpha < -pi:
            alpha += 2 * pi
        return alpha

    def get_velocity(self, cur, goal, dT):
        #  a is the difference between the current yaw (cur_yaw) and the direction to the goal (goal_heading)
        # b is the error in heading, considering the goal's orientation (goal_yaw), adjusted by a.

        goal_heading = atan2(goal.position.y - cur.position.y, goal.position.x - cur.position.x)
        normalized_goal_heading = self.normalize_pi(goal_heading)
        # a = -self.cur_theta + goal_heading # goal_theta - cur_theta
        tar1 = self.add_yaw(cur, normalized_goal_heading, order='zyx')
        del_yaw, _, _ = self.compute_rotation_error(tar1, cur, order='zyx')
        print("del_yaw: ", del_yaw)
        a = self.normalize_pi(-del_yaw)
        print("a: ", a)

        # In Automomous Mobile Robots, they assume theta_G=0. So for
        # the error in heading, we have to adjust theta based on the
        # (possibly non-zero) goal theta.
        err_theta, _, _ = self.compute_rotation_error(goal, cur, order='zyx')
        theta = self.normalize_pi(err_theta)
        b = -theta - a
        print("b: ", b)


# Initialize the PoseModifier
modifier = PoseModifier()

# Create the target and original poses

orin_pose = Pose()
orin_pose.position.x = 1.0
orin_pose.position.y = -0.3
orin_pose.position.z = 0.22417
orin_pose.orientation.x = -0.70700
orin_pose.orientation.y = -0.00035
orin_pose.orientation.z = 0.70700
orin_pose.orientation.w = -0.00035
# Print the original and new poses
print(f"Original pose: {orin_pose}")

# Add yaw to the original pose
tar_pose = Pose()
tar_pose = modifier.add_yaw(orin_pose, 0.20)
tar_pose.position.x = 1.5
tar_pose.position.y = 0.0
tar_pose.position.z = 0.22417

#print
print(f"tras deg: {0.20/3.1415*180}")

# Print the original and new poses
print(f"Original pose: {orin_pose}")
print(f"New pose: {tar_pose}")

# Compute and print the rotation error
yaw_error, pitch_error, roll_error = modifier.compute_rotation_error(tar_pose, orin_pose)
print(f"Yaw error: {yaw_error}, Pitch error: {pitch_error}, Roll error: {roll_error}")
modifier.get_velocity(orin_pose, tar_pose, 0.1)
