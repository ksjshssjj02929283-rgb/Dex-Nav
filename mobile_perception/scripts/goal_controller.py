from __future__ import division, print_function
from math import pi, sqrt, sin, cos, atan2
# from diff_drive.pose import Pose
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import math
import numpy as np
import math
#import rospy


class GoalController:
    """Finds linear and angular velocities necessary to drive toward
    a goal pose.
    """

    def __init__(self):
        self.kA = 0.1050 * 1.05  #1.18
        self.kP = 0.0800 * 1.05  #0.2
        self.kB = -0.0040 * 1.05  #2.0 #-0.15
        self.max_linear_speed = 0.50  #0.2
        self.min_linear_speed = 0
        self.max_angular_speed = 0.500  #0.2
        self.min_angular_speed = -0.500  #0.2
        self.max_linear_acceleration = 2.5
        self.max_angular_acceleration = 2.5
        self.linear_tolerance = 0.03  # 8mm
        self.angular_tolerance = 0.035  #0.035 # 2degrees
        self.forward_movement_only = False
        self.cur_theta = 0
        self.goal_theta = 0
        self.xVel = 0
        self.yVel = 0
        self.thetaVel = 0
        self.res_ = False

    def set_constants(self, kP, kA, kB):
        self.kP = kP
        self.kA = kA
        self.kB = kB

    def set_max_linear_speed(self, speed):
        self.max_linear_speed = speed

    def set_min_linear_speed(self, speed):
        self.min_linear_speed = speed

    def set_max_angular_speed(self, speed):
        self.max_angular_speed = speed

    def set_min_angular_speed(self, speed):
        self.min_angular_speed = speed

    def set_max_linear_acceleration(self, accel):
        self.max_linear_acceleration = accel

    def set_max_angular_acceleration(self, accel):
        self.max_angular_acceleration = accel

    def set_linear_tolerance(self, tolerance):
        self.linear_tolerance = tolerance

    def set_angular_tolerance(self, tolerance):
        self.angular_tolerance = tolerance

    def set_forward_movement_only(self, forward_only):
        self.forward_movement_only = forward_only

    def get_goal_distance(self, cur, goal):
        if goal is None:
            return 0
        diffX = cur.position.x - goal.position.x
        diffY = cur.position.y - goal.position.y
        # log distance
        # print("diffX, diffX", diffX, diffY)
        return sqrt(diffX * diffX + diffY * diffY)

    def at_goal(self, cur, goal):
        if goal is None:
            return True
        d = self.get_goal_distance(cur, goal)
        err_theta, _, _ = self.compute_rotation_error(goal, cur, order='zyx')
        dy = abs(cur.position.y - goal.position.y)
        dTh = abs(self.normalize_pi(err_theta))
        self.res_ = (d < self.linear_tolerance and
                     dTh < self.angular_tolerance and dy < 0.01)
        if self.res_:
            print("dist_err = ", d, "theta_yaw_err = ", dTh)
        return self.res_

    def add_yaw(self, pose, yaw, order='zyx'):
        # Create a rotation representing the yaw
        q_yaw = R.from_euler(order, [yaw, 0, 0], degrees=False)

        # Create a rotation from the pose's orientation quaternion
        q1 = R.from_quat([
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w
        ])

        # Apply the yaw rotation to the pose's orientation
        q_new = q_yaw * q1

        # Normalize the resulting quaternion
        q_new = R.from_quat(q_new.as_quat() / np.linalg.norm(q_new.as_quat()))

        # Update the pose's orientation
        tar_pose = Pose()
        tar_pose.position = pose.position
        tar_pose.orientation.x = q_new.as_quat()[0]
        tar_pose.orientation.y = q_new.as_quat()[1]
        tar_pose.orientation.z = q_new.as_quat()[2]
        tar_pose.orientation.w = q_new.as_quat()[3]

        return tar_pose

    def compute_rotation_error(self, tar_pose, orin_pose, order='zyx'):
        # Create rotations from the poses' orientation quaternions
        q_tar = R.from_quat([
            tar_pose.orientation.x, tar_pose.orientation.y,
            tar_pose.orientation.z, tar_pose.orientation.w
        ])
        q_orin = R.from_quat([
            orin_pose.orientation.x, orin_pose.orientation.y,
            orin_pose.orientation.z, orin_pose.orientation.w
        ])

        # Compute the rotation error
        q_error = q_tar * q_orin.inv()

        # Convert the rotation error to Euler angles
        yaw, pitch, roll = q_error.as_euler(order, degrees=False)

        return yaw, pitch, roll

    def get_pose_yaw(self, pose, order='zyx'):
        q = R.from_quat([
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w
        ])
        yaw, pitch, roll = q.as_euler(order, degrees=False)
        return yaw

    def get_velocity(self, cur, goal, dT, direction=1):
        #  a is the difference between the current yaw (cur_yaw) and the direction to the goal (goal_heading)
        # b is the error in heading, considering the goal's orientation (goal_yaw), adjusted by a.
        goal_heading = atan2(cur.position.y - goal.position.y,
                             cur.position.x - goal.position.x)

        normalized_goal_heading = self.normalize_pi(goal_heading)
        tar1 = self.add_yaw(cur, normalized_goal_heading, order='zyx')
        del_yaw, _, _ = self.compute_rotation_error(tar1, cur, order='zyx')
        yerr_set0 = math.fabs(cur.position.y - goal.position.y)
        # print("yerr_set0: ", yerr_set0)
        if (yerr_set0 < self.linear_tolerance):
            del_yaw = 0.00001
            # print("delta y for yaw: ", cur.position.y-goal.position.y, "delta x for yaw: ", cur.position.x-goal.position.x)
        a = self.normalize_pi(del_yaw)

        err_theta, _, _ = self.compute_rotation_error(goal, cur, order='zyx')
        theta = self.normalize_pi(err_theta)
        b = self.normalize_pi(-theta - a)

        d = self.get_goal_distance(cur, goal)

        # if self.forward_movement_only:
        if direction == 1:
            a = self.normalize_pi(a)
            b = self.normalize_pi(b)
        else:
            #     # direction = self.sign(cos(a))
            #     direction = self.sign(cur.position.x-goal.position.x)
            a = self.normalize_half_pi(a)
            b = self.normalize_half_pi(b)
        # print("direction: ", direction )

        if abs(d) < self.linear_tolerance:
            self.xVel = 0.0
            self.thetaVel = self.kB * theta
            # print("d< tolarence.............", d)
            # print("err_theta: ", err_theta)
            # print("thetaVel: ", self.thetaVel)
            # print("error_x: ", goal.position.x - cur.position.x)
            # print("error_y: ",goal.position.y - cur.position.y)
            # print("error_yaw: ",self.compute_rotation_error( goal, cur, order='zyx')[0])

        else:
            self.xVel = self.kP * d * direction
            self.thetaVel = self.kA * a + self.kB * b
            if direction == 1:
                self.thetaVel = -self.thetaVel

            # print("err_theta: ", theta)
            # print("error_x: ", goal.position.x - cur.position.x)
            # print("error_y: ",goal.position.y - cur.position.y)
            # print("b: ", b, "atan2 yaw a: ", a)
            # print("thetaVel: ", self.thetaVel)

        # Adjust velocities if X velocity is too high.
        if abs(self.xVel) > self.max_linear_speed:
            ratio = self.max_linear_speed / abs(self.xVel)
            self.xVel *= ratio
            self.thetaVel *= ratio

        # Adjust velocities if turning velocity too high.
        if abs(self.thetaVel) > self.max_angular_speed:
            ratio = self.max_angular_speed / abs(self.thetaVel)
            self.xVel *= ratio
            self.thetaVel *= ratio

        # TBD: Adjust velocities if linear or angular acceleration
        # too high.

        # Adjust velocities if too low, so robot does not stall.
        if abs(self.xVel) > 0 and abs(self.xVel) < self.min_linear_speed:
            ratio = self.min_linear_speed / abs(self.xVel)
            self.xVel *= ratio
            self.thetaVel *= max(ratio * 0.75, 1.0)
        elif self.xVel == 0 and abs(self.thetaVel) < self.min_angular_speed:
            ratio = self.min_angular_speed / abs(self.thetaVel)
            self.xVel *= ratio
            self.thetaVel *= ratio
        if abs(self.thetaVel) < self.min_angular_speed:
            print(f' omega: {self.thetaVel:0.4f}')
            ratio = self.min_angular_speed / abs(self.thetaVel)
            # self.xVel *= ratio
            if abs(self.thetaVel) > 0.002:
                self.thetaVel *= ratio
        return self.xVel, self.thetaVel

    def normalize_half_pi(self, alpha):
        alpha = self.normalize_pi(alpha)
        if alpha > pi / 2:
            return alpha - pi
        elif alpha < -pi / 2:
            return alpha + pi
        else:
            return alpha

    def normalize_pi(self, alpha):
        while alpha > pi:
            alpha -= 2 * pi
        while alpha < -pi:
            alpha += 2 * pi
        return alpha

    def sign(self, x):
        if x >= 0:
            return 1
        else:
            return -1
