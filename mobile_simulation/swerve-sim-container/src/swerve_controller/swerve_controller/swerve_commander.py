"""
Swerve commander for the simulated DexForce omni base.

This is a port of the real robot's omni controller
(mobile_joy_control/src/mobile_cmd_omni.cpp): same swerve kinematics, the same
+/-pi two-solution continuity selection, and the same speed ramping. The real
controller's hard "steer first, then drive" gate is replaced here by per-wheel
cos(steering_error) speed scaling so the base stays controllable under a
continuous planner (e.g. Nav2/MPPI) that keeps changing the commanded twist.
The other differences are that this node speaks the simulation's interfaces
instead of the real robot's:

  * Input : geometry_msgs/Twist on /cmd_vel  (real robot: /control/cmd_vel_omni)
  * Output: per-joint Float64MultiArray on the 8 ros2_control topics
            /{steering|wheel}_{front|rear}_{left|right}/commands
            (real robot: a single ChassisMotorControl message)
  * Feedback for the two-phase logic comes from /joint_states (the real robot
    uses /feedback/omni_chassis_motor).

Wheel angular velocity (rad/s) is published directly to the velocity
controllers, so no rpm conversion is needed.
"""

import math
import sys
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .swerve_joints import (
    JointEnd,
    JointKey,
    JointKind,
    JointSide,
    joint_name_from_key,
    joint_positions,
    joints,
)

# --- Tuning constants, mirrored from mobile_cmd_omni.cpp -------------------
CONTROL_PERIOD_S: float = 0.05  # 20 Hz, same as the real controller
CMD_TIMEOUT_S: float = 0.5  # zero the command if /cmd_vel goes stale
STEER_MAX_STEP: float = 6.28  # max steering change per cycle (rad)
SPEED_RAMP_RATE: float = 0.8  # wheel-speed low-pass factor (0-1)


def wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_diff(target: float, current: float) -> float:
    """Smallest signed difference target-current, wrapped to (-pi, pi]."""
    return wrap_pi(target - current)


def ramp_angle(target: float, current: float, max_step: float) -> float:
    """Step 'current' toward 'target' by at most 'max_step' (shortest way)."""
    d = angle_diff(target, current)
    if abs(d) <= max_step:
        return target
    return current + math.copysign(max_step, d)


class SwerveCommander(Node):
    """
    Converts a body twist on /cmd_vel into 8 swerve joint commands using the
    same algorithm as the real DexForce omni chassis controller.
    """

    wheelbase_descriptor = ParameterDescriptor(
        name="wheelbase",
        type=ParameterType.PARAMETER_DOUBLE,
        description="distance between front and rear wheels in meters",
    )
    wheel_track_descriptor = ParameterDescriptor(
        name="wheel_track",
        type=ParameterType.PARAMETER_DOUBLE,
        description="distance between left and right wheels in meters",
    )
    wheel_radius_descriptor = ParameterDescriptor(
        name="wheel_radius",
        type=ParameterType.PARAMETER_DOUBLE,
        description="radius of wheel in meters",
    )

    def __init__(self):
        super().__init__("swerve_commander")

        # declare and get geometry parameters
        self.wheelbase: float
        self.wheel_track: float
        self.wheel_radius: float
        for descriptor in (
            self.wheelbase_descriptor,
            self.wheel_track_descriptor,
            self.wheel_radius_descriptor,
        ):
            self.declare_parameter(name=descriptor.name, descriptor=descriptor)
            value = self.get_parameter(descriptor.name).get_parameter_value()
            setattr(self, descriptor.name, value.double_value)

        # one command publisher per swerve joint
        self.joint_publisher: Dict[JointKey, Publisher] = {}
        for key in joints():
            topic: str = f"/{joint_name_from_key(key)}/commands"
            self.joint_publisher[key] = self.create_publisher(
                Float64MultiArray, topic, 10
            )

        # latest commanded twist (cached, solved on a fixed-rate timer)
        self.latest_cmd: Twist = Twist()
        self.last_cmd_time = self.now()

        # feedback used as the continuity / two-phase base, mirroring the real
        # controller's feedback callback. Keyed by (end, side).
        self.actual_angle: Dict[Tuple[JointEnd, JointSide], float] = {
            pos: 0.0 for pos in joint_positions()
        }
        self.actual_wheel_vel: Dict[Tuple[JointEnd, JointSide], float] = {
            pos: 0.0 for pos in joint_positions()
        }

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_cb, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )
        self.timer = self.create_timer(CONTROL_PERIOD_S, self._on_timer)

    def now(self):
        return self.get_clock().now()

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.now()

    def _joint_state_cb(self, msg: JointState) -> None:
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        for end, side in joint_positions():
            steer = f"{joint_name_from_key((JointKind.STEERING, end, side))}_joint"
            wheel = f"{joint_name_from_key((JointKind.WHEEL, end, side))}_joint"
            if steer in name_to_index and name_to_index[steer] < len(msg.position):
                self.actual_angle[(end, side)] = msg.position[name_to_index[steer]]
            if wheel in name_to_index and name_to_index[wheel] < len(msg.velocity):
                self.actual_wheel_vel[(end, side)] = msg.velocity[
                    name_to_index[wheel]
                ]

    def _compute_wheels(
        self, vx: float, vy: float, wz: float
    ) -> Tuple[Dict, Dict]:
        """
        Returns (out_angle, out_wheel_angvel) dicts keyed by (end, side).

        Phase 0: per-wheel geometric target (angle + signed wheel speed) with
                 +/-pi two-solution selection that stays closest to the current
                 (measured) steering angle.
        Phase 1: per-wheel alignment scaling - each wheel drives proportionally
                 to cos(steering_error) so the base steers and translates at the
                 same time and stays controllable under a continuous planner.
        Phase 2: ramped output (steering rate-limited, wheel speed low-passed).
        """
        L = self.wheelbase
        W = self.wheel_track

        target_ang: Dict[Tuple[JointEnd, JointSide], float] = {}
        target_linear: Dict[Tuple[JointEnd, JointSide], float] = {}
        ang_err: Dict[Tuple[JointEnd, JointSide], float] = {}

        # Phase 0 - geometric targets
        for end, side in joint_positions():
            kx = 1.0 if side is JointSide.RIGHT else -1.0
            ky = 1.0 if end is JointEnd.FRONT else -1.0
            # x-velocity carries the lateral (track) term, y carries the
            # longitudinal (wheelbase) term: v_wheel = v_body + wz x r.
            vx_i = vx + kx * W * wz * 0.5
            vy_i = vy + ky * L * wz * 0.5

            speed = math.hypot(vx_i, vy_i)
            ang = math.atan2(vy_i, vx_i)

            prev = self.actual_angle[(end, side)]

            ang_a = wrap_pi(ang)
            spd_a = speed
            ang_b = wrap_pi(ang + math.pi)
            spd_b = -speed

            valid_a = abs(ang_a) <= math.pi / 2.0
            valid_b = abs(ang_b) <= math.pi / 2.0

            cost_a = abs(angle_diff(ang_a, prev)) if valid_a else 1e6
            cost_b = abs(angle_diff(ang_b, prev)) if valid_b else 1e6

            if cost_a <= cost_b:
                target_ang[(end, side)] = ang_a
                target_linear[(end, side)] = spd_a
            else:
                target_ang[(end, side)] = ang_b
                target_linear[(end, side)] = spd_b

            ang_err[(end, side)] = abs(angle_diff(target_ang[(end, side)], prev))

        # Phase 1+2 - ramped outputs with per-wheel alignment scaling.
        #
        # The real robot uses a hard global gate ("zero all wheel speeds until
        # every wheel is within ANGLE_EPS"), which is fine when the input twist
        # is smooth. Under a closed-loop planner like Nav2/MPPI the commanded
        # vx/vy/wz change every cycle, so the steering targets never all settle
        # at the same instant and the gate keeps the robot pinned in place
        # (symptom: "only turns, never translates"). Instead we drive each wheel
        # proportionally to how well it already points where it should:
        # cos(steering_error) is the projection of the desired wheel velocity
        # onto the wheel's current heading. Misaligned wheels contribute little,
        # aligned wheels contribute fully, so the base steers and translates
        # simultaneously and stays controllable under a continuous controller.
        out_angle: Dict[Tuple[JointEnd, JointSide], float] = {}
        out_wheel_angvel: Dict[Tuple[JointEnd, JointSide], float] = {}
        for end, side in joint_positions():
            prev = self.actual_angle[(end, side)]
            out_angle[(end, side)] = ramp_angle(
                target_ang[(end, side)], prev, STEER_MAX_STEP
            )

            align = math.cos(ang_err[(end, side)])
            if align < 0.0:
                align = 0.0
            target_angvel = target_linear[(end, side)] / self.wheel_radius * align
            prev_w = self.actual_wheel_vel[(end, side)]
            out_wheel_angvel[(end, side)] = (
                prev_w + (target_angvel - prev_w) * SPEED_RAMP_RATE
            )

        return out_angle, out_wheel_angvel

    def _on_timer(self) -> None:
        cmd = self.latest_cmd
        if (self.now() - self.last_cmd_time).nanoseconds * 1e-9 > CMD_TIMEOUT_S:
            cmd = Twist()

        out_angle, out_wheel_angvel = self._compute_wheels(
            cmd.linear.x, cmd.linear.y, cmd.angular.z
        )

        for end, side in joint_positions():
            for kind, value in (
                (JointKind.STEERING, out_angle[(end, side)]),
                (JointKind.WHEEL, out_wheel_angvel[(end, side)]),
            ):
                msg = Float64MultiArray()
                msg.data = [float(value)]
                self.joint_publisher[(kind, end, side)].publish(msg)


def main():
    rclpy.init(args=sys.argv)
    swerve_commander = SwerveCommander()
    rclpy.spin(swerve_commander)


if __name__ == "__main__":
    main()
