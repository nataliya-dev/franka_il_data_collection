import math
import time
from franky import *
import numpy as np
from scipy.spatial.transform import Rotation
import numpy as np

from dataclasses import dataclass


@dataclass
class RobotInputs:
    left_x: float
    left_y: float
    right_z: float
    roll: float
    pitch: float
    yaw: float
    gripper: float
    square_btn: bool
    triangle_btn: bool

    def __init__(self, array=None, square_btn=None, triangle_btn=None):
        if array is not None:
            self.left_x = array[0]
            self.left_y = array[1]
            self.right_z = array[2]
            self.roll = array[3]
            self.pitch = array[4]
            self.yaw = array[5]
            self.gripper = array[6]
            self.square_btn = square_btn
            self.triangle_btn = triangle_btn
        else:
            # Default values when no array is provided
            self.left_x = 0.0
            self.left_y = 0.0
            self.right_z = 0.0
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            self.gripper = 0.0
            self.square_btn = False
            self.triangle_btn = False


class FrankaRobot(object):
    def __init__(self, robot_ip="192.168.0.2"):
        self.robot = Robot(robot_ip)
        self.gripper = Gripper(robot_ip)
        self.gripper_speed = 0.05  # m/s
        self.min_gripper_pos_change = self.gripper.max_width / 10.0
        self.robot.recover_from_errors()
        # This controls the max percentage of jerk, acceleration, and velocity allowed in a motion.
        # A value of 0.05 means 5% of the maximum allowed.
        self.robot.relative_dynamics_factor = 0.05
        self.home = JointMotion([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

    def move_home(self):
        self.robot.move(self.home)
        # self.gripper.move(self.gripper.max_width, self.gripper_speed)
        self.gripper.move(0, self.gripper_speed)

    def rotate_to_zero(self):
        quat = Rotation.from_euler("xyz", [0, 0, 0]).as_quat()
        curr_pose = self.robot.current_pose
        self.robot.move(
            CartesianMotion(
                RobotPose(Affine(curr_pose.end_effector_pose.translation, quat))
            )
        )

    def move_velocity_inputs(self, robot_inputs: RobotInputs, duration_ms=500):
        linear_velocity = [robot_inputs.left_x,
                           robot_inputs.left_y, robot_inputs.right_z]
        angular_velocity = [robot_inputs.roll,
                            robot_inputs.pitch, robot_inputs.yaw]

        self.move_velocity_array(
            linear_velocity, angular_velocity, robot_inputs.gripper, duration_ms=duration_ms)

    def move_velocity_array(self, linear_velocity, angular_velocity, gripper, duration_ms=500):

        self.robot.move(
            CartesianVelocityMotion(
                Twist(
                    linear_velocity=linear_velocity,
                    angular_velocity=angular_velocity,
                ),
                duration=Duration(duration_ms),
            ),
            asynchronous=True,
        )

        if gripper > 0.5:
            self.gripper_grasp_open()
        if gripper < -0.5:
            self.gripper_grasp_close()

    def open_gripper(self):
        new_position = min(self.gripper.width +
                           self.min_gripper_pos_change, self.gripper.max_width)
        self.gripper.move_async(speed=self.gripper_speed, width=new_position)

    def close_gripper(self):
        new_position = max(self.gripper.width - self.min_gripper_pos_change, 0)
        self.gripper.move_async(speed=self.gripper_speed, width=new_position)

    def gripper_grasp_close(self, force=20):
        self.gripper.grasp(width=0.0, speed=self.gripper_speed, force=force)

    def gripper_grasp_open(self, force=20):
        self.gripper.open(self.gripper_speed)

    def quaternion_array_to_rpy(self, quaternion, degrees=False):
        """
        Convert array of quaternions to RPY angles.

        Args:
            quaternions: Array of shape (N, 4) with quaternions as [x, y, z, w]
            degrees: If True, return angles in degrees; if False, radians

        Returns:
            array: Shape (N, 3) with [roll, pitch, yaw] for each quaternion
        """
        rot = Rotation.from_quat(quaternion)
        return rot.as_euler('xyz', degrees=degrees)

    def get_ee_state(self):
        start = time.time()
        ee_pos_q = self.robot.current_cartesian_state.pose.end_effector_pose.quaternion
        print(f"ee_pos_q {time.time() - start}")
        start = time.time()
        ee_pos_rpy = self.quaternion_array_to_rpy(ee_pos_q)
        ee_pos_t = self.robot.current_cartesian_state.pose.end_effector_pose.translation
        print(f"ee_pos_t {time.time() - start}")
        gpos = self.gripper.width

        obs_data = {
            "ee_pos_t": ee_pos_t,
            "ee_pos_rpy": ee_pos_rpy,
            "gpos": np.array([gpos])
        }

        return obs_data
