import math
import time
from franky import *
import numpy as np
from scipy.spatial.transform import Rotation
import numpy as np

from dataclasses import dataclass
import threading


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
        self.homeRef = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.home = JointMotion(self.homeRef)

       # Keep track of gripper state to avoid redundant commands
        self.last_gripper_command = None
        self.last_gripper_time = 0
        self.gripper_cooldown = 0.1

    def positionsCloseEnough(self):
        isCloseEnough = True
        for homePos, jointPos in zip(self.homeRef, self.robot.current_joint_positions):
            if abs(homePos - jointPos) > 0.01:
                isCloseEnough = False

        return isCloseEnough

    def move_home(self):
        self.robot.move(self.home)
        # if (self.positionsCloseEnough()):
        #     print("Already at home...")
        #     return
        # self.gripper.move(self.gripper.max_width, self.gripper_speed)
        print("Not at home, moving...")
        self.gripper_grasp_open()

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

        # start = time.time()

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
        # print(f"move tm {time.time()-start}")

        start = time.time()

        # print(f"gripper val {gripper}")

        # if gripper > 0.5:
        #     self.open_gripper()
        # if gripper < -0.5:
        #     self.close_gripper()

     # Handle gripper with throttling to prevent blocking
        current_time = time.time()
        if current_time - self.last_gripper_time > self.gripper_cooldown:
            if gripper > 0.5 and self.last_gripper_command != "open":
                self.open_gripper_threaded()
                self.last_gripper_command = "open"
                self.last_gripper_time = current_time
            elif gripper < -0.5 and self.last_gripper_command != "close":
                self.close_gripper_threaded()
                self.last_gripper_command = "close"
                self.last_gripper_time = current_time

        # print(f"gripper tm {time.time()-start}")

    def _gripper_worker(self, action, **kwargs):
        """Worker function to run gripper operations in a separate thread"""
        try:
            if action == "open":
                # Use the truly async version without waiting
                self.gripper.grasp_async(speed=self.gripper_speed,
                                         width=0.2, force=kwargs.get('force', 20))
            elif action == "close":
                # Use the truly async version without waiting
                self.gripper.grasp_async(speed=self.gripper_speed,
                                         width=0, force=kwargs.get('force', 20))
            elif action == "grasp_close":
                # This might be blocking, so we run it in the thread
                self.gripper.grasp(width=0.0, speed=self.gripper_speed,
                                   force=kwargs.get('force', 20))
            elif action == "grasp_open":
                # This might be blocking, so we run it in the thread
                self.gripper.open(self.gripper_speed)
        except Exception as e:
            print(f"Gripper operation failed: {e}")

    def _start_gripper_thread(self, action, **kwargs):
        """Start a new gripper thread, cancelling any existing one if needed"""
        with self.gripper_thread_lock:
            # Optional: Wait for previous thread to complete or interrupt it
            if self.current_gripper_thread and self.current_gripper_thread.is_alive():
                # You might want to implement a way to cancel the previous operation
                # For now, we'll just let it finish naturally
                pass

            # Start new thread
            self.current_gripper_thread = threading.Thread(
                target=self._gripper_worker,
                args=(action,),
                kwargs=kwargs,
                daemon=True  # Thread will not prevent program exit
            )
            self.current_gripper_thread.start()

    def open_gripper_threaded(self, force=20):
        """Truly non-blocking version using threading"""
        def _open_gripper():
            try:
                # Try the async version first
                self.gripper.grasp_async(speed=self.gripper_speed,
                                         width=0.2, force=force)
            except Exception as e:
                print(f"Open gripper async failed, trying sync: {e}")
                try:
                    # Fallback to synchronous version in thread
                    self.gripper.move(0.2, self.gripper_speed)
                except Exception as e2:
                    print(f"Open gripper sync also failed: {e2}")

        # Always use threading to ensure non-blocking behavior
        thread = threading.Thread(target=_open_gripper, daemon=True)
        thread.start()

    def close_gripper_threaded(self, force=20):
        """Truly non-blocking version using threading"""
        def _close_gripper():
            try:
                # Try the async version first
                self.gripper.grasp_async(speed=self.gripper_speed,
                                         width=0, force=force)
            except Exception as e:
                print(f"Close gripper async failed, trying sync: {e}")
                try:
                    # Fallback to synchronous version in thread
                    self.gripper.move(0, self.gripper_speed)
                except Exception as e2:
                    print(f"Close gripper sync also failed: {e2}")

        # Always use threading to ensure non-blocking behavior
        thread = threading.Thread(target=_close_gripper, daemon=True)
        thread.start()

    def open_gripper(self):
        # new_position = min(self.gripper.width +
        #                    self.min_gripper_pos_change, self.gripper.max_width)
        self.gripper.grasp_async(speed=self.gripper_speed,
                                 width=0.2, force=20)

    def close_gripper(self):
        # new_position = max(self.gripper.width - self.min_gripper_pos_change, 0)
        self.gripper.grasp_async(speed=self.gripper_speed, width=0, force=20)

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
