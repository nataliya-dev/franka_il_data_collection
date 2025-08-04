import time
import numpy as np

# our imports
from franka_robot import FrankaRobot, RobotInputs
from game_controller import GameController
from cameras import Cameras
from data_recorder import DataRecorder


if __name__ == "__main__":
    joy = GameController()
    franka_robot = FrankaRobot()
    franka_robot.move_home()
    cameras = Cameras()
    recorder = DataRecorder(franka_robot, cameras)
    is_recording = False
    print("Beginning control loop")
    previous = time.time()
    while True:
        t0 = time.time()
        robot_inputs = joy.get_robot_inputs_from_controller()
        if robot_inputs.square_btn:
            time.sleep(1)
            print("Moving to home")
            franka_robot.move_home()
            recorder.reset()
            is_recording = True
            print("Recording started")
        elif robot_inputs.triangle_btn:
            print("Saving recording")
            recorder.save_data()
            is_recording = False
            time.sleep(1)
            print("Recording stopped")
            print("Moving to home")
            franka_robot.move_home()
        elif is_recording:
            # This is how long the previous action was applied.
            action_tm = time.time() - previous
            previous = time.time()
            recorder.record_sample(robot_inputs, action_tm)
        try:
            franka_robot.move_velocity_inputs(robot_inputs)
        except:  # Exception as e:
            # print(f"Exception occured: {e}")
            print("Exception occurred while moving robot.")
            franka_robot.robot.recover_from_errors()

        dt = time.time()-t0
        diff_to_target = 1/10.0 - dt
        diff_to_target = np.clip(diff_to_target, 0.0, 100.0)
        # print(f"{diff_to_target=}")
        time.sleep(float(diff_to_target))
