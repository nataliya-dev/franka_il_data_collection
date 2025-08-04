
import threading
from inputs import get_gamepad
# our imports
from franka_robot import RobotInputs


def truncate_float(x, decimal_places=2):
    factor = 10**decimal_places
    return int(x * factor) / factor


class GameController(object):
    # These values are different for different controllers.
    MAX_TRIG_VAL = 128.0
    MAX_JOY_VAL = 128.0

    def __init__(self):
        self.LeftJoystickY = 0.0
        self.LeftJoystickX = 0.0
        self.RightJoystickY = 0.0
        self.RightJoystickX = 0.0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.Square = 0
        self.Triangle = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.HorizontalDPad = 0
        self.VerticalDPad = 0

        self._monitor_thread = threading.Thread(
            target=self._monitor_controller, args=()
        )
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def get_robot_inputs_from_controller(self):
        position_multiplier = 10.0 / 100.0
        angle_multiplier = 2.0
        left_x = truncate_float(self.LeftJoystickY) * position_multiplier
        left_y = truncate_float(self.LeftJoystickX) * position_multiplier
        right_z = -truncate_float(self.RightJoystickY) * position_multiplier

        roll = self.VerticalDPad
        pitch = self.HorizontalDPad
        pitch *= angle_multiplier
        roll *= angle_multiplier
        yaw_neg = self.LeftBumper * angle_multiplier
        yaw_pos = self.RightBumper * angle_multiplier
        yaw = yaw_pos - yaw_neg

        gripper = 1 if self.RightTrigger > 0.5 else -1 if self.LeftTrigger > 0.5 else 0

        square_btn = self.Square == 1
        triangle_btn = self.Triangle == 1
        # print(f"Left: {left_x:.2f}, {left_y:.2f}, Right Z: {right_z:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Gripper: {gripper:.2f}, X: {square_btn}, Y: {triangle_btn}")
        return RobotInputs([left_x, left_y, right_z, roll, pitch, yaw, gripper], square_btn, triangle_btn)

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == "ABS_Y":
                    self.LeftJoystickY = (
                        event.state - GameController.MAX_JOY_VAL
                    ) / GameController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == "ABS_X":
                    self.LeftJoystickX = (
                        event.state - GameController.MAX_JOY_VAL
                    ) / GameController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == "ABS_RY":
                    self.RightJoystickY = (
                        event.state - GameController.MAX_JOY_VAL
                    ) / GameController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == "ABS_RX":
                    self.RightJoystickX = (
                        event.state - GameController.MAX_JOY_VAL
                    ) / GameController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == "ABS_Z":
                    self.LeftTrigger = (
                        event.state / GameController.MAX_TRIG_VAL
                    )  # normalize between 0 and 1
                elif event.code == "ABS_RZ":
                    self.RightTrigger = (
                        event.state / GameController.MAX_TRIG_VAL
                    )  # normalize between 0 and 1
                elif event.code == "BTN_TL":
                    self.LeftBumper = event.state
                elif event.code == "BTN_TR":
                    self.RightBumper = event.state
                elif event.code == "BTN_SOUTH":
                    self.A = event.state
                elif event.code == "BTN_NORTH":
                    self.Triangle = event.state  # On some controllers, this is switched with BTN_WEST
                elif event.code == "BTN_WEST":
                    self.Square = event.state  # On some controllers, this is switched with BTN_NORTH
                elif event.code == "BTN_EAST":
                    self.B = event.state
                elif event.code == "BTN_THUMBL":
                    self.LeftThumb = event.state
                elif event.code == "BTN_THUMBR":
                    self.RightThumb = event.state
                elif event.code == "BTN_SELECT":
                    self.Back = event.state
                elif event.code == "BTN_START":
                    self.Start = event.state
                elif event.code == "ABS_HAT0X":
                    self.HorizontalDPad = event.state
                elif event.code == "ABS_HAT0Y":
                    self.VerticalDPad = event.state
