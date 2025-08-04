import cv2
import numpy as np

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

# =============================================================================
# CAMERA CONFIGURATION - MODIFY THIS SECTION FOR YOUR SETUP
# =============================================================================

CAMERA_CONFIG = {
    "wrist": {
        "type": "realsense",
        "serial_number": "838212073725"
    },
    "ext1": {
        "type": "realsense",
        "serial_number": "943222071556"
    },
    "ext2": {
        "type": "realsense",
        "serial_number": "913522070103"
    },
}

# Camera settings
REALSENSE_SETTINGS = {"width": 640, "height": 480, "fps": 30}
WEBCAM_SETTINGS = {"width": 640, "height": 480, "fps": 30}
OUTPUT_SIZE = (224, 224)

# =============================================================================
# CAMERA CLASS
# =============================================================================


class Cameras:
    def __init__(self, camera_config=None):
        if camera_config is None:
            camera_config = CAMERA_CONFIG

        self.cameras = {}

        for name, config in camera_config.items():
            if config["type"] == "realsense":
                self.cameras[name] = self._init_realsense(config)
            elif config["type"] == "webcam":
                self.cameras[name] = self._init_webcam(config)
            else:
                raise ValueError(
                    f"Unknown camera type: {config['type']} for camera {name}")

    def _init_realsense(self, config):
        if not REALSENSE_AVAILABLE:
            raise Exception("pyrealsense2 not available")

        # Check if camera exists
        ctx = rs.context()
        devices = ctx.query_devices()
        available_serials = [device.get_info(
            rs.camera_info.serial_number) for device in devices]

        if config["serial_number"] not in available_serials:
            raise Exception(
                f"RealSense camera {config['serial_number']} not found")

        # Initialize pipeline
        pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_device(config["serial_number"])
        rs_config.enable_stream(
            rs.stream.color,
            REALSENSE_SETTINGS["width"],
            REALSENSE_SETTINGS["height"],
            rs.format.bgr8,
            REALSENSE_SETTINGS["fps"]
        )
        # rs_config.enable_stream(
        #     rs.stream.depth,
        #     REALSENSE_SETTINGS["width"],
        #     REALSENSE_SETTINGS["height"],
        #     rs.format.z16,
        #     REALSENSE_SETTINGS["fps"]
        # )

        profile = pipeline.start(rs_config)
        return {"type": "realsense", "pipeline": pipeline, "profile": profile}

    def _init_webcam(self, config):
        capture = cv2.VideoCapture(config["index"])

        if not capture.isOpened():
            raise Exception(
                f"Could not open webcam at index {config['index']}")

        # Set properties
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, WEBCAM_SETTINGS["width"])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, WEBCAM_SETTINGS["height"])
        capture.set(cv2.CAP_PROP_FPS, WEBCAM_SETTINGS["fps"])
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # disabled

        # Test capture
        ret, _ = capture.read()
        if not ret:
            capture.release()
            raise Exception(
                f"Could not read from webcam at index {config['index']}")

        return {"type": "webcam", "capture": capture}

    def get_frames(self):
        frames = {}
        for name, camera in self.cameras.items():
            if camera["type"] == "realsense":
                pipeline_frames = camera["pipeline"].wait_for_frames()
                color_frame = pipeline_frames.get_color_frame()
                if not color_frame:
                    raise Exception(f"No color frame from camera {name}")
                color_image = np.asanyarray(color_frame.get_data())
                frames[name] = cv2.resize(color_image, OUTPUT_SIZE)

            elif camera["type"] == "webcam":
                ret, frame = camera["capture"].read()
                if not ret:
                    raise Exception(f"Could not read frame from camera {name}")
                frames[name] = cv2.resize(frame, OUTPUT_SIZE)

        return frames

    def get_depth_frames(self):
        depth_frames = {}
        for name, camera in self.cameras.items():
            if camera["type"] == "realsense":
                pipeline_frames = camera["pipeline"].wait_for_frames()
                depth_frame = pipeline_frames.get_depth_frame()
                if not depth_frame:
                    raise Exception(f"No depth frame from camera {name}")
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_frames[name] = cv2.resize(depth_image, OUTPUT_SIZE)
            elif camera["type"] == "webcam":
                depth_frames[name] = None

        return depth_frames

    def get_intrinsics(self):
        intrinsics = {}
        for name, camera in self.cameras.items():
            if camera["type"] == "realsense":
                profile = camera["profile"]
                color_stream = profile.get_stream(rs.stream.color)
                color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
                depth_stream = profile.get_stream(rs.stream.depth)
                depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()

                intrinsics[name] = {
                    "color": {
                        "fx": color_intrinsics.fx,
                        "fy": color_intrinsics.fy,
                        "cx": color_intrinsics.ppx,
                        "cy": color_intrinsics.ppy,
                        "distortion": color_intrinsics.coeffs
                    },
                    "depth": {
                        "fx": depth_intrinsics.fx,
                        "fy": depth_intrinsics.fy,
                        "cx": depth_intrinsics.ppx,
                        "cy": depth_intrinsics.ppy,
                        "distortion": depth_intrinsics.coeffs
                    }
                }
            elif camera["type"] == "webcam":
                # dummy functions, will need json file
                intrinsics[name] = None

        return intrinsics

    def get_extrinsics(self):
        # dummy function for now, will need json file
        extrinsics = {}

        for name, camera in self.cameras.items():
            extrinsics[name] = None

        return extrinsics

    def close(self):
        for camera in self.cameras.values():
            if camera["type"] == "realsense":
                camera["pipeline"].stop()
            elif camera["type"] == "webcam":
                camera["capture"].release()

    def __del__(self):
        self.close()
