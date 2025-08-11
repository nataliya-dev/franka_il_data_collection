import cv2
import numpy as np
import time
import pyrealsense2 as rs
import depthai as dai


# =============================================================================
# CAMERA CONFIGURATION - MODIFY THIS SECTION FOR YOUR SETUP
# =============================================================================

CAMERA_CONFIG = {
    "wrist": {
        "type": "realsense",
        "serial_number": "838212073725",
        "fps": 30,                  # optional; set True if you must force USB2
        "width": 960,
        "height": 540
    },
    "ext1": {
        "type": "luxonis",
        "mxid": "1844301041B4351300",  # <-- set your OAK MXID here
        "usb2": False,                  # optional; set True if you must force USB2
        "width": 960,
        "height": 540,
        "fps": 30,
    },
    "ext2": {
        "type": "luxonis",
        "mxid": "18443010E1D3381300",  # <-- set your OAK MXID here
        "usb2": False,                  # optional; set True if you must force USB2
        "width": 960,
        "height": 540,
        "fps": 30,
    },
}


# =============================================================================
# CAMERA CLASS
# =============================================================================
class Cameras:
    def __init__(self, camera_config=None):
        if camera_config is None:
            camera_config = CAMERA_CONFIG

        self.cameras = {}
        self.camera_config = camera_config
        for name, config in camera_config.items():
            if config["type"] == "realsense":
                self.cameras[name] = self._init_realsense(config)
            elif config["type"] == "webcam":
                self.cameras[name] = self._init_webcam(config)
            elif config["type"] == "luxonis":
                self.cameras[name] = self._init_luxonis(config)
            else:
                raise ValueError(
                    f"Unknown camera type: {config['type']} for camera {name}")

    def _init_realsense(self, config):
        ctx = rs.context()
        devices = ctx.query_devices()
        available_serials = [device.get_info(
            rs.camera_info.serial_number) for device in devices]

        if config["serial_number"] not in available_serials:
            raise Exception(
                f"RealSense camera {config['serial_number']} not found")

        pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_device(config["serial_number"])
        rs_config.enable_stream(
            rs.stream.color,
            config["width"],
            config["height"],
            rs.format.bgr8,
            config["fps"]
        )
        profile = pipeline.start(rs_config)
        return {"type": "realsense", "pipeline": pipeline, "profile": profile, "config": config}

    def _init_webcam(self, config):
        capture = cv2.VideoCapture(config["index"])
        if not capture.isOpened():
            raise Exception(
                f"Could not open webcam at index {config['index']}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])
        capture.set(cv2.CAP_PROP_FPS, config["fps"])
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        ret, _ = capture.read()
        if not ret:
            capture.release()
            raise Exception(
                f"Could not read from webcam at index {config['index']}")
        return {"type": "webcam", "capture": capture, "config": config}

    def _init_luxonis(self, config):
        print("Initializing first luxonis camera")
        mxid = config.get("mxid") or config.get("serial_number")
        if not mxid:
            raise ValueError(
                "Luxonis config must include 'mxid' (MXID printed on the device or via depthai tools).")

        # Build a simple RGB pipeline
        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgb")

        # Resolution/FPS
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        cam.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setFps(config["fps"])
        # Use preview sized close to your desired output; you can also use cam.video
        cam.setPreviewSize(
            config["width"], config["height"])
        cam.preview.link(xout.input)

        # Connect to the specified device by MXID
        dev_info = dai.DeviceInfo(mxid)
        usb2_mode = bool(config.get("usb2", False))
        device = dai.Device(pipeline, dev_info, usb2Mode=usb2_mode)

        # Prepare queue
        q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        return {"type": "luxonis", "device": device, "queue": q, "config": config}

    def get_frames(self):
        frames = {}
        for name, camera in self.cameras.items():
            if camera["type"] == "realsense":
                pipeline_frames = camera["pipeline"].wait_for_frames()
                color_frame = pipeline_frames.get_color_frame()
                if not color_frame:
                    raise Exception(f"No color frame from camera {name}")
                color_image = np.asanyarray(color_frame.get_data())
                frames[name] = cv2.resize(
                    color_image, (camera["config"]["width"], camera["config"]["height"]))

            elif camera["type"] == "webcam":
                ret, frame = camera["capture"].read()
                if not ret:
                    raise Exception(f"Could not read frame from camera {name}")
                frames[name] = cv2.resize(
                    frame, (camera["config"]["width"], camera["config"]["height"]))

            elif camera["type"] == "luxonis":
                inRgb = camera["queue"].tryGet()
                if inRgb is None:
                    raise Exception(f"No frame from Luxonis camera {name}")
                # no need to resize because set preview size does this for us
                frames[name] = inRgb.getCvFrame()

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
                depth_frames[name] = cv2.resize(
                    depth_image, (camera["config"]["width"], camera["config"]["height"]))
            elif camera["type"] == "webcam":
                depth_frames[name] = None
            elif camera["type"] == "luxonis":
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
                    "color": {"fx": color_intrinsics.fx, "fy": color_intrinsics.fy, "cx": color_intrinsics.ppx, "cy": color_intrinsics.ppy, "distortion": color_intrinsics.coeffs},
                    "depth": {"fx": depth_intrinsics.fx, "fy": depth_intrinsics.fy, "cx": depth_intrinsics.ppx, "cy": depth_intrinsics.ppy, "distortion": depth_intrinsics.coeffs}
                }
            elif camera["type"] in ("webcam", "luxonis"):
                # TODO(nn) add instrinsics here
                intrinsics[name] = None
        return intrinsics

    def get_extrinsics(self):
        extrinsics = {}
        for name in self.cameras.keys():
            extrinsics[name] = None
        return extrinsics

    def close(self):
        for camera in self.cameras.values():
            if camera["type"] == "realsense":
                camera["pipeline"].stop()
            elif camera["type"] == "webcam":
                camera["capture"].release()
            elif camera["type"] == "luxonis":
                camera["device"].close()

    def __del__(self):
        self.close()
