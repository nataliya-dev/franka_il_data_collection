import pyrealsense2 as rs
import numpy as np
import cv2

# Depth contrast multiplier - increase this to make close objects more visible
DEPTH_CONTRAST = 1.3

# Configure depth and color streams
ctx = rs.context()
devices = ctx.query_devices()
# wrist, closest camera (r1), farthest camera (r2)
# device_serials = ['838212073725', '943222071556', '913522070103']
for device in devices:
    print(device)
device_serials = ['838212073725', '943222071556']


def start_camera(serial):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    # config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

    pipeline.start(config)
    return pipeline


pipelines = [start_camera(n) for n in device_serials]


def display_frame(pipeline, serial):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # ir_left = frames.get_infrared_frame(1)   # Left IR camera
    # ir_right = frames.get_infrared_frame(2)  # Right IR camera

    # Convert to numpy arrays if needed
    # ir_left_image = np.asanyarray(ir_left.get_data())
    # ir_right_image = np.asanyarray(ir_right.get_data())

    if not color_frame or not depth_frame:
        print("error")
        return

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # print(f"image size {color_image.shape}")

    # Resize color image
    # resized_color = cv2.resize(color_image, (224, 224))
    resized_color = color_image

    # Process depth image for display
    # Apply contrast enhancement and convert to 8-bit for display
    depth_normalized = cv2.normalize(
        depth_image, None, 0, 255, cv2.NORM_MINMAX)
    depth_enhanced = np.clip(
        depth_normalized * DEPTH_CONTRAST, 0, 255).astype(np.uint8)
    depth_colormap = cv2.applyColorMap(depth_enhanced, cv2.COLORMAP_JET)
    # resized_depth = cv2.resize(depth_colormap, (224, 224))
    resized_depth = depth_colormap

    # print(f"resized size {resized_color.shape}")

    # Show images
    cv2.imshow(f'RealSense Color: {serial}', resized_color)
    cv2.imshow(f'RealSense Depth: {serial}', resized_depth)
    # cv2.imshow(f'RealSense IR left: {serial}', ir_left_image)
    # cv2.imshow(f'RealSense IR right: {serial}', ir_right_image)

    # cv2.imwrite(f'ir_left_{serial}.png', ir_left_image)
    # cv2.imwrite(f'ir_right_{serial}.png', ir_right_image)


try:
    while True:
        for serial, pipeline in zip(device_serials, pipelines):
            display_frame(pipeline, serial)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    for serial, pipeline in zip(device_serials, pipelines):
        pipeline.stop()
        cv2.destroyAllWindows()
