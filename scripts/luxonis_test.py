#!/usr/bin/env python3

import argparse
from typing import List
import depthai as dai
import cv2
import sys


def list_devices(infos: List[dai.DeviceInfo]) -> None:
    if not infos:
        print("No devices found.")
        return
    for i, info in enumerate(infos):
        state = str(info.state).split('X_LINK_')[-1]
        print(f"[{i}] name={info.name} mxid={info.mxid} state={state}")


def pick_device(infos: List[dai.DeviceInfo], mxid: str = None, index: int = None) -> dai.DeviceInfo:
    if not infos:
        print("No devices found.")
        sys.exit(1)
    if mxid:
        for info in infos:
            if info.mxid == mxid:
                return info
        print(f"mxid '{mxid}' not found.")
        sys.exit(1)
    if index is not None:
        if 0 <= index < len(infos):
            return infos[index]
        print(f"index {index} out of range [0, {len(infos)-1}].")
        sys.exit(1)
    return infos[0]


def build_pipeline() -> dai.Pipeline:
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    xout = p.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setPreviewSize(960, 540)

    cam.preview.link(xout.input)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--mxid", type=str, default=None)
    ap.add_argument("--index", type=int, default=None)
    ap.add_argument("--usb2", action="store_true")
    args = ap.parse_args()

    infos: List[dai.DeviceInfo] = dai.DeviceBootloader.getAllAvailableDevices()
    if args.list:
        list_devices(infos)
        sys.exit(0)

    info = pick_device(infos, mxid=args.mxid, index=args.index)
    pipeline = build_pipeline()

    print(f"Connecting to device: name={info.name} mxid={info.mxid}")
    with dai.Device(pipeline, info, usb2Mode=args.usb2) as device:
        print('Connected cameras:', device.getConnectedCameraFeatures())
        print('Usb speed:', device.getUsbSpeed().name)
        if device.getBootloaderVersion() is not None:
            print('Bootloader version:', device.getBootloaderVersion())
        print('Device name:', device.getDeviceName(),
              'Product name:', device.getProductName())

        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        while True:
            inRgb = qRgb.get()
            frame = inRgb.getCvFrame()
            print(frame.shape)
            cv2.imshow("rgb", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


if __name__ == "__main__":
    main()
