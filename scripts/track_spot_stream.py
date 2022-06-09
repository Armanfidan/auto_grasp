# Copyright (c) 2021 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Simple image capture tutorial."""

import argparse
import sys
import time

from bosdyn.api import image_pb2
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient
from bosdyn.api import image_pb2
import cv2
import imutils
from imutils.video import FPS
import numpy as np


source = 'frontright_fisheye_image'


def main(argv):
    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('image_capture')
    robot = sdk.create_robot('192.168.50.3')
    robot.authenticate('user', 'oxyw9las1i8b')
    robot.sync_with_directory()
    robot.time_sync.wait_for_sync()

    initBB = None
    fps = None

    image_client = robot.ensure_client(ImageClient.default_service_name)

    tracker = cv2.TrackerCSRT_create()

    while True:

        # Capture and save images to disk
        image = image_client.get_image_from_sources([source])[0]
        frame = np.frombuffer(image.shot.image.data, dtype=np.uint8)
        frame = cv2.imdecode(frame, -1)
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        

        if initBB:
            success, box = tracker.update(frame)

            if success:
                x, y, w, h = [int(v) for v in box]
                cv2.rectangle(frame, (x, y), (x + w, y + h),
                    (255, 0, 0), 2)
            
            fps.update()
            fps.stop()

            info = [
                ("Tracker", "CSRT"),
                ("Success", "Yes" if success else "No"),
                ("FPS", "{:.2f}".format(fps.fps()))
            ]

            for i, (k, v) in enumerate(info):
                text = "{}: {}".format(k, v)
                cv2.putText(frame, text, (10, 640 - (i * 20 + 20),),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                initBB = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
                print("Bounding box created!")
                tracker.init(frame, initBB)
                print("Tracker initialised!")
                fps = FPS().start()
                print("FPS started!")
            elif key == ord("q"):
                break

            cv2.destroyAllWindows()
        # # Save the image from the GetImage request to the current directory with the filename
        # # matching that of the image source.
        # image_saved_path = image.source.name + str(time.time())
        # image_saved_path = image_saved_path.replace(
        #     "/", '')  # Remove any slashes from the filename the image is saved at locally.
        # cv2.imwrite(image_saved_path + 'jpg', frame)
    return True


if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)
