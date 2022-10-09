## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        prev_time=time.time()
        
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        cur_time=time.time()
        print("Part 1 time (getting the frame): ", cur_time-prev_time)
        prev_time=cur_time

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        color_colormap_dim = color_image.shape
        color_image = cv2.rotate(color_image, cv2.cv2.ROTATE_90_CLOCKWISE)
        
        cur_time=time.time()
        print("Part 2 time (convert images into numpy arrays): ", cur_time-prev_time)
        prev_time=cur_time

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)

        cur_time=time.time()
        print("Part 3 time (show images): ", cur_time-prev_time)
        prev_time=cur_time        

finally:

    # Stop streaming
    pipeline.stop()
