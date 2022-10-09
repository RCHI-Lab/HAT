#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
import cv2
import numpy as np
import time
import os

#os.nice(-20)

samp_freq=10
last_read_time = time.time()
def callback(data):
  global last_read_time
  global samp_freq
  curr_time = time.time()
  if curr_time - last_read_time > 1.0/samp_freq:
    br = CvBridge()
    rospy.loginfo("receiving video frame")
    current_frame = br.imgmsg_to_cv2(data)
    current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
    cv_image_array = np.array(current_frame, dtype = np.dtype('f8'))
    cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
    color_image = cv2.rotate(cv_image_norm, cv2.cv2.ROTATE_90_CLOCKWISE)

    h,w = color_image.shape[0:2]
    color_image = cv2.resize(color_image, (int(w*2.3), int(h*2.3)))
    cv2.imshow("camera", color_image)
    #cv2.resizeWindow("camera", 500, 300)
    cv2.waitKey(1)
    #print(curr_time - last_read_time, 1.0/samp_freq)
    last_read_time = curr_time

  #r.sleep()

    

def receive_message():
  rospy.init_node('video_sub_py', anonymous=True)
  rospy.Subscriber('/camera/color/image_raw', Image, callback, queue_size = 1)
  rospy.spin()
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  #cv2.namedWindow("camera", cv2.WINDOW_KEEPRATIO)
  
  receive_message()
