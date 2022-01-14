#!/usr/bin/env python

import sys, time
import numpy as np
from scipy.ndimage import filters
import cv2
import roslib
import rospy
print('Hi!')
from sensor_msgs.msg import CompressedImage

class image_read :
    num = 0
    def __init__(self , num):
        self.subscriber = rospy.Subscriber("/hands_on_camera/RGBcamera/image_raw/compressed" , CompressedImage , self.callback, queue_size = 1)
        self.num = num

    def callback(self, ros_data) :
        np_arr = np.fromstring(ros_data.data , np.uint8)
        image_np = cv2.imdecode(np_arr , cv2.IMREAD_COLOR)
        resized_image = cv2.resize(image_np , (800 , 800) , interpolation = cv2.INTER_NEAREST)
        filename = "test" + str(self.num) + ".png"
        cv2.imwrite(filename , resized_image)
        cv2.namedWindow("save" , cv2.WINDOW_NORMAL)
        cv2.imshow('save' , resized_image)
        cv2.imread(filename)
        cv2.waitKey(2)

def main(args) :
    src_img = image_read(args)
    rospy.init_node('camera_test' , anonymous = True)
    try :
        rospy.spin()
    except KeyboardInterrupt :
        print("Shut down !")
        cv2.destoryAllWindows()

if __name__ == '__main__' :
    main(sys.argv)
