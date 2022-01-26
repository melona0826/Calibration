#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import signal

import sys
import os

class Calibration :
    def __init__(self, topic) :
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic , Image , self.imageCallback)
        self.checker_row = 8
        self.checker_col = 6

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER , 30 , 0.001)

        self.objp = np.zeros((self.checker_row * self.checker_col , 3) , np.float32)
        self.objp[: , :2] = np.mgrid[0:self.checker_row , 0:self.checker_col].T.reshape(-1 , 2)*25

        self.camera_mtx = np.array( [[612.109570, 0.000000, 310.362412] , [0.000000, 627.545878, 262.814425] , [0.000 , 0.000 , 1.000]] , dtype = np.float32)
        self.dist_coef = np.array( [0.150593, -0.398806, 0.003193, -0.006965, 0.000000] , dtype = np.float32)

        self.axis = np.float32([[20,0,0] , [0,20,0] , [0,0,-20]]).reshape(-1,3)


    def draw(self, img, corners, imgpts) :
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()) , (255, 0, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()) , (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()) , (0, 0, 255), 5)
        return img

    def imageCallback(self, data) :
        try :
            cv_image = self.bridge.imgmsg_to_cv2(data , desired_encoding='bgr8')

        except CvBridgeError as e:
            print(e)
            return

        gray_img = cv2.cvtColor(cv_image , cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_img , (self.checker_row , self.checker_col) , None)
        if ret == True:
            corners2 = cv2.cornerSubPix(gray_img,corners,(11,11),(-1,-1),self.criteria)

            ret, rvecs, tvecs = cv2.solvePnP(self.objp , corners2, self.camera_mtx, self.dist_coef, cv2.SOLVEPNP_P3P)

            imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.camera_mtx , self.dist_coef)

            R, _ = cv2.Rodrigues(rvecs)

            print("-------Rotation Matrix----------")
            print(R)
            print("-------Translation Matrix---------")
            print(tvecs)

            print("-------Transfrom Matrix--------")
            inv = np.concatenate((R , tvecs ) , axis = 1)
            ones = np.array([(0,0,0,1)] , np.float32)

            inv = np.concatenate((inv , ones) , axis = 0)
            inv_result = np.linalg.inv(inv)
            print("-------Inverse Matrix----------")
            print(inv_result)
            cv_image = self.draw(cv_image, corners2 , imgpts)

        cv2.imshow('img' , cv_image)
        cv2.waitKey(2)




def main(args) :

    rospy.init_node("Image_test")
    topic = "/camera/color/image_raw"
    calib = Calibration(topic)
    rospy.spin()

if __name__ == "__main__" :
    main(sys.argv)
