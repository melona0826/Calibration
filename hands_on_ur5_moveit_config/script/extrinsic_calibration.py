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

        self.objp = np.zeros((self.checker_row * self.checker_col , 3) , np.float64)
        self.objp[: , :2] = np.mgrid[0:self.checker_row , 0:self.checker_col].T.reshape(-1 , 2)*25

        self.camera_mtx = np.array( [[947.937514 , 0.000 , 647.198897] , [0.000 , 949.744496 , 362.029067] , [0.000 , 0.000 , 1.000]] , dtype = np.float64)
        self.dist_coef = np.array( [0.134149 , -0.190451 , 0.000198 , 0.007935 , 0.000] , dtype = np.float64)

        self.objpoints = np.array([] , dtype = np.float64)
        self.imgpoints = []

        self.axis = np.float32([[3, 0, 0] , [0, 3, 0] , [0, 0, -3]]).reshape(-1 , 3)

    def draw_axis(img , corners, imgpts) :
        corners = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravle()) , (255, 0, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravle()) , (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravle()) , (0, 0, 255), 5)
        return img


    def shutdown_print() :
        print(" ")

    def imageCallback(self, data) :
        try :
            cv_image = self.bridge.imgmsg_to_cv2(data , desired_encoding='bgr8')

        except CvBridgeError as e:
            print(e)
            return

        self.gray_img = cv2.cvtColor(cv_image , cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(self.gray_img , (self.checker_row , self.checker_col) , None)
        if ret == True:
            self.objpoints = np.append(self.objpoints , self.objp)
            corners2 = cv2.cornerSubPix(self.gray_img,corners,(11,11),(-1,-1),self.criteria)
            self.imgpoints.append(corners2)
            self.imgpoints[:][:].append(0.000)
            img_np = np.array(self.imgpoints)
            ret , revecs, tvecs = cv2.solvePnP(self.objpoints , img_np , self.camera_mtx , self.dist_coef )

            imgpts, jac = cv2.projectPoints(axis, rvecs , tvecs , camera_mtx , dist_coef)

            img = draw(img , corners2 , imgpts)
            cv2.imshow('img' , img)
            cv2.waitKey(5)

        else :
            cv2.imshow('image' , cv_image)
            cv2.waitKey(2)




def main(args) :

    rospy.init_node("Image_test")
    topic = "/camera/color/image_raw"
    calib = Calibration(topic)
    rospy.spin()

if __name__ == "__main__" :
    main(sys.argv)
