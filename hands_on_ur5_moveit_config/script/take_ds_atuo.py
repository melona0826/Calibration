#!/usr/bin/env python
import sys, time
import copy
import rospy
import roslib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from scipy.ndimage import filters
from sensor_msgs.msg import CompressedImage
from tf.transformations import quaternion_from_euler
import cv2

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class auto_calibration :
    def cal

class image_read :
    num = 0
    def __init__(self, num):
        self.subscriber = rospy.Subscriber("/hands_on_camera/RGBcamera/image_raw/compressed" , CompressedImage , self.callback, queue_size = 1)
        self.num = num

    def callback(self, ros_data) :
        np_arr = np.fromstring(ros_data.data , np.uint8)
        image_np = cv2.imdecode(np_arr , cv2.IMREAD_COLOR)
        resized_image = cv2.resize(image_np , (800 , 800) , interpolation = cv2.INTER_NEAREST)
        filename = 'testing' + str(self.num) + '.png'
        cv2.imwrite(filename , resized_image)
        return "wow"
        # cv2.namedWindow("testing" , cv2.WINDOW_NORMAL)
        # cv2.imshow('testing' , resized_image)
        # cv2.waitKey(2)

def main(args):

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('save_img' , anonymous = True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "Manipulators"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

    print("-------")
    eef_link = group.get_end_effector_link()
    print ("============ End effector: %s" % eef_link)

    quaternion = quaternion_from_euler(0, 1.57, 0)

    print(quaternion)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]


    pose_goal.position.x = 0.817
    pose_goal.position.y = 0.22
    pose_goal.position.z = 1.3
    group.set_pose_target(pose_goal , "camera_link")

    plan = group.go(wait = True)
    group.stop()
    group.clear_pose_targets()






if __name__ == '__main__' :
    main(sys.argv)
