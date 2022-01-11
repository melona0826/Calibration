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
import cv2

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
