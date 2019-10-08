#!/usr/bin/env python

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8

import numpy as np
import tf2_ros
import rospy

from fabulous.color import highlight_red


class Perception(object):
    '''
    Read out position of the drone, pose of the drone and position of the
    obstacles
    '''

    def __init__(self):
        """
        Initialization of Perception object.
        """
        self.pose_vive = TwistStamped()
        self.pose_drone = Twist()
        self.twist_drone = Twist()
        self.tf_t_in_w_prev = TransformStamped()
        self.init = True
        self.vive_calibrating = False

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Name of topic can change depending on name used in the code for
        # reading out the vive.
        rospy.Subscriber('/dji_sdk/gps_health', UInt8, self.store_gps_health)

    def get_drone_data(self, data):
        """
        Updates pose and twist data by using measurements on the drone itself.
        """
        self.pose_drone = data.pose.pose
        self.twist_drone = data.twist.twist

    def measurement_check(self):
        '''Monitor function: check GPS health.
        '''
        measurement_valid = False
        if self.gps_health:
            measurement_valid = (self.gps_health >= 3)

        return measurement_valid

    def store_gps_health(self, health):
        self.gps_health = health

    def get_transform(self, _from, _to):
        '''
        '''
        tf_f_in_t = self.tfBuffer.lookup_transform(
            _to, _from, rospy.Time(0), rospy.Duration(0.1))

        return tf_f_in_t


if __name__ == '__main__':
    perception = Perception()
