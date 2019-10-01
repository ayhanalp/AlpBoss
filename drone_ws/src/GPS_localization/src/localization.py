#!/usr/bin/env python


from geometry_msgs.msg import (PointStamped, PoseStamped, TwistStamped,
                               TransformStamped, Point, QuaternionStamped,
                               Quaternion)
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from std_msgs.msg import String, Int16
from gps_localization.msg import PoseMeas

from dji_sdk.srv import SetLocalPosRef

import numpy as np
import rospy
import sys
import tf
import tf2_ros
import tf2_geometry_msgs as tf2_geom

from fabulous.color import highlight_red, blue, green


class GpsLocalization(object):
    '''
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''
				
        self.init = True
        rospy.init_node('gps_localization')

        self.broadc = tf2_ros.TransformBroadcaster()
        self.stbroadc = tf2_ros.StaticTransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pose_d_in_w = PoseStamped()
        self.pose_d_in_w.header.frame_id = "world"

        self.tf_r_in_w_timestamp_old = rospy.Time.now()
        self.tf_d_in_w_timestamp_old = rospy.Time.now()
        #self.sample_time = rospy.get_param(
        #                                'GPS_localization/sample_time', 0.02)

        self.pos_update = rospy.Publisher(
            'gps_localization/pose', PoseMeas, queue_size=1)
        self.pos_update_rot = rospy.Publisher(
            'gps_localization/pose_rot', PoseMeas, queue_size=1)
        self.ready = rospy.Publisher(
            'gps_localization/ready', Empty, queue_size=1)

        rospy.Subscriber('dji_sdk/local_position', PointStamped, self.get_position_gps)
        rospy.Subscriber('dji_sdk/attitude', QuaternionStamped, self.get_orientation)

    def start(self):
        '''
        Starts running of localization node.
        '''

        self.calibrate()

        self.init_transforms()
				
        rospy.spin()

    def calibrate(self, *_):
        print 'waiting for service local pos ref'
        rospy.wait_for_service("/dji_sdk/set_local_pos_ref")
        try:
            set_local_pos_ref = rospy.ServiceProxy(
                "/dji_sdk/set_local_pos_ref",SetLocalPosRef)
            resp = set_local_pos_ref()
            if not resp.result:
                print highlight_red('Set local pos ref FAILED')
        except rospy.ServiceException, e:
            print highlight_red('Service call failed: %s') % e


        print blue('---- Calibrated ---- \n')

    def init_transforms(self):
        '''
        '''

        self.tf_w_in_ref = TransformStamped()
        self.tf_w_in_ref.header.frame_id = "ref"
        self.tf_w_in_ref.child_frame_id = "world"
        self.tf_w_in_ref.transform.rotation.w = 1.
        self.stbroadc.sendTransform(self.tf_w_in_ref)
        rospy.sleep(2.)

        self.tf_r_in_w = TransformStamped()
        self.tf_r_in_w.header.frame_id = "world"
        self.tf_r_in_w.child_frame_id = "world_rot"

        self.tf_d_in_w = TransformStamped()
        self.tf_d_in_w.header.frame_id = "world"
        self.tf_d_in_w.child_frame_id = "drone"

        # Wait for first gps coordinate
        while self.pose_d_in_w == PoseStamped():
            rospy.sleep(0.1)
        self.tf_d_in_w = self.pose_to_tf(self.pose_d_in_w, "drone")
        self.broadc.sendTransform(self.tf_d_in_w)

        self.init = False       
        self.ready.publish(Empty())
        print green('---- GPS Localization running ----')


    def get_position_gps(self, gps_coord):
        '''Returns PoseStamped of the i'th object in self.tracked_objects.
        Pose is expressed in gps reference frame.

        gps_coord: PointStamped()
        '''
        # READ GPS DATA
        self.pose_d_in_w.header.stamp = rospy.Time.now()
        self.pose_d_in_w.pose.position = gps_coord.point

        if not self.init:
        	self.publish_pose_est()

    def get_orientation(self, quat):
        '''Get yaw. Broadcast new tf.
        '''
        self.pose_d_in_w.header.stamp = quat.header.stamp
        self.pose_d_in_w.pose.orientation = quat.quaternion   	


    def publish_pose_est(self):
        '''Publishes message that calibration is completed. Starts publishing
        pose measurements.
        '''
        self.tf_d_in_w = self.pose_to_tf(self.pose_d_in_w, "drone")

        self.broadc.sendTransform(self.tf_d_in_w)

        # Wait until transform has been updated
        tf_d_in_w = TransformStamped()
        tf_d_in_w.header.stamp = self.tf_d_in_w_timestamp_old
        rate = rospy.Rate(1000.)
        while tf_d_in_w.header.stamp == self.tf_d_in_w_timestamp_old and (
                not rospy.is_shutdown()):

            tf_d_in_w = self.get_transform("drone", "world")
            rate.sleep()
        self.tf_d_in_w_timestamp_old = tf_d_in_w.header.stamp


        # Calculate and broadcast the rotating world frame.
        # - Tf drone in world to euler angles.
        euler = self.get_euler_angles(self.tf_d_in_w)
        # - Get yaw.
        yaw = euler[2]

        # - Yaw only (roll and pitch 0.0) to quaternions.
        quat = tf.transformations.quaternion_from_euler(0., 0., yaw)
        self.tf_r_in_w.transform.rotation = Quaternion(x=quat[0],
                                                       y=quat[1],
                                                       z=quat[2],
                                                       w=quat[3])
        self.tf_r_in_w.header.stamp = rospy.Time.now()
        self.broadc.sendTransform(self.tf_r_in_w)

        # Wait until transform has been updated
        tf_r_in_w = TransformStamped()
        tf_r_in_w.header.stamp = self.tf_r_in_w_timestamp_old
        rate = rospy.Rate(1000.)
        while tf_r_in_w.header.stamp == self.tf_r_in_w_timestamp_old and (
                not rospy.is_shutdown()):

            tf_r_in_w = self.get_transform("world_rot", "world")
            rate.sleep()
        self.tf_r_in_w_timestamp_old = tf_r_in_w.header.stamp

        # Publish pose of drone in world frame as well as yaw angle.
        data = PoseMeas(meas_world=self.pose_d_in_w, yaw=yaw)
        pose_d_in_r = self.transform_pose(self.pose_d_in_w, "world",
                                          "world_rot")
        data_rot = PoseMeas(meas_world=pose_d_in_r, yaw=yaw)

        self.pos_update_rot.publish(data_rot)
        self.pos_update.publish(data)

    def get_euler_angles(self, transf):
        '''
        '''
        quat = (transf.transform.rotation.x,
                transf.transform.rotation.y,
                transf.transform.rotation.z,
                transf.transform.rotation.w)
        euler = tf.transformations.euler_from_quaternion(quat)

        return euler

    def get_transform(self, _from, _to):
        '''
        '''
        tf_f_in_t = self.tfBuffer.lookup_transform(
            _to, _from, rospy.Time(0), rospy.Duration(0.1))

        return tf_f_in_t

    def pose_to_tf(self, pose, child_frame_id):
        '''
        '''
        transf = TransformStamped()
        transf.header.stamp = rospy.Time.now()
        transf.header.frame_id = pose.header.frame_id
        transf.child_frame_id = child_frame_id
        transf.transform.translation = pose.pose.position
        transf.transform.rotation = pose.pose.orientation

        return transf

    def tf_to_pose(self, transf):
        '''
        '''
        pose = PoseStamped()
        pose.header.stamp = transf.header.stamp
        pose.header.frame_id = transf.header.frame_id
        pose.pose.position = transf.transform.translation
        pose.pose.orientation = transf.transform.rotation

        return pose

    def transform_pose(self, pose, _from, _to):
        '''Transforms pose (geometry_msgs/PoseStamped) from frame "_from" to
        frame "_to".
        Arguments:
            - _from, _to = string, name of frame
        '''
        transform = self.get_transform(_from, _to)
        pose_tf = tf2_geom.do_transform_pose(pose, transform)
        pose_tf.header.stamp = pose.header.stamp
        pose_tf.header.frame_id = _to

        return pose_tf


if __name__ == '__main__':
    localization = GpsLocalization()
    localization.start()
