#!/usr/bin/env python

from std_msgs.msg import Empty, UInt8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from gps_localization.msg import PoseMeas
import rospy
import numpy as np
import scipy.io as io


class Ident(object):

    def __init__(self):
        """
        """
        self.input_cmd_max = 0.15
        self.ident_length = 3
        self.index = 0
        self.rate = 14
        self.wait1 = 0.1
        self.wait2 = 0.11
        self.wait3 = 0.10
        span = int(self.ident_length*(
                    self.wait1*4 + self.wait2+self.wait3)*self.rate*50 + 10)
        self.input = np.zeros(span)
        self.output_x = np.zeros(span)
        self.output_y = np.zeros(span)
        self.output_z = np.zeros(span)
        self.time = np.zeros(span)
        self.input = Twist()
        self.measuring = False

        self.cmd_input = rospy.Publisher('/dji_sdk/flight_control_setpoint_generic',
                                         Joy, queue_size=1)
        
        rospy.Subscriber('demo', Empty, self.flying)
        rospy.Subscriber(
            'GPS_localization/pose', PoseMeas, self.update_pose)

    def start(self):
        rospy.init_node('identification')
        print 'identification started'
        rospy.spin()

    def flying(self, empty):

        # Take off
        try:
            takeoff = rospy.ServiceProxy(
                "/dji_sdk/drone_task_control", DroneTaskControl)
            takeoff_success = takeoff(task=4)
        except rospy.ServiceException, e:
            print highlight_red('Takeoff service call failed: %s') % e
            takeoff_success = False
        print 'Flying, starting experiment in 10s'

        rospy.sleep(10)


        # move back and forth with a pause in between
        self.input.linear.x = 0.0
        self.input.linear.y = 0.0
        self.input.linear.z = 0.0
        self.measuring = True

        for k in range(0, self.ident_length):
            self.input.linear.x = self.input_cmd_max/3.

            for x in range(0, self.rate):
                self.cmd_input.publish(self.input)
                rospy.sleep(self.wait1)

            self.input.linear.x = -self.input_cmd_max/3.

            for x in range(0, self.rate):
                self.cmd_input.publish(self.input)
                rospy.sleep(self.wait2)

            self.input.linear.x = self.input_cmd_max/3.*2.

            for x in range(0, self.rate):
                self.cmd_input.publish(self.input)
                rospy.sleep(self.wait1)

            self.input.linear.x = -self.input_cmd_max/3.*2.

            for x in range(0, self.rate):
                self.cmd_input.publish(self.input)
                rospy.sleep(self.wait3)

            self.input.linear.x = self.input_cmd_max

            for x in range(0, self.rate):
                self.cmd_input.publish(self.input)
                rospy.sleep(self.wait1)

            self.input.linear.x = -self.input_cmd_max

            for x in range(0, self.rate):
                self.cmd_input.publish(self.input)
                rospy.sleep(self.wait1)

        self.measuring = False

        self.input.linear.x = 0.0
        self.input.linear.y = 0.0
        self.input.linear.z = 0.0
        self.cmd_input.publish(self.input)

        rospy.sleep(1)

        try:
            land = rospy.ServiceProxy(
                "/dji_sdk/drone_task_control", DroneTaskControl)
            takeoff_success = takeoff(task=6)
            print 'Landed'

        except rospy.ServiceException, e:
            print highlight_red('Land service call failed: %s') % e
            takeoff_success = False

        meas = {}
        meas['input'] = self.input
        meas['output_x'] = self.output_x
        meas['output_y'] = self.output_y
        meas['output_z'] = self.output_z
        meas['time'] = self.time
        io.savemat('../angle_identification_x.mat', meas)

    def update_pose(self, meas):
        if self.measuring:
            self.input[self.index] = self.input.linear.x
            self.output_x[self.index] = meas.meas_world.pose.position.x
            self.output_y[self.index] = meas.meas_world.pose.position.y
            self.output_z[self.index] = meas.meas_world.pose.position.z
            self.time[self.index] = meas.meas_world.header.stamp.to_sec()
            self.index += 1

    def send_input(self, input_cmd):
        '''Publish input command both as a Twist() (old bebop style,
        needed for Kalman) and as a sensor_msgs/Joy msg for DJI drone.
        
        input_cmd: Twist()
        '''
        self.input = input_cmd
        self.cmd_vel.publish(self.full_cmd.twist)

        flag = UInt8(data=np.uint8(0))  # VERT_VEL, HORI_ATTI_TILT_ANG, YAW_ANG
        # flag = UInt8(data=np.uint8(8))  # VERT_VEL, HORI_ATTI_TILT_ANG, YAW_RATE

        full_cmd_dji = Joy()
        full_cmd_dji.header = self.full_cmd.header
        full_cmd_dji.axes = [full_cmd.linear.y,
                             full_cmd.linear.x,
                             -full_cmd.linear.z,
                             -full_cmd.angular.z,
                             flag]

        self.cmd_vel_dji.publish(full_cmd_dji)


if __name__ == '__main__':
    Billy = Ident()
    Billy.start()
