#!/usr/bin/env python

from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from gps_localization.msg import PoseMeas

from dji_sdk.srv import DroneTaskControl, SDKControlAuthority

import rospy
import numpy as np
import scipy.io as io


class Ident(object):

    def __init__(self):
        """
        """
        rospy.init_node('identification')
        self.input_angle_max = 0.4
        self.index = 0
        Ts = 0.02
        self.rate = rospy.Rate(1/Ts)
        nrofcycles = 5
        
        self.input = ([self.input_angle_max for i in range(0,80)] + 
								[-self.input_angle_max for i in range(0,160)] +
								[self.input_angle_max for i in range(0,160)] +
								[-self.input_angle_max for i in range(0,80)] +
								[self.input_angle_max for i in range(0,160)] + 
								[-self.input_angle_max for i in range(0,80)] +
								[self.input_angle_max for i in range(0,80)] +
								[-self.input_angle_max for i in range(0,160)])
        print 'len self.input', len(self.input)
        self.input = self.input*nrofcycles
        self.span = len(self.input)
        self.input_rec = np.zeros(self.span*50)
        self.output_x = np.zeros(self.span*50)
        self.output_y = np.zeros(self.span*50)
        self.output_z = np.zeros(self.span*50)
        self.time = np.zeros(self.span*50)
        self.input_cmd = Twist()
        self.measuring = False

        # Controller flag settings (dji sdk)
        self.VERTICAL_VEL = 0x00
        self.VERTICAL_POS = 0x10
        self.VERTICAL_THRUST = 0x20

        self.HORIZONTAL_ANGLE = 0x00
        self.HORIZONTAL_VELOCITY = 0x40
        self.HORIZONTAL_POSITION = 0x80
        self.HORIZONTAL_ANGULAR_RATE = 0xC0

        self.YAW_ANGLE = 0x00
        self.YAW_RATE = 0x08

        self.HORIZONTAL_GROUND = 0x00
        self.HORIZONTAL_BODY = 0x02

        self.STABLE_DISABLE = 0x00
        self.STABLE_ENABLE = 0x01

        # TOPICS
        self.cmd_vel_dji = rospy.Publisher('/dji_sdk/flight_control_setpoint_generic',
                                         Joy, queue_size=1)
        
        rospy.Subscriber('demo', Empty, self.flying)
        rospy.Subscriber(
            'gps_localization/pose_rot', PoseMeas, self.update_pose)

    def start(self):
 
        print 'Ready to go'
        rospy.spin()

    def flying(self, empty):
        print 'Identification experiment started'
        
        # Request control authority from the sdk.
        self.get_control_authority()

        # Take off
        self.take_off()
        print 'Taking off, starting experiment in a few seconds'

        rospy.sleep(8)

        self.input_cmd = Twist()
        for x in range(0, 100):
           self.send_input(self.input_cmd)
           self.rate.sleep()
        

        # START RECORDING ----------------------------
        self.measuring = True

        print 'start measurements'
        for i in range(0,self.span):
           self.input_cmd.linear.y = self.input[i]
           #self.input_cmd.angular.z = self.input[i]
           #print self.input[i]
           self.send_input(self.input_cmd)
           self.rate.sleep()
        
        self.measuring = False
        # STOP RECORDING -----------------------------

        print 'stop measurements'
        self.input_cmd = Twist()
        for x in range(0, 100):
           self.send_input(self.input_cmd)
           self.rate.sleep()

        self.land()
        print self.index
        # STORE THE DATA
        meas = {}
        meas['input'] = self.input_rec
        meas['output_x'] = self.output_x
        meas['output_y'] = self.output_y
        meas['output_z'] = self.output_z
        meas['time'] = self.time
        print 'identification experiment terminated'

        io.savemat('../identification_y.mat', meas)
        print 'data stored'

    def update_pose(self, meas):
        if self.measuring:
            print 'measuring', self.index
            self.input_rec[self.index] = self.input_cmd.linear.y
            self.output_x[self.index] = meas.meas_world.pose.position.x
            self.output_y[self.index] = meas.meas_world.pose.position.y
            self.output_z[self.index] = meas.meas_world.pose.position.z
            self.time[self.index] = meas.meas_world.header.stamp.to_sec()
            self.index += 1

    def take_off(self):

        print 'Trying to take off'
        try:
            takeoff = rospy.ServiceProxy(
                "/dji_sdk/drone_task_control", DroneTaskControl)
            takeoff_success = takeoff(np.uint8(4))
            print 'inside try'
        except rospy.ServiceException, e:
            print highlight_red('Takeoff service call failed: %s') % e
            takeoff_success = False
        print takeoff_success

    def land(self):

        try:
            land = rospy.ServiceProxy(
                "/dji_sdk/drone_task_control", DroneTaskControl)
            land_success = land(task=6)
            print 'Landing'

        except rospy.ServiceException, e:
            print highlight_red('Land service call failed: %s') % e
            land_success = False

    def send_input(self, input_cmd):
        '''Publish input command both as a Twist() (old bebop style,
        needed for Kalman) and as a sensor_msgs/Joy msg for DJI drone.
        
        input_cmd: Twist()
        '''
        #flag = 0
        flag = np.uint8(self.VERTICAL_VEL|
                        self.HORIZONTAL_ANGLE|
                        self.YAW_RATE|
                        self.HORIZONTAL_BODY|
                        self.STABLE_ENABLE)
        cmd_dji = Joy()
        cmd_dji.header.frame_id = "world_rot"
        cmd_dji.header.stamp = rospy.Time.now()
        cmd_dji.axes = [-input_cmd.linear.y,
                             input_cmd.linear.x,
                             input_cmd.linear.z,
                             input_cmd.angular.z,
                             flag]

        self.cmd_vel_dji.publish(cmd_dji)

    def get_control_authority(self):

        print 'Asking control authority'
        rospy.wait_for_service("/dji_sdk/sdk_control_authority")
        try:
            ctrl_auth = rospy.ServiceProxy(
                "/dji_sdk/sdk_control_authority", SDKControlAuthority)
            ctrl_success = ctrl_auth(1)
            print 'ctrl_success', ctrl_success
        except rospy.ServiceException, e:
            print 'Ctrl authority service call failed: %s' % e
            ctrl_success = False
        print ctrl_success

if __name__ == '__main__':
    Billy = Ident()
    Billy.start()
