#!/usr/bin/env python

from geometry_msgs.msg import (
    Twist, PoseStamped, Point, PointStamped, TwistStamped)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, Empty, Bool, UInt8

from gps_localization.msg import PoseMeas


from drone_demo.srv import GetPoseEst, GetPoseEstResponse, GetPoseEstRequest

import numpy as np
import math as m
import rospy
import tf2_ros
import tf2_geometry_msgs as tf2_geom

from fabulous.color import (highlight_red, highlight_green, highlight_blue,
                            highlight_yellow, cyan, green)

from perception import *
from world_model import *
from kalman import *


class Demo(object):
    '''
    Acts as hub for the Perception and WorldModel.
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''

        rospy.init_node('drone_demo')

        self.state = "initialization"
        self.state_sequence = []
        self.change_state = False
        self.new_task = False
        self.state_finish = False
        self.omg_standby = False
        self.airborne = False
        self.state_button_held = False
        self.menu_button_held = False
        self.task_dict = {
            "standby": [],
            "invalid measurement": ["emergency"],
            "take-off": ["take-off"],
            "land": ["land"],
            "track drawn trajectory fast": ["take-off",
                                            "build fast trajectory",
                                            "follow path",
                                            "land"],
            "track drawn trajectory slow": ["take-off",
                                            "build slow trajectory",
                                            "follow path",
                                            "land"]}

        self.pose_pub = rospy.Publisher(
            'world_model/yhat', PointStamped, queue_size=1)
        self.pose_r_pub = rospy.Publisher(
            'world_model/yhat_r', PointStamped, queue_size=1)
        self.fsm_state = rospy.Publisher(
            'fsm/state', String, queue_size=1)
        # Initialization finishes when pushing controller buttons
        self.fsm_state.publish("initialization")

        rospy.Subscriber(
            'gps_localization/ready', Empty, self.localization_ready)
        rospy.Subscriber(
            'gps_localization/pose', PoseMeas, self.new_measurement)
        rospy.Subscriber(
            'fsm/task', String, self.switch_task)
        rospy.Subscriber(
            'fsm/state_button', Bool, self.switch_state)
        rospy.Subscriber(
            'controller/state_finish', Empty, self.ctrl_state_finish)
        rospy.Subscriber(
            '/dji_sdk/flight_status', UInt8, self.get_flight_status)
        rospy.Subscriber(
            '/dji_sdk/battery_state', BatteryState, self.get_battery_state)

        self._get_pose_service = None

    def start(self):
        '''
        Starts running of bebop_demo node. Runs along the state sequence, sends
        out the current state and returns to the standby state when task is
        completed.
        '''
        print green('----    Core running     ----')

        while not rospy.is_shutdown():
            if self.new_task:
                self.new_task = False
                self.change_state = False
                self.state_finish = False

                # Run over sequence of states corresponding to current task.
                for state in self.state_sequence:
                    self.state = state
                    print cyan(' Core state changed to: ', self.state)
                    self.fsm_state.publish(state)

                    # IRRELEVANT WHEN NOT USING OMGTOOLS
                    # # Omg tools should return to its own standby status
                    # #  unless the state_button has been pressed.
                    # if self.state in {"omg standby", "omg fly"}:
                    #     self.omg_standby = True
                    # else:
                    #     self.omg_standby = False

                    if self.state == "land":
                        self.change_state = True

                    task_final_state = (self.state == self.state_sequence[-1])
                    # Check if previous state is finished and if allowed to
                    # switch state based on controller input.
                    while not ((self.state_finish and (
                                self.change_state or task_final_state)) or
                               self.new_task or rospy.is_shutdown()):
                        # Remaining in state. Allow state action to continue.
                        rospy.sleep(0.1)

                    self.change_state = False
                    self.state_finish = False

                # OMG STUFF
                    leave_omg = False
                #     leave_omg = (
                #         self.state == "omg standby" and not self.omg_standby)
                #     # User forces leaving omg with state_button or other new
                #     # task received --> leave the for loop for the current
                #     # task.
                    if (leave_omg or self.new_task):
                        # print cyan('---- Broke for loop ----')
                        break

                # # Make sure that omg-tools task is repeated until force quit.
                # if self.omg_standby:
                #     self.new_task = True

                # Only publish standby state when task is finished.
                # Except for repetitive tasks (back to first state in task).
                if not self.new_task:
                    self.state = "standby"
                    self.fsm_state.publish("standby")
                    print cyan(' Core state changed to: ', "standby")
            
            #print 'core sleeping'
            rospy.sleep(0.1)

    def localization_ready(self, *_):
        '''
        Provides get_pose service when localization is calibrated.
        '''
        self._get_pose_service = rospy.Service(
            "/world_model/get_pose", GetPoseEst, self.get_kalman_pos_est)

    def get_kalman_pos_est(self, req_vel):
        '''
        Receives a time-stamped twist and returns an estimate of the position
        Argument:
            - req_vel = TwistStamped
        '''
        # Don't do prediction and transformation calculations if the
        # measurement is invalid.
        if (not self.measurement_valid) and (
             not self.state_sequence == ["emergency"]):
            req_vel.input_cmd.twist = Twist()
            inv_meas_task = String(data="invalid measurement")
            self.switch_task(inv_meas_task)

        self.kalman.input_cmd_list.append(req_vel.input_cmd)
        self.kalman.latest_input_cmd = req_vel.input_cmd

        self.wm.yhat_r, self.wm.vhat_r = self.kalman.kalman_pos_predict(
                                self.kalman.latest_input_cmd, self.wm.yhat_r)

        # Transform the rotated yhat and vhat to world frame.
        self.wm.yhat = self.transform_point(
            self.wm.yhat_r, "world_rot", "world")
        self.wm.vhat = self.transform_point(
            self.wm.vhat_r, "world_rot", "world")
        self.wm.yaw = self.pc.yaw

        self.pose_r_pub.publish(self.wm.yhat_r)
        self.pose_pub.publish(self.wm.yhat)

        return GetPoseEstResponse(
            self.wm.yhat, self.wm.vhat, self.wm.yaw, self.measurement_valid)

    def new_measurement(self, data):
        '''Processes incoming measurement from Localization.
        data:
            meas_world: PoseStamped
            yaw: float32
        '''
        self.pc.pose_raw = data.meas_world
        self.pc.yaw = data.yaw
        self.measurement_valid = self.pc.measurement_check()

        measurement = self.kalman.transform_pose(
                                self.pc.pose_raw, "world", "world_rot")
        if self.measurement_valid:
            if self.kalman.init:
                self.wm.yhat_r_t0.header = measurement.header
                zero_input_cmd = TwistStamped()
                zero_input_cmd.header = measurement.header
                self.kalman.input_cmd_list = [zero_input_cmd]
                self.kalman.init = False

            # Apply correction step.
            self.wm.yhat_r, self.wm.yhat_r_t0 = self.kalman.kalman_pos_correct(
                                                measurement, self.wm.yhat_r_t0)
            self.wm.yhat = self.transform_point(
                self.wm.yhat_r, "world_rot", "world")
            self.pose_pub.publish(self.wm.yhat)

####################
# Task functions #
####################

    def switch_task(self, task):
        '''Reads out the task topic and switches to the desired task.
        '''
        if task.data not in self.task_dict:
            print highlight_red(
                    ' Not a valid task, drone will remain in standby state.')

        self.state_sequence = self.task_dict.get(task.data, [])
        self.new_task = True
        print cyan(' Core received a new task: ', task.data)

    # REPLACE WITH TAKE-OFF/LANDING PROCEDURE DJI
    def take_off_land(self, pressed):
        '''Check if menu button is pressed and switch to take-off or land
        sequence depending on last task that was executed.
        '''
        if pressed.data and not self.menu_button_held:
            if self.airborne:
                self.state_sequence = self.task_dict.get("land", [])
            else:
                self.state_sequence = self.task_dict.get("take-off", [])
            self.new_task = True
            print cyan(
                ' Bebop_core received a new task: ',
                self.state_sequence[0])
            self.menu_button_held = True
        elif not pressed.data and self.menu_button_held:
            self.menu_button_held = False

####################
# Helper functions #
####################

    def switch_state(self, state_button_pressed):
        '''When state_button is pressed changes change_state variable
        to true to allow fsm to switch states in state sequence.
        '''
        if (state_button_pressed.data and not self.state_button_held) and (
                self.state not in {"standby", "initialization"}):
            self.state_button_held = True
            if self.state == "omg standby":
                self.omg_standby = False
                self.new_task = False
            self.change_state = True
            print highlight_blue(' Switching to next state ')

        elif not state_button_pressed.data and self.state_button_held:
            self.state_button_held = False

    def get_flight_status(self, flight_status):
        '''Checks whether the drone is standing on the ground or flying and
        changes the self.airborne variable accordingly.
        '''
        if flight_status.data == 1:
            self.airborne = False
        elif flight_status.data == 3:
            self.airborne = True

    # REPLACE WITH DJI BATTERY STATE INFO (dji sdk probably)
    def get_battery_state(self, battery):
        '''Checks the discharge state of the battery and gives a warning
        when the battery voltage gets low.
        '''
        if ((battery.percentage <= 0.2) and ((battery.percentage % 5) == 0)):
            print 'battery.percent', battery.percentage, (battery.percentage % 5)
            print highlight_yellow(
                        ' Battery voltage low -- ', battery.percentage,
                        '% left, switch to a freshly charged battery! ')

    def ctrl_state_finish(self, empty):
        '''Checks whether controller has finished the current state.
        '''
        print 'ctrl state finish received'
        self.state_finish = True

    def transform_point(self, point, _from, _to):
        '''Transforms point (geometry_msgs/PointStamped) from frame "_from" to
        frame "_to".
        Arguments:
            - _from, _to = string, name of frame
        '''
        transform = self.kalman.get_transform(_from, _to)
        point_transformed = tf2_geom.do_transform_point(point, transform)

        return point_transformed


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.kalman = Kalman(demo.wm.model)
    demo.start()
