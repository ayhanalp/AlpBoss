#!/usr/bin/env python

from geometry_msgs.msg import (Twist, TwistStamped, Point, PointStamped,
                               Pose, PoseStamped)
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, String, UInt8
from visualization_msgs.msg import Marker, MarkerArray
from drone_demo.msg import Trigger, Trajectories, Obstacle
from gps_localization.msg import PoseMeas

from drone_demo.srv import GetPoseEst, ConfigMotionplanner
from dji_sdk.srv import DroneTaskControl, SDKControlAuthority

import rospy
import numpy as np
import scipy.io as io
from scipy.signal import butter, filtfilt, lfilter
import tf
import tf2_ros
import tf2_geometry_msgs as tf2_geom

from fabulous.color import (highlight_red, highlight_green, highlight_blue,
                            green, yellow, highlight_yellow)


class Controller(object):

    ############################
    # Initialization functions #
    ############################
    def __init__(self):
        """Initialization of Controller object.
        """
        rospy.init_node("controller")

        self.state = "initialization"
        self.state_dict = {"standby": self.hover,
                           "emergency": self.repeat_safety_brake,
                           "take-off": self.take_off_land,
                           "land": self.take_off_land,
                           "build fast trajectory": self.build_traj,
                           "build slow trajectory": self.build_traj,
                           "follow path": self.follow_traj,
                           "reset PID": self.reset_pid_gains}

        self._init_constants()
        self._init_params()
        self._init_variables()
        self._marker_setup()

        self._init_vel_model()
        self._init_topics()
        self._request_ctrl_authority()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def _init_vel_model(self):
        '''Initializes model parameters for conversion of desired velocities to
        angle inputs.Discrete time state space model (Ts=0.01s) of the
        inverted, LPF filtered velocity system.

        '''
        Ax = np.array([
                    [2.824240357667064, -1.331794270100174, 0.838724830380165],
                    [2.0,    0.,    0.],
                    [0.,     0.5,   0.]])
        Ay = np.array([
                    [2.824240357667064, -1.331794270100174, 0.838724830380165],
                    [2.0,    0.,    0.],
                    [0.,     0.5,   0.]])
        Az = np.array([
                    [2.849310422521509, -1.354880669436438, 0.860053408185684],
                    [2.0,    0.,    0.],
                    [0.,     0.5,   0.]])

        self.A = np.zeros([9, 9])
        self.A[0:3, 0:3] = Ax
        self.A[3:6, 3:6] = Ay
        self.A[6:9, 6:9] = Az

        self.B = np.zeros([9, 3])
        self.B[0, 0] = 0.25  # Bx
        self.B[3, 1] = 0.25  # By
        self.B[6, 2] = 0.50  # Bz

        self.C = np.zeros([3, 9])
        Cx = [0.137390474398463,
              -0.132646688192171,
              0.127900741701118]
        Cy = [0.134968479308109,
              -0.130191644509422,
              0.125414137842811]
        Cz = [0.208920377233792,
              -0.199803776960514,
              0.191477872249400]
        self.C[0, 0:3] = Cx
        self.C[1, 3:6] = Cy
        self.C[2, 6:9] = Cz
        Dx = 0.018104848888170
        Dy = 0.017769811206118
        Dz = 0.053915103541076
        self.D = np.array([[Dx, 0.0, 0.0],
                           [0.0, Dy, 0.0],
                           [0.0, 0.0, Dz]])

    def _init_topics(self):
        '''Initializes rostopic Publishers and Subscribers.
        '''
        self.cmd_vel_dji = rospy.Publisher(
            '/dji_sdk/flight_control_setpoint_generic',
            Joy, queue_size=1)
        self.cmd_vel = rospy.Publisher(
            'drone/cmd_vel', Twist, queue_size=1)
        self.vhat_vector_pub = rospy.Publisher(
            'motionplanner/vhat_vector', Marker, queue_size=1)
        self.trajectory_desired = rospy.Publisher(
            'motionplanner/desired_path', Marker, queue_size=1)
        self.trajectory_real = rospy.Publisher(
            'motionplanner/real_path', Marker, queue_size=1)
        self.trajectory_drawn = rospy.Publisher(
            'motionplanner/drawn_path', Marker, queue_size=1)
        self.trajectory_smoothed = rospy.Publisher(
            'motionplanner/smoothed_path', Marker, queue_size=1)
        self.current_ff_vel_pub = rospy.Publisher(
            'motionplanner/current_ff_vel', Marker, queue_size=1)
        self.draw_room = rospy.Publisher(
            'motionplanner/room_contours', Marker, queue_size=1)
        self.ctrl_state_finish = rospy.Publisher(
            'controller/state_finish', Empty, queue_size=1)
        self.pos_error_pub = rospy.Publisher(
            'controller/position_error', PointStamped, queue_size=1)

        rospy.Subscriber('gps_localization/ready', Empty,
                         self.publish_obst_room)
        rospy.Subscriber('fsm/state', String, self.switch_state)
        rospy.Subscriber(
            '/dji_sdk/flight_status', UInt8, self.get_flight_status)

    def _init_constants(self):
        '''Initialize non-configurable values.
        '''
        # Controller flag settings (dji sdk)
        # The controller flag is the bitwise 'OR' of these five flags.
        # (see DJI sdk documentation)
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

        self.service_timeout = 10.

    def _init_params(self):
        '''Initializes (reads and sets) externally configurable parameters
        (rosparams).
        '''
        self._sample_time = rospy.get_param('controller/sample_time', 0.02)
        self.rate = rospy.Rate(1./self._sample_time)
        self.Kp_x = rospy.get_param('controller/Kp_x', 0.6864)
        self.Ki_x = rospy.get_param('controller/Ki_x', 0.6864)
        self.Kd_x = rospy.get_param('controller/Kd_x', 0.6864)
        self.Kp_y = rospy.get_param('controller/Kp_y', 0.6864)
        self.Ki_y = rospy.get_param('controller/Ki_y', 0.6864)
        self.Kd_y = rospy.get_param('controller/Kd_y', 0.6864)
        self.Kp_z = rospy.get_param('controller/Kp_z', 0.5)
        self.Ki_z = rospy.get_param('controller/Ki_z', 1.5792)
        self.Kd_z = rospy.get_param('controller/Kd_z', 1.5792)
        self.Kp_yaw = rospy.get_param('controller/Kp_yaw', 0.3)
        self.Ki_yaw = rospy.get_param('controller/Ki_yaw', 0.3)
        self.Kd_yaw = rospy.get_param('controller/Kd_yaw', 0.3)

        self.max_input = rospy.get_param('controller/max_input', 0.5)
        self.max_z_input = rospy.get_param('controller/max_z_input', 0.5)
        self.max_yaw_input = rospy.get_param('controller/max_yaw_input', 0.5)

        self.pos_nrm_tol = rospy.get_param(
                                       'controller/goal_reached_pos_tol', 0.05)

        self.room_width = 1000.
        self.room_height = 1000.
        self.room_depth = 1000.

        # Setup low pass filter for trajectory drawing task.
        cutoff_freq_LPF = rospy.get_param('controller/LPF_cutoff', 0.5)
        LPF_order = rospy.get_param('controller/LPF_order', 4)

        norm_fc_LPF = cutoff_freq_LPF/(0.5)*self._sample_time
        self.butter_b, self.butter_a = butter(
            LPF_order, norm_fc_LPF, btype='low', analog=False)

    def _init_variables(self):
        '''Initializes variables that are used later on.
        '''
        # State related variables
        self.airborne = False
        self.calc_succeeded = False
        self.target_reached = False
        self.startup = False
        self.state_changed = False
        self.executing_state = False
        self.state_killed = False
        self.overtime = False

        # Measurement related variables
        self.meas_pos_x = 0.
        self.meas_pos_y = 0.
        self.meas_pos_z = 0.
        self.meas_time = 0.

        # Other
        self._traj = {'u': [0.0], 'v': [0.0], 'w': [0.0],
                      'x': [0.0], 'y': [0.0], 'z': [0.0]}
        self._traj_strg = {'u': [0.0], 'v': [0.0], 'w': [0.0],
                           'x': [0.0], 'y': [0.0], 'z': [0.0]}
        self.X = np.array(
                    [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
        self.desired_yaw = 0.
        self.drone_yaw_est = 0.0
        self.pos_nrm = np.inf
        self.fb_cmd_prev = Twist()
        self.pos_error_prev = PointStamped()
        self.vel_error_prev = PointStamped()
        self.yaw_error_prev = 0.
        self.measurement_valid = False
        self._goal = Pose()
        self.hover_setpoint = Pose()
        self.draw = False
        self.drag = False

        self.full_cmd = TwistStamped()
        self.full_cmd.header.frame_id = "world_rot"
        self.full_cmd.header.stamp = rospy.Time.now()
        self.ff_velocity = TwistStamped()
        self.ff_cmd = Twist()
        self.drone_vel_est = Point()
        self.drone_pose_est = Pose()
        self.drone_yaw_est = 0.

    def _request_ctrl_authority(self):
        '''Service call to dji sdk to obtain control authority.
        '''
        rospy.wait_for_service(
            "/dji_sdk/sdk_control_authority", timeout=self.service_timeout)
        try:
            ctrl_auth = rospy.ServiceProxy(
                "/dji_sdk/sdk_control_authority", SDKControlAuthority)
            ctrl_success = ctrl_auth(1)
        except rospy.ServiceException, e:
            rospy.logerr('Ctrl authority service call failed: %s' % e)
            print 'Ctrl authority service call failed: %s' % e
            ctrl_success = False
        if ctrl_success:
            print yellow('---- Control authority obtained ----')

    ##################
    # Main functions #
    ##################

    def start(self):
        '''Configures,
        Starts the controller's periodical loop.
        '''
        self.draw_room_contours()
        print green('----    Controller running     ----')

        while not rospy.is_shutdown():
            if self.state_changed:
                self.state_changed = False
                print yellow(' Controller state changed to: ', self.state)
                self.executing_state = True
                # Execute state function.
                self.state_dict[self.state]()
                print "done executing"
                self.executing_state = False

                # State has not finished if it has been killed!
                if not self.state_killed:
                    print 'finish pub'
                    self.ctrl_state_finish.publish(Empty())
                    print yellow('------------------------')
                self.state_killed = False
                print 'checkpoint 1'
                # Adjust goal to make sure hover uses PID actions to stay in
                # current place.
                self.full_cmd.header.stamp = rospy.Time.now()
                (self.drone_pose_est, self.drone_vel_est,
                 self.drone_yaw_est, measurement_valid) = self.get_pose_est()
                self.hover_setpoint.position = self.drone_pose_est.position
                print 'ctrl - end of if state changed'

            if not self.state == "initialization":
                self.hover()
            self.rate.sleep()

    def switch_state(self, state):
        '''Switches state according to the general fsm state as received by
        core.
        '''
        print 'received new state', state
        if not (state.data == self.state):
            self.state = state.data
            self.state_changed = True
            # If new state received before old one is finished,
            # kill current state.
            if self.executing_state:
                self.state_killed = True
        else:
            print yellow(' Controller already in the correct state!')

        print self.state, state.data, self.state_changed

        # When going to standby, remove markers in Rviz from previous task.
        # GEBEURT TOCH NOOIT?
        if state.data == "standby":
            self.reset_traj_markers()

    def hover(self, vel_desired=Twist()):
        '''Drone keeps itself in same location through a PID controller.
        '''
        if self.airborne:
            (self.drone_pose_est, self.drone_vel_est, self.drone_yaw_est,
                measurement_valid) = self.get_pose_est()

            if not measurement_valid:
                self.safety_brake()
                return
            pos_desired = PointStamped()
            pos_desired.point = Point(x=self.hover_setpoint.position.x,
                                      y=self.hover_setpoint.position.y,
                                      z=self.hover_setpoint.position.z)

            fb_cmd = self.feedbeck(pos_desired, vel_desired)

            self.send_input(fb_cmd)

    def take_off_land(self):
        '''Take off or land.
        '''
        self.full_cmd.twist = Twist()

        if self.state == "take-off":
            try:
                takeoff = rospy.ServiceProxy(
                    "/dji_sdk/drone_task_control", DroneTaskControl)
                takeoff_success = takeoff(task=4)
            except rospy.ServiceException, e:
                print highlight_red('Takeoff service call failed: %s') % e
                takeoff_success = False

        elif self.state == "land":
            self.reset_pid_gains()
            try:
                land = rospy.ServiceProxy(
                    "/dji_sdk/drone_task_control", DroneTaskControl)
                land_success = land(task=6)
            except rospy.ServiceException, e:
                print highlight_red('Land service call failed: %s') % e
                land_success = False
        rospy.sleep(10.)

    def build_traj(self):
        '''Build and pre-process a trajectory.
        '''
        self.reset_traj_markers()
        print yellow('---- Creating path ----')

        if self.state == "build fast trajectory":
            self.max_vel = rospy.get_param('motionplanner/vmax_fast', 0.5)
        else:
            self.max_vel = rospy.get_param('motionplanner/vmax_low', 0.5)

        # CREATE HARDCODED TRAJECTORY HERE!
        v_xy = 10.
        v_z = 2.
        A = 10
        w = 0.5
        start_pos = self.drone_pose_est.position

        t = [i/50. for i in range(0, 5000)]
        self.drawn_pos_x = [start_pos.x + A*np.sin(w*ti) for ti in t]
        self.drawn_pos_y = [start_pos.y + A*np.cos(w*ti) for ti in t]
        self.drawn_pos_z = [start_pos.z + v_z*ti for ti in t]

        # Show the new trajectory in rviz.
        self.draw_ctrl_path()

        # Clip positions to make sure path does not lie outside room.
        # self.drawn_pos_x = [
        #             max(- (self.room_width/2. - self.drone_radius),
        #                 min((self.room_width/2. - self.drone_radius),
        #                 (elem))) for elem in self.drawn_pos_x]
        # self.drawn_pos_y = [
        #             max(- (self.room_depth/2. - self.drone_radius),
        #                 min((self.room_depth/2. - self.drone_radius),
        #                 (elem))) for elem in self.drawn_pos_y]
        # self.drawn_pos_z = [
        #             max((self.drone_radius * 2.5),
        #                 min((self.room_height - self.drone_radius),
        #                 (elem))) for elem in self.drawn_pos_z]

        # Add padding to path for filtering purposes
        padding = 10
        self.drawn_pos_x = (
                        [self.drawn_pos_x[0] for i in range(padding)]
                        + self.drawn_pos_x +
                        [self.drawn_pos_x[-1] for i in range(padding)])
        self.drawn_pos_y = (
                        [self.drawn_pos_y[0] for i in range(padding)]
                        + self.drawn_pos_y +
                        [self.drawn_pos_y[-1] for i in range(padding)])
        self.drawn_pos_z = (
                        [self.drawn_pos_z[0] for i in range(padding)]
                        + self.drawn_pos_z +
                        [self.drawn_pos_z[-1] for i in range(padding)])

        # Process the drawn trajectory so the drone is able to follow
        # this path.
        if len(self.drawn_pos_x) > (50 + padding*2):
            self.diff_interp_traj()
            self.low_pass_filter_drawn_traj()
            self.differentiate_traj()
            (self.drawn_vel_filt_x,
             self.drawn_vel_filt_y,
             self.drawn_vel_filt_z) = (
                                    self.pad_lpf(self.drawn_vel_x[:],
                                                 self.drawn_vel_y[:],
                                                 self.drawn_vel_z[:]))
            padding = 50
            self.drawn_pos_x, self.drawn_pos_y, self.drawn_pos_z = (
                                    self.pad_in_front(self.drawn_pos_x,
                                                      self.drawn_pos_y,
                                                      self.drawn_pos_z,
                                                      padding))
            self.drawn_vel_x, self.drawn_vel_y, self.drawn_vel_z = (
                                    self.pad_in_front(self.drawn_vel_x,
                                                      self.drawn_vel_y,
                                                      self.drawn_vel_z,
                                                      padding, True))
        else:
            print highlight_red(
                            ' Path too short, draw a longer path! ')

    def follow_traj(self):
        '''Lets the drone fly along the drawn path.
        '''
        # If no path drawn, do nothing.
        if not len(self.drawn_pos_x):
            return

        # Reset omg path markers in Rviz.
        self._desired_path.points = []
        self.trajectory_desired.publish(self._desired_path)
        self._real_path.points = []
        self.trajectory_real.publish(self._real_path)
        self.current_ff_vel.points = [Point(), Point()]
        self.current_ff_vel_pub.publish(self.current_ff_vel)
        self.vhat_vector.points = [Point(), Point()]
        self.vhat_vector_pub.publish(self.vhat_vector)

        self.set_ff_pid_gains()
        # Reset ff model
        self.X = np.array(
                    [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

        # Preparing hover setpoint for when trajectory is completed.
        self._goal = Pose()
        self._goal.position.x = self.drawn_pos_x[-1]
        self._goal.position.y = self.drawn_pos_y[-1]
        self._goal.position.z = self.drawn_pos_z[-1]

        self.hover_setpoint = self._goal
        self.target_reached = False

        self.full_cmd.twist = Twist()
        self.full_cmd.twist.linear.x = self.drawn_vel_x[0]
        self.full_cmd.twist.linear.y = self.drawn_vel_y[0]
        self.full_cmd.twist.linear.z = self.drawn_vel_z[0]

        index = 1
        while (not self.target_reached and (index < len(self.drawn_vel_filt_x))
               and (not rospy.is_shutdown())):
            if self.state_killed:
                break

            self.traj_update(index)
            index += 1
            # Determine whether goal has been reached.
            if ((len(self.drawn_vel_filt_x) - index) < 100):
                self.check_goal_reached()

            self.rate.sleep()
        self.reset_pid_gains()

    def traj_update(self, index):
        '''
        - Updates the controller with newly calculated trajectories and
        velocity commands.
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        '''
        # Send velocity sample.
        self.full_cmd.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self.full_cmd.twist)

        # Retrieve new pose estimate from World Model.
        # This is a pose estimate for the first following time instance [k+1]
        # if the velocity command sent above corresponds to time instance [k].
        (self.drone_pose_est, self.drone_vel_est, self.drone_yaw_est,
            measurement_valid) = self.get_pose_est()
        self.publish_vhat_vector(self.drone_pose_est.position,
                                 self.drone_vel_est)

        # Publish pose to plot in rviz.
        self.publish_real(self.drone_pose_est.position.x,
                          self.drone_pose_est.position.y,
                          self.drone_pose_est.position.z)

        if not measurement_valid:
            self.safety_brake()
            return

        # publish current pose and velocity calculated by omg-tools
        pos = PointStamped()
        pos.header.frame_id = "world"
        pos.point = Point(x=self.drawn_pos_x[index],
                          y=self.drawn_pos_y[index],
                          z=self.drawn_pos_z[index])
        vel = TwistStamped()
        vel.header.frame_id = "world"
        vel.twist.linear.x = self.drawn_vel_filt_x[index]
        vel.twist.linear.y = self.drawn_vel_filt_y[index]
        vel.twist.linear.z = self.drawn_vel_filt_z[index]

        self.publish_current_ff_vel(pos, vel)

        # Calculate the desired yaw angle based on the pointing direction of
        # the resulting feedforward velocity vector.
        # self.desired_yaw = np.arctan2(vel.point.y, vel.point.x)

        # Transform feedforward command from frame world to world_rotated.
        self.rotate_vel_cmd(vel)

        # Convert feedforward velocity command to angle input.
        self.convert_vel_cmd()

        # Combine feedback and feedforward commands.
        vel = TwistStamped()
        vel.header.frame_id = "world"
        vel.twist.linear.x = self.drawn_vel_x[index]
        vel.twist.linear.y = self.drawn_vel_y[index]
        vel.twist.linear.z = self.drawn_vel_z[index]
        self.combine_ff_fb(pos, vel)

    def set_ff_pid_gains(self):
        '''Sets pid gains to a lower setting for combination with feedforward
        flight to keep the controller stable.
        '''
        if self.state in {"follow path"}:
            self.Kp_x = rospy.get_param('controller/Kp_tt_x', 0.6864)
            self.Ki_x = rospy.get_param('controller/Ki_tt_x', 0.6864)
            self.Kd_x = rospy.get_param('controller/Kd_tt_x', 0.6864)
            self.Kp_y = rospy.get_param('controller/Kp_tt_y', 0.6864)
            self.Ki_y = rospy.get_param('controller/Ki_tt_y', 0.6864)
            self.Kd_y = rospy.get_param('controller/Kd_tt_y', 0.6864)
            self.Kp_z = rospy.get_param('controller/Kp_tt_z', 0.5)
            self.Ki_z = rospy.get_param('controller/Ki_tt_z', 1.5792)
            self.Kd_z = rospy.get_param('controller/Kd_tt_z', 1.5792)
            self.Kp_yaw = rospy.get_param('controller/Kp_tt_yaw', 0.5)
            self.Ki_yaw = rospy.get_param('controller/Ki_tt_yaw', 1.5792)
            self.Kd_yaw = rospy.get_param('controller/Kd_tt_yaw', 1.5792)

    def reset_pid_gains(self):
        '''Resets the PID gains to the rosparam vaules after tasks "undamped
        spring" or "viscous fluid".
        '''
        self.Kp_x = rospy.get_param('controller/Kp_x', 0.6864)
        self.Ki_x = rospy.get_param('controller/Ki_x', 0.6864)
        self.Kd_x = rospy.get_param('controller/Kd_x', 0.6864)
        self.Kp_y = rospy.get_param('controller/Kp_y', 0.6864)
        self.Ki_y = rospy.get_param('controller/Ki_y', 0.6864)
        self.Kd_y = rospy.get_param('controller/Kd_y', 0.6864)
        self.Kp_z = rospy.get_param('controller/Kp_z', 0.5)
        self.Ki_z = rospy.get_param('controller/Ki_z', 1.5792)
        self.Kd_z = rospy.get_param('controller/Kd_z', 1.5792)
        self.Kp_yaw = rospy.get_param('controller/Kp_yaw', 0.5)
        self.Ki_yaw = rospy.get_param('controller/Ki_yaw', 1.5792)
        self.Kd_yaw = rospy.get_param('controller/Kd_yaw', 1.5792)

    def check_goal_reached(self):
        '''Determines whether goal is reached.
        Returns:
            not stop: boolean whether goal is reached. If not, controller
                      proceeds to goal.
        '''
        pos_nrm = self.position_diff_norm(self.drone_pose_est.position,
                                          self._goal.position)

        self.target_reached = (pos_nrm < self.pos_nrm_tol)
        if self.target_reached:
            # io.savemat('../mpc_calc_time.mat', self.calc_time)
            print yellow('=========================')
            print yellow('==== Target Reached! ====')
            print yellow('=========================')

    def safety_brake(self):
        '''Brake as emergency measure: Bebop brakes automatically when
            /bebop/cmd_vel topic receives all zeros.
        '''
        self.full_cmd.twist = Twist()
        self.send_input(self.full_cmd.twist)

    def repeat_safety_brake(self):
        '''More permanent emergency measure: keep safety braking until new task
        (eg. land) is given.
        '''
        while not (rospy.is_shutdown() or self.state_killed):
            self.safety_brake()
            self.rate.sleep()

    ####################
    # Helper functions #
    ####################

    def send_input(self, input_cmd):
        '''Publish input command both as a Twist() (old bebop style) and as a
        sensor_msgs/Joy message for DJI drone.

        input_cmd: Twist()
        '''
        self.full_cmd.twist = input_cmd
        self.full_cmd.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self.full_cmd.twist)

        flag = np.uint8(self.VERTICAL_VEL |
                        self.HORIZONTAL_ANGLE |
                        self.YAW_RATE |
                        self.HORIZONTAL_BODY |
                        self.STABLE_ENABLE)

        full_cmd_dji = Joy()
        full_cmd_dji.header = self.full_cmd.header
        full_cmd_dji.axes = [-input_cmd.linear.y,
                             input_cmd.linear.x,
                             input_cmd.linear.z,
                             input_cmd.angular.z,
                             flag]

        self.cmd_vel_dji.publish(full_cmd_dji)

    def rotate_vel_cmd(self, vel):
        '''Transforms the velocity commands from the global world frame to the
        rotated world frame world_rot.
        '''
        self.ff_velocity = self.transform_twist(vel, "world", "world_rot")

    def convert_vel_cmd(self):
        '''Converts a velocity command to a desired input angle according to
        the state space representation of the inverse velocity model.
        '''
        u = np.array([[self.ff_velocity.twist.linear.x],
                      [self.ff_velocity.twist.linear.y],
                      [self.ff_velocity.twist.linear.z]])

        Y = np.matmul(self.C, self.X) + np.matmul(self.D, u)
        self.X = np.matmul(self.A, self.X) + np.matmul(self.B, u)
        self.ff_cmd.linear.x = Y[0, 0]
        self.ff_cmd.linear.y = Y[1, 0]
        self.ff_cmd.linear.z = Y[2, 0]

    def combine_ff_fb(self, pos_desired, vel_desired):
        '''Combines the feedforward and feedback commands to generate the full
        input angle command.
        '''
        # Transform feedback desired position and velocity from world frame to
        # world_rot frame
        fb_cmd = self.feedbeck(pos_desired, vel_desired.twist)

        if self.state == 'follow path':
            self.full_cmd.twist.linear.x = max(min((
                    self.ff_cmd.linear.x + fb_cmd.linear.x),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.linear.y = max(min((
                    self.ff_cmd.linear.y + fb_cmd.linear.y),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.linear.z = max(min((
                    self.ff_cmd.linear.z + fb_cmd.linear.z),
                    self.max_z_input), - self.max_z_input)
            self.full_cmd.twist.angular.z = max(min((
                    self.ff_cmd.angular.z + fb_cmd.angular.z),
                    self.max_yaw_input), - self.max_yaw_input)
        else:
            self.full_cmd.twist.linear.x = max(min((
                    fb_cmd.linear.x),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.linear.y = max(min((
                    fb_cmd.linear.y),
                    self.max_input), - self.max_input)
            self.full_cmd.twist.linear.z = max(min((
                    fb_cmd.linear.z),
                    self.max_z_input), - self.max_z_input)
            self.full_cmd.twist.angular.z = max(min((
                    fb_cmd.angular.z),
                    self.max_yaw_input), - self.max_yaw_input)

    def feedbeck(self, pos_desired, vel_desired, yaw_desired=0.):
        '''Whenever the target is reached, apply position feedback to the
        desired end position to remain in the correct spot and compensate for
        drift.
        Tustin discretized PID controller for x and y, PI for z.
        '''
        fb_cmd = Twist()

        # PID
        pos_error_prev = self.pos_error_prev
        pos_error = PointStamped()
        pos_error.header.frame_id = "world"
        pos_error.point.x = (pos_desired.point.x
                             - self.drone_pose_est.position.x)
        pos_error.point.y = (pos_desired.point.y
                             - self.drone_pose_est.position.y)
        pos_error.point.z = (pos_desired.point.z
                             - self.drone_pose_est.position.z)

        vel_error_prev = self.vel_error_prev
        vel_error = PointStamped()
        vel_error.header.frame_id = "world"
        vel_error.point.x = vel_desired.linear.x - self.drone_vel_est.x
        vel_error.point.y = vel_desired.linear.y - self.drone_vel_est.y
        vel_error.point.z = vel_desired.linear.z - self.drone_vel_est.z

        pos_error = self.transform_point(pos_error, "world", "world_rot")
        vel_error = self.transform_point(vel_error, "world", "world_rot")

        yaw_error_prev = self.yaw_error_prev
        yaw_error = ((((yaw_desired - self.drone_yaw_est) -
                       np.pi) % (2*np.pi)) - np.pi)

        fb_cmd.linear.x = max(- self.max_input, min(self.max_input, (
                self.fb_cmd_prev.linear.x +
                (self.Kp_x + self.Ki_x*self._sample_time/2) *
                pos_error.point.x +
                (-self.Kp_x + self.Ki_x*self._sample_time/2) *
                pos_error_prev.point.x +
                self.Kd_x*(vel_error.point.x - vel_error_prev.point.x))))

        fb_cmd.linear.y = max(- self.max_input, min(self.max_input, (
                self.fb_cmd_prev.linear.y +
                (self.Kp_y + self.Ki_y*self._sample_time/2) *
                pos_error.point.y +
                (-self.Kp_y + self.Ki_y*self._sample_time/2) *
                pos_error_prev.point.y +
                self.Kd_y*(vel_error.point.y - vel_error_prev.point.y))))

        fb_cmd.linear.z = max(- self.max_input, min(self.max_input, (
                self.fb_cmd_prev.linear.z +
                (self.Kp_z + self.Ki_z*self._sample_time/2) *
                pos_error.point.z +
                (-self.Kp_z + self.Ki_z*self._sample_time/2) *
                pos_error_prev.point.z +
                self.Kd_z*(vel_error.point.z - vel_error_prev.point.z))))

        # UNDER CONSTRUCTION - TODO: yaw velocity estimator
        # fb_cmd.angular.z = max(- self.max_input, min(self.max_input, (
        #         self.fb_cmd_prev.linear.z +
        #         (self.Kp_z + self.Ki_z*self._sample_time/2) *
        #         pos_error.point.z +
        #         (-self.Kp_z + self.Ki_z*self._sample_time/2) *
        #         pos_error_prev.point.z +
        #         self.Kd_z*(vel_error.point.z - vel_error_prev.point.z))))

        self.pos_error_prev = pos_error
        self.vel_error_prev = vel_error
        self.yaw_error_prev = yaw_error
        self.fb_cmd_prev = fb_cmd

        # Publish the position error.
        # self.pos_error_pub.publish(pos_error)

        return fb_cmd

    def get_pose_est(self):
        '''Retrieves a new pose estimate from world model.
        '''
        # This service is provided as soon as localization is ready.
        rospy.wait_for_service(
            "/world_model/get_pose", timeout=self.service_timeout)
        try:
            pose_est = rospy.ServiceProxy(
                "/world_model/get_pose", GetPoseEst)
            resp = pose_est(self.full_cmd)

            yhat = resp.pose_est.point
            pose = Pose()
            pose.position.x = yhat.x
            pose.position.y = yhat.y
            pose.position.z = yhat.z

            vhat = resp.vel_est.point
            yaw = resp.yaw
            measurement_valid = resp.measurement_valid

            return pose, vhat, yaw, measurement_valid

        except rospy.ServiceException, e:
            print highlight_red('Service call failed: %s') % e
            return

    def get_flight_status(self, flight_status):
        '''Checks whether the drone is standing on the ground or flying and
        changes the self.airborne variable accordingly.
        '''
        if flight_status.data == 1:
            self.airborne = False
        elif flight_status.data == 3:
            self.airborne = True

    def diff_interp_traj(self):
        '''Differentiate and interpolate obtained trajectory to obtain
        feedforward velocity commands.
        '''
        self.differentiate_traj()

        # Search for the highest velocity in the trajectory to determine the
        # step size needed for interpolation.
        highest_vel = max(max(self.drawn_vel_x),
                          max(self.drawn_vel_y),
                          max(self.drawn_vel_z))
        self.interpolate_drawn_traj(self.max_vel/highest_vel)

    def differentiate_traj(self):
        '''Numerically differentiates position traject to recover a list of
        feedforward velocities.
        '''
        self.drawn_vel_x = (
                        np.diff(self.drawn_pos_x)/self._sample_time).tolist()
        self.drawn_vel_y = (
                        np.diff(self.drawn_pos_y)/self._sample_time).tolist()
        self.drawn_vel_z = (
                        np.diff(self.drawn_pos_z)/self._sample_time).tolist()

    def interpolate_drawn_traj(self, step):
        '''Linearly interpolates a list so that it contains the desired amount
        of elements where the element distance is equal to step.
        '''
        self.drawn_pos_x = np.interp(np.arange(0, len(self.drawn_pos_x), step),
                                     range(len(self.drawn_pos_x)),
                                     self.drawn_pos_x).tolist()
        self.drawn_pos_y = np.interp(np.arange(0, len(self.drawn_pos_y), step),
                                     range(len(self.drawn_pos_y)),
                                     self.drawn_pos_y).tolist()
        self.drawn_pos_z = np.interp(np.arange(0, len(self.drawn_pos_z), step),
                                     range(len(self.drawn_pos_z)),
                                     self.drawn_pos_z).tolist()

    def low_pass_filter_drawn_traj(self):
        '''Low pass filter the trajectory drawn with the controller in order to
        be suitable for the drone to track it.
        '''
        self.drawn_pos_x = filtfilt(
            self.butter_b, self.butter_a, self.drawn_pos_x, padlen=50).tolist()
        self.drawn_pos_y = filtfilt(
            self.butter_b, self.butter_a, self.drawn_pos_y, padlen=50).tolist()
        self.drawn_pos_z = filtfilt(
            self.butter_b, self.butter_a, self.drawn_pos_z, padlen=50).tolist()

        # Plot the smoothed trajectory in Rviz.
        self.draw_smoothed_path()

    def position_diff_norm(self, point1, point2):
        '''Returns the norm of the difference vector between two given points.
        point1 and point2 are geometry_msgs/Point objects.
        '''
        norm = np.linalg.norm(np.array([point1.x, point1.y, point1.z])
                              - np.array([point2.x, point2.y, point2.z]))
        return norm

    def pad_lpf(self, x_vec, y_vec, z_vec):
        '''Adds padding based on derivative of curve at the end, since padding
        will be removed due to phase shift of low pass filter which is applied
        after padding.
        '''
        # Reverse vector.
        x_vec.reverse()
        y_vec.reverse()
        z_vec.reverse()
        # Add padding and filter.
        dx = (3./2.*x_vec[-1] - 2.*x_vec[-2] + 1./2.*x_vec[-3])
        dy = (3./2.*y_vec[-1] - 2.*y_vec[-2] + 1./2.*y_vec[-3])
        dz = (3./2.*z_vec[-1] - 2.*z_vec[-2] + 1./2.*z_vec[-3])

        padlen = 50
        x_pad = [x_vec[-1] + (np.arange(dx, dx*(padlen + 1), dx).tolist())[i]
                 for i in range(padlen)]
        y_pad = [y_vec[-1] + (np.arange(dy, dy*(padlen + 1), dy).tolist())[i]
                 for i in range(padlen)]
        z_pad = [z_vec[-1] + (np.arange(dz, dz*(padlen + 1), dz).tolist())[i]
                 for i in range(padlen)]

        x_vec = lfilter(self.butter_b, self.butter_a, x_vec + x_pad).tolist()
        y_vec = lfilter(self.butter_b, self.butter_a, y_vec + y_pad).tolist()
        z_vec = lfilter(self.butter_b, self.butter_a, z_vec + z_pad).tolist()

        # Again reverse vector.
        x_vec.reverse()
        y_vec.reverse()
        z_vec.reverse()
        # Cutoff last part since this is created due to lpf
        x_vec = x_vec[: -padlen]
        y_vec = y_vec[: -padlen]
        z_vec = z_vec[: -padlen]

        return x_vec, y_vec, z_vec

    def pad_in_front(self, x_vec, y_vec, z_vec, pad, zeros=False):
        '''Adds padding of len pad in front of vectors.
        '''
        if zeros:
            x_vec = [0. for i in range(pad)] + x_vec
            y_vec = [0. for i in range(pad)] + y_vec
            z_vec = [0. for i in range(pad)] + z_vec
        else:
            x_vec = [x_vec[0] for i in range(pad)] + x_vec
            y_vec = [y_vec[0] for i in range(pad)] + y_vec
            z_vec = [z_vec[0] for i in range(pad)] + z_vec

        return x_vec, y_vec, z_vec

    def transform_twist(self, twist, _from, _to):
        '''Transforms twist (geometry_msgs/Twist) from frame "_from" to
        frame "_to".
        Arguments:
            - _from, _to = string, name of frame
        '''
        cmd_vel = PointStamped()
        cmd_vel.header = twist.header
        cmd_vel.point = twist.twist.linear
        cmd_vel_rotated = self.transform_point(cmd_vel, _from, _to)

        twist_rotated = TwistStamped()
        twist_rotated.header.stamp = twist.header.stamp
        twist_rotated.twist.linear = cmd_vel_rotated.point

        return twist_rotated

    def transform_point(self, point, _from, _to):
        '''Transforms point from _from frame to _to frame.
        '''
        transform = self.tfBuffer.lookup_transform(
            _to, _from, rospy.Time(0), rospy.Duration(0.1))
        point_transformed = tf2_geom.do_transform_point(point, transform)

        return point_transformed

    #######################################
    # Functions for plotting Rviz markers #
    #######################################

    def _marker_setup(self):
        '''Setup markers to display the desired and real path of the drone in
        rviz, along with the current position in the omg-tools generated
        position list.
        '''

        # Real path
        self._real_path = Marker()
        self._real_path.header.frame_id = 'world'
        self._real_path.ns = "trajectory_real"
        self._real_path.id = 1
        self._real_path.type = 4  # Line List.
        self._real_path.action = 0
        self._real_path.scale.x = 0.03
        self._real_path.color.r = 0.0
        self._real_path.color.g = 1.0
        self._real_path.color.b = 0.0
        self._real_path.color.a = 1.0
        self._real_path.lifetime = rospy.Duration(0)

        # Feedforward position and velocity
        self.current_ff_vel = Marker()
        self.current_ff_vel.header.frame_id = 'world'
        self.current_ff_vel.ns = "current_ff_vel"
        self.current_ff_vel.id = 2
        self.current_ff_vel.type = 0  # Arrow
        self.current_ff_vel.action = 0
        self.current_ff_vel.scale.x = 0.06  # shaft diameter
        self.current_ff_vel.scale.y = 0.1  # head diameter
        self.current_ff_vel.scale.z = 0.15  # head length
        self.current_ff_vel.color.r = 0.0
        self.current_ff_vel.color.g = 0.0
        self.current_ff_vel.color.b = 1.0
        self.current_ff_vel.color.a = 1.0
        self.current_ff_vel.lifetime = rospy.Duration(0)

        # Controller drawn path
        self.drawn_path = Marker()
        self.drawn_path.header.frame_id = 'world'
        self.drawn_path.ns = "drawn_path"
        self.drawn_path.id = 4
        self.drawn_path.type = 4  # Line List.
        self.drawn_path.action = 0
        self.drawn_path.scale.x = 0.03
        self.drawn_path.color.r = 1.0
        self.drawn_path.color.g = 0.86
        self.drawn_path.color.b = 0.0
        self.drawn_path.color.a = 1.0
        self.drawn_path.lifetime = rospy.Duration(0)

        # Smoother version of the path
        self.smooth_path = Marker()
        self.smooth_path.header.frame_id = 'world'
        self.smooth_path.ns = "smooth_path"
        self.smooth_path.id = 5
        self.smooth_path.type = 4  # Line List.
        self.smooth_path.action = 0
        self.smooth_path.scale.x = 0.03
        self.smooth_path.color.r = 1.0
        self.smooth_path.color.g = 0.38
        self.smooth_path.color.b = 0.0
        self.smooth_path.color.a = 1.0
        self.smooth_path.lifetime = rospy.Duration(0)

        # Room contours
        self.room_contours = Marker()
        self.room_contours.header.frame_id = 'world'
        self.room_contours.ns = "room_contours"
        self.room_contours.id = 6
        self.room_contours.type = 4  # Line List.
        self.room_contours.action = 0
        self.room_contours.scale.x = 0.03
        self.room_contours.color.r = 0.8
        self.room_contours.color.g = 0.8
        self.room_contours.color.b = 0.8
        self.room_contours.color.a = 1.0
        self.room_contours.lifetime = rospy.Duration(0)

        # Vhat vector
        self.vhat_vector = Marker()
        self.vhat_vector.header.frame_id = 'world'
        self.vhat_vector.ns = "vhat_vector"
        self.vhat_vector.id = 7
        self.vhat_vector.type = 0  # Arrow.
        self.vhat_vector.action = 0
        self.vhat_vector.scale.x = 0.06  # shaft diameter
        self.vhat_vector.scale.y = 0.1  # head diameter
        self.vhat_vector.scale.z = 0.15  # head length
        self.vhat_vector.color.r = 1.0
        self.vhat_vector.color.g = 1.0
        self.vhat_vector.color.b = 0.3
        self.vhat_vector.color.a = 1.0
        self.vhat_vector.lifetime = rospy.Duration(0)

    def reset_traj_markers(self):
        '''Resets all Rviz markers (except for obstacles).
        '''
        self.drawn_path.points = []
        self.trajectory_drawn.publish(self.drawn_path)
        self._real_path.points = []
        self.trajectory_real.publish(self._real_path)
        self.smooth_path.points = []
        self.trajectory_smoothed.publish(self.smooth_path)
        self.current_ff_vel.points = [Point(), Point()]
        self.current_ff_vel_pub.publish(self.current_ff_vel)
        self.vhat_vector.points = [Point(), Point()]
        self.vhat_vector_pub.publish(self.vhat_vector)

    def publish_real(self, x_pos, y_pos, z_pos):
        '''Publish real x and y trajectory to topic for visualisation in
        rviz.
        '''
        self._real_path.header.stamp = rospy.get_rostime()

        point = Point(x=x_pos, y=y_pos, z=z_pos)
        # After a while list becomes really long so only keep last XXXX values.
        if len(self._real_path.points) > 1000:
            self._real_path.points = self._real_path.points[1:] + [point]
        else:
            self._real_path.points.append(point)

        self.trajectory_real.publish(self._real_path)

    def publish_current_ff_vel(self, pos, vel):
        '''Publish current velocity reference vector where origin of the
        vector is equal to current reference position.
        '''
        self.current_ff_vel.header.stamp = rospy.get_rostime()

        point_start = Point(x=pos.point.x, y=pos.point.y, z=pos.point.z)
        point_end = Point(x=(pos.point.x + vel.twist.linear.x),
                          y=(pos.point.y + vel.twist.linear.y),
                          z=(pos.point.z + vel.twist.linear.z))
        self.current_ff_vel.points = [point_start, point_end]

        self.current_ff_vel_pub.publish(self.current_ff_vel)

    def publish_vhat_vector(self, pos, vel):
        '''Publish current vhat estimate from the kalman filter as a vector
        with origin equal to the current position estimate.
        '''
        self.vhat_vector.header.stamp = rospy.get_rostime()

        point_start = Point(x=pos.x, y=pos.y, z=pos.z)
        point_end = Point(x=(pos.x + vel.x),
                          y=(pos.y + vel.y),
                          z=(pos.z + vel.z))
        self.vhat_vector.points = [point_start, point_end]

        self.vhat_vector_pub.publish(self.vhat_vector)

    def publish_obst_room(self, empty):
        '''Publish static obstacles as well as the boundary of the room.
        '''
        # Delete markers
        # marker = Marker()
        # marker.ns = "obstacles"
        # marker.action = 3 # 3 deletes markers
        # self.rviz_obst.markers = [marker]
        # self.obst_pub.publish(self.rviz_obst)

        self.reset_traj_markers()
        # self.obst_pub.publish(self.rviz_obst)
        self.draw_room_contours()

    def draw_ctrl_path(self):
        '''Publish real x and y trajectory to topic for visualisation in
        rviz.
        '''
        self.drawn_path.header.stamp = rospy.get_rostime()
        for i in range(0, len(self.drawn_pos_x)+1):
            point = Point(x=self.drawn_pos_x[i],
                          y=self.drawn_pos_y[i],
                          z=self.drawn_pos_z[i])
        self.drawn_path.points.append(point)

        self.trajectory_drawn.publish(self.drawn_path)

    def draw_smoothed_path(self):
        '''Publish the smoothed x and y trajectory to topic for visualisation
        in rviz.
        '''
        self.smooth_path.header.stamp = rospy.get_rostime()
        self.smooth_path.points = []

        for index in range(len(self.drawn_pos_x)):
            point = Point(x=self.drawn_pos_x[index],
                          y=self.drawn_pos_y[index],
                          z=self.drawn_pos_z[index])
            self.smooth_path.points.append(point)

        self.trajectory_smoothed.publish(self.smooth_path)

    def draw_room_contours(self):
        '''Publish the edges of the room for visualization in rviz.
        '''
        self.room_contours.header.stamp = rospy.get_rostime()

        bottom_left = Point(x=-self.room_width/2.,
                            y=-self.room_depth/2.)
        bottom_right = Point(x=self.room_width/2.,
                             y=-self.room_depth/2.)
        top_right = Point(x=self.room_width/2.,
                          y=self.room_depth/2.)
        top_left = Point(x=-self.room_width/2.,
                         y=self.room_depth/2.)
        corners = [bottom_left, bottom_right, top_right, top_left, bottom_left]
        for point in corners:
            self.room_contours.points.append(point)

        self.draw_room.publish(self.room_contours)


if __name__ == '__main__':
    controller = Controller()
    controller.start()
