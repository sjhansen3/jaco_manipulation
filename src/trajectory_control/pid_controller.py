#!/usr/bin/env python

"""
Adapted from Andrea Bajcsy
https://github.com/abajcsy/iact_control/
"""

from enum import Enum
import sys
import time

import numpy as np
from numpy import array
import roslib
import rospy
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import kinova_msgs.srv

import pid
import ros_utils

roslib.load_manifest('kinova_demo')

PREFIX = 'j2s7s300_driver'

EPSILON = 0.05
MAX_CMD_TORQUE = 40.0
INTERACTION_TORQUE_THRESHOLD = 8.0

MODE = Enum('Mode', 'hold executing stepping grav_comp')


class PIDController(object):
    """
    This class represents a node that moves the Jaco with PID control.
    The joint velocities are computed as:

        V = -K_p(e) - K_d(e_dot) - K_i*Integral(e)
    where:
        e = (target_joint configuration) - (current joint configuration)
        e_dot = derivative of error
        K_p = accounts for present values of position error
        K_i = accounts for past values of error, accumulates error over time
        K_d = accounts for possible future trends of error, based on current rate of change

    Subscribes to:
        /j2s7s300_driver/out/joint_angles	- Jaco sensed joint angles
        /j2s7s300_driver/out/joint_torques	- Jaco sensed joint torques

    Publishes to:
        /j2s7s300_driver/in/joint_velocity	- Jaco commanded joint velocities

    Required parameters:
        p_gain, i_gain, d_gain    - gain terms for the PID controller
        sim_flag 				  - flag for if in simulation or not
    """

    def __init__(self):
        """
        Setup of the ROS node. Publishing computed torques happens at 100Hz.
        """

        #self.robot = robot #OpenRAVE robot I think?
        self.mode = MODE.hold
        self.reached_start = False
        self.reached_goal = False
        self.last_dof = None
        self.target_index = 0
        self.step_size = 1
        self.time_points = None

        # ----- Controller Setup ----- #

        # stores maximum COMMANDED joint torques
        self.max_cmd = MAX_CMD_TORQUE * np.eye(7)
        # stores current COMMANDED joint torques
        self.cmd = np.eye(7)
        # stores current joint MEASURED joint torques
        self.joint_torques = np.zeros((7, 1))

        # P, I, D gains
        p_gain = 50.0
        i_gain = 0.00
        d_gain = 20.0
        self.P = p_gain * np.eye(7)
        self.I = i_gain * np.eye(7)
        self.D = d_gain * np.eye(7)
        self.controller = pid.PID(self.P, self.I, self.D, 0, 0)

        # ---- ROS Setup ---- #

        rospy.init_node("pid_trajopt", anonymous=True)

        # create joint-velocity publisher
        self.vel_pub = rospy.Publisher(PREFIX + '/in/joint_velocity',
                                       kinova_msgs.msg.JointVelocity,
                                       queue_size=1)

        # create subscriber to joint_angles
        rospy.Subscriber(PREFIX + '/out/joint_angles',
                         kinova_msgs.msg.JointAngles,
                         self.joint_angles_callback, queue_size=1)
        # create subscriber to joint_torques
        rospy.Subscriber(PREFIX + '/out/joint_torques',
                         kinova_msgs.msg.JointTorque,
                         self.joint_torques_callback, queue_size=1)

    def execute_loop(self):
        r = rospy.Rate(100)

        while not rospy.is_shutdown() and not (self.reached_goal and
                                               self.reached_start):
            #print "sending command", self.cmd
            self.vel_pub.publish(ros_utils.cmd_to_JointVelocityMsg(self.cmd))
            r.sleep()

    def execute_trajectory(self, traj, duration=10.):

        #self.start_admittance_mode()
        

        trajectory = traj #self.fix_joint_angles(traj)
        self.trajectory = traj

        print "self.trajectory", self.trajectory

        #TODO using this function garuntees that distance corresponds to time,
        # I should probably use the time points provided by moveit.
        # time_points will work with any list like structure. 1XN numpy array where
        # N is the number of time points
        self.time_points = self.time_trajectory(trajectory, duration)
        print "self.time_points", self.time_points

        # ---- Trajectory Setup ---- #

        # total time for trajectory
        self.trajectory_time = duration

        self.start = trajectory[0].reshape((7, 1))
        self.goal = trajectory[-1].reshape((7, 1))
        print "self.start", self.start
        print "self.goal", self.goal

        self.target_pos = trajectory[0].reshape((7, 1))
        self.target_index = 0

        # track if you have gotten to start/goal of path
        self.reached_start = False
        self.reached_goal = False
        self.mode = MODE.executing

        # keeps running time since beginning of path
        self.path_start_T = time.time()

        self.execute_loop()

        print "******************************************** END Admittance control"
        # end admittance control mode
        self.stop_admittance_mode()
        self.executing = False

    @classmethod
    def time_trajectory(cls, trajectory, total_time):
        """
        Spaces out time points based on l2 distance traveled between
        configurations.

        Parameters
        ----------
        trajectory : array
        total_time : float

        Returns
        -------
        Array of time stamps
        """
        total_distance = 0.
        distances = np.zeros((len(trajectory)))
        for i in range(len(trajectory) - 1):
            dist = np.linalg.norm(trajectory[i + 1] - trajectory[i])
            total_distance += dist
            distances[i + 1] = dist
        time_points = np.cumsum(distances) * total_time / total_distance
        return time_points

    def step_trajectory(self, traj, starting_index=0, step_size=1):
        self.start_stepping(traj, starting_index)
        self.step_size = step_size
        while True:
            cmd = raw_input().lower()
            if 'n' in cmd:
                self.step_next()
            if 'p' in cmd:
                self.step_prev()
            if 'q' in cmd:
                break
        self.stop_stepping()
        return self.target_index

    def start_stepping(self, traj, starting_index):
        trajectory = self.fix_joint_angles(traj)
        self.trajectory = trajectory
        self.target_index = starting_index
        self.target_pos = trajectory[self.target_index].reshape((7, 1))
        self.goal = self.target_pos
        self.reached_start = True
        self.reached_goal = False
        self.mode = MODE.stepping
        self.execute_loop()

    def stop_stepping(self):
        self.mode = MODE.hold

    def step_next(self):
        self.target_index += self.step_size
        if self.target_index > len(self.trajectory) - 1:
            self.target_index = len(self.trajectory) - 1
        self.target_pos = self.trajectory[self.target_index].reshape((7, 1))
        self.goal = self.target_pos
        self.reached_goal = False
        self.execute_loop()

    def step_prev(self):
        self.target_index -= self.step_size
        if self.target_index == 0:
            self.target_index = 0
        self.target_pos = self.trajectory[self.target_index].reshape((7, 1))
        self.goal = self.target_pos
        self.reached_goal = False
        self.execute_loop()

    def grav_comp(self):
        self.start_admittance_mode()
        self.mode = MODE.grav_comp
        raw_input("Press enter to exit gravity compensation mode.")
        self.mode = MODE.hold
        self.stop_admittance_mode()
        return self.last_dof

    def start_admittance_mode(self):
        """
        Switches Jaco to admittance-control mode using ROS services
        """

        service_address = PREFIX + '/in/start_force_control'
        rospy.wait_for_service(service_address)
        try:
            startForceControl = rospy.ServiceProxy(
                service_address,
                kinova_msgs.srv.Start
            )
            startForceControl()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None

    def stop_admittance_mode(self):
        """
        Switches Jaco to position-control mode using ROS services
        """

        service_address = PREFIX + '/in/stop_force_control'
        rospy.wait_for_service(service_address)
        try:
            stopForceControl = rospy.ServiceProxy(
                service_address,
                kinova_msgs.srv.Stop
            )
            stopForceControl()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None

    def PID_control(self, pos):
        """
        Return a control torque based on PID control
        """
        #TODO what should I do with the pi's here??
        error = PIDController.shortest_angular_distance(self.target_pos, pos)
        return -self.controller.update_PID(error)

    def joint_torques_callback(self, msg):
        """
        Reads the latest torque sensed by the robot and records it for
        plotting & analysis
        """
        return

    def joint_angles_callback(self, msg):
        """
        Reads the latest position of the robot and publishes an
        appropriate torque command to move the robot to the target
        """
        # read the current joint angles from the robot
        curr_pos = np.array(
            [msg.joint1, msg.joint2, msg.joint3, msg.joint4, msg.joint5,
             msg.joint6, msg.joint7]).reshape((7, 1))

        # convert to radians
        curr_pos = curr_pos * (np.pi / 180.0)

        # update the OpenRAVE simulation
        # self.planner.update_curr_pos(curr_pos)
        #self.update_robot(curr_pos) #TODO can I just delete this line?
        self.last_dof = curr_pos

        # update target position to move to depending on:
        # - if moving to START of desired trajectory or
        # - if moving ALONG desired trajectory
        self.update_target_pos(curr_pos)

        # update cmd from PID based on current position
        self.cmd = self.PID_control(curr_pos)
        #print "target pos: ", self.target_pos
        #print "robot commend: " , self.cmd

        #TODO change references to torque to velocity - I believe its just velocity
        # check if each angular torque is within set limits
        for i in range(7):
            if self.cmd[i][i] > self.max_cmd[i][i]:
                self.cmd[i][i] = self.max_cmd[i][i]
            if self.cmd[i][i] < -self.max_cmd[i][i]:
                self.cmd[i][i] = -self.max_cmd[i][i]

    def update_target_pos(self, curr_pos):
        """
        Takes the current position of the robot. Determines what the next
        target position to move to should be depending on:
        - if robot is moving to start of desired trajectory or
        - if robot is moving along the desired trajectory
        """

        if self.mode == MODE.hold or self.mode == MODE.grav_comp:
            self.target_pos = curr_pos
            return

        if self.mode == MODE.stepping:
            #TODO repeated pi code - can we centralize this?

            dist_from_goal = PIDController.shortest_angular_distance(curr_pos, self.goal)
            if np.all(np.abs(dist_from_goal) < EPSILON):
                self.reached_goal = True
            return

        if self.mode == MODE.executing:

            # check if the arm is at the start of the path to execute
            if not self.reached_start:
                print "not reached start"
                print "curr_pos", curr_pos, "self.goal", self.goal

                #dist_from_start = -((curr_pos - self.start + np.pi) %
                #                    (2 * np.pi) - np.pi)
                dist_from_start = PIDController.shortest_angular_distance(curr_pos, self.start)
                dist_from_start = np.abs(dist_from_start) #TODO this waas fabs but my version of numpy doeesnt have fabs
                print dist_from_start, "distance from start"
                # print "d to start: ", np.linalg.norm(dist_from_start)

                # if all joints are close enough, robot is at start
                is_at_start = np.all(dist_from_start < EPSILON)

                if is_at_start:
                    self.reached_start = True
                    self.path_start_T = time.time()
                else:
                    self.target_pos = self.start.reshape((7, 1))
            else:
                t = time.time() - self.path_start_T

                self.target_pos = self.interpolate_trajectory(t)

                if not self.reached_goal:

                    dist_from_goal = PIDController.shortest_angular_distance(curr_pos, self.goal)
                    if np.all(np.abs(dist_from_goal) < EPSILON):
                        self.reached_goal = True

                else:
                    self.stop_admittance_mode()

    def interpolate_trajectory(self, time):
        if time >= self.trajectory_time:
            #TODO is this the correct behavior? if time is greater than 
            target_pos = self.trajectory[-1]
        else:
            while self.time_points[self.target_index] < time:
                self.target_index += 1
            prev_t = self.time_points[self.target_index - 1]
            next_t = self.time_points[self.target_index]
            delta_t = next_t - prev_t

            prev_p = self.trajectory[self.target_index - 1]
            next_p = self.trajectory[self.target_index]
            delta_p = next_p - prev_p

            diff = delta_p * (time - prev_t) / delta_t
            target_pos = diff + prev_p

        return np.array(target_pos).reshape((7, 1))

    def fix_joint_angles(self, trajectory):
        #TODO is off by pi an issue with openRAVE or kinova? 
        # should I fix for kinova as well? TBD
        trajectory = trajectory.copy()
        for dof in trajectory:
            dof[2] -= np.pi
        return trajectory[:,:7]
    
    @staticmethod
    def shortest_angular_distance(angle1, angle2):
        return -((angle1 - angle2 + np.pi) % (2 * np.pi) - np.pi) 

def shortest_angular_distance_test():
    pi = np.pi
    tests = [[0, pi, pi], [0, pi/2, pi/2], [0, 2*pi, 0], [pi/2, 3*pi, pi/2]]
    for test in tests:
        res = test[2]
        test1_res = PIDController.shortest_angular_distance(test[0], test[1])
        test2_res = PIDController.shortest_angular_distance(test[1], test[0])
        print "res {}, test1_res, {}, test2_res {}".format(res, test1_res, test2_res)

if __name__ == '__main__':
    #argument = ' '.join(sys.argv[1:])
    controller = PIDController()
    home = np.asarray([180]*7)*(np.pi/180)
    pnt = np.asarray([80.363975525, 197.091796875, 179.857910156, 43.4620018005, -94.3617858887, 257.270996094, 287.989074707])*(np.pi/180)
    traj = np.vstack([home, pnt])
    print traj
    controller.execute_trajectory(traj)

