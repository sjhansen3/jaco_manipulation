#!/usr/bin/env python

import rospkg
import json
import time
import random
import math
import numpy as np
from abc import ABCMeta, abstractmethod
import trajoptpy
import openravepy
import rospy
import angles
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


class CostFunction:
    """ Base class for a cost function for Trajopt """

    __metaclass__ = ABCMeta

    def __init__(self, params):
        self.params = params

    @abstractmethod
    def get_cost(self, config):
        """ Returns the cost incurred at the specified configuration """
        pass


class JacoTrajopt:
    """ Interface to Trajopt planner and OpenRAVE """

    def __init__(self):
        self.env = openravepy.Environment()

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('interact_manipulation')
        jaco_urdf_path = package_path + '/config/jaco.urdf'
        jaco_srdf_path = package_path + '/config/jaco.srdf'
        rospy.loginfo("Loading Jaco URDF from {} and SRDF from {}...".format(jaco_urdf_path,
                                                                     jaco_srdf_path))

        self.urdf_module = openravepy.RaveCreateModule(self.env, 'urdf')
        name = self.urdf_module.SendCommand("load {} {}".format(jaco_urdf_path,
                                                                jaco_srdf_path))
        self.jaco = self.env.GetRobot(name)
        # self.finger_joint_values = [0.0, 0.0, 0.0]
        self.finger_joint_values = [1.0, 1.0, 1.0]
        self.joint_names = ['j2s7s300_joint_{}'.format(i) for i in range(1,8)]

        self.trajopt_num_waypoints = 30
        self.dt = 0.2 #time between waypoints to be added in post processing
        self.cost_functions = []

    def load_body_from_urdf(self, path_to_urdf, transform=np.eye(4, 4)):
        """ Load a body (non-robot object) from a URDF file into the OpenRAVE environment """
        name = self.urdf_module.SendCommand("load {}".format(path_to_urdf))
        body = self.env.GetKinBody(name)
        body.SetTransform(transform)
        self.env.Add(body, True)

    def add_cube(self, x, y, z, dim_x, dim_y, dim_z, name='cube'):
        body = openravepy.RaveCreateKinBody(self.env, '')
        body.InitFromBoxes(np.array([[0.0, 0.0, 0.0, dim_x, dim_y, dim_z]]))
        body.SetTransform([[1.0, 0.0, 0.0, x],
                           [0.0, 1.0, 0.0, y],
                           [0.0, 0.0, 1.0, z],
                           [0.0, 0.0, 0.0, 1.0]])
        body.SetName(name)
        self.env.Add(body, True)

    def get_body_markers(self):
        """ Returns a list of visualization_msgs/MarkerArray with all the links of each body in the environment """
        body_markers = []

        # Get all the bodies in the OpenRAVE environment
        bodies = self.env.GetBodies()
        for body in bodies:
            print("Found body with name: {}".format(body.GetName()))
            body_marker = MarkerArray()

            # Choose a random color for this body
            color_r = random.random()
            color_g = random.random()
            color_b = random.random()

            # Create a separate marker for each link
            for link in body.GetLinks():
                print("  Link name: {}".format(link.GetName()))
                link_transform = link.GetTransform()

                link_marker = Marker()
                link_marker.header.frame_id = '/root'
                link_marker.header.stamp = rospy.get_rostime()
                link_marker.ns = body.GetName() + '/link/' + link.GetName()
                link_marker.id = 0
                link_marker.type = Marker.SPHERE

                pose = openravepy.poseFromMatrix(link_transform)

                link_marker.pose.position.x = pose[4]
                link_marker.pose.position.y = pose[5]
                link_marker.pose.position.z = pose[6]
                link_marker.pose.orientation.x = pose[1]
                link_marker.pose.orientation.y = pose[2]
                link_marker.pose.orientation.z = pose[3]
                link_marker.pose.orientation.w = pose[0]

                link_marker.scale.x = 0.2
                link_marker.scale.y = 0.1
                link_marker.scale.z = 0.1
                link_marker.color.r = color_r
                link_marker.color.g = color_g
                link_marker.color.b = color_b
                link_marker.color.a = 0.50
                link_marker.lifetime = rospy.Duration(0)
                body_marker.markers.append(link_marker)

            body_markers.append(body_marker)

        return body_markers

    def plan(self, start_config, goal_config):
        """ Plan from a start configuration to goal configuration """
        rospy.logdebug("Planning from config {} to {}...".format(start_config,
                                                        goal_config))

        dofs = len(start_config)

        start_config[2] += math.pi  # TODO this seems to be a bug in OpenRAVE?
        goal_config[2] += math.pi  # TODO this seems to be a bug in OpenRAVE?

        self.jaco.SetDOFValues(start_config + self.finger_joint_values)

        request = {
            "basic_info":
                {
                    "n_steps": self.trajopt_num_waypoints,
                    "manip": self.jaco.GetActiveManipulator().GetName(),
                    "start_fixed": True
                },
            "costs":
                [
                    {
                        "type": "joint_vel",  # joint-space velocity cost
                        "params": {"coeffs": [1]} # a list of length one is automatically expanded to a list of length n_dofs
                    },
                    {
                        "type": "collision",
                        "params": {
                            "coeffs": [20],
                        # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
                            "dist_pen": [0.025],
                        # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
                            "first_step": 0,
                            "last_step": self.trajopt_num_waypoints - 1,
                            "continuous": True
                        },
                    }
                ],
            "constraints":
                [
                    {
                        "type": "joint",  # joint-space target
                        "params": {"vals": goal_config}  # length of vals = # dofs of manip
                    }
                ],
            "init_info": {
                "type": "straight_line",  # straight line in joint space.
                "endpoint": goal_config
            }
        }
        s = json.dumps(request)  # convert dictionary into json-formatted string
        prob = trajoptpy.ConstructProblem(s, self.env)  # create object that stores optimization problem

        # Add the cost function
        for i, cost_function in enumerate(self.cost_functions):
            prob.AddCost(cost_function.get_cost, [(t, j) for j in range(dofs) for t in range(self.trajopt_num_waypoints)], "cost_{}".format(i))

        t_start = time.time()
        rospy.loginfo("trajoptpy instance: {}".format(trajoptpy))
        result = trajoptpy.OptimizeProblem(prob)  # do optimization
        t_elapsed = time.time() - t_start
        rospy.logdebug("Planning took {} seconds".format(t_elapsed))
        print(result)
        return self._to_trajectory_msg(result.GetTraj())

    def _to_trajectory_msg(self, traj, max_joint_vel=0.2):
        """ Converts to a moveit_msgs/RobotTrajectory message """
        msg = RobotTrajectory()
        msg.joint_trajectory.joint_names = self.joint_names
        t = 0.03
        for i in range(traj.shape[0]):
            p = JointTrajectoryPoint()
            p.positions = traj[i, :].tolist()
            p.positions[2] -= math.pi  # TODO this seems to be a bug in OpenRAVE?
            p.time_from_start = rospy.Duration(t)
            t += self.dt

            msg.joint_trajectory.points.append(p)

        #self._assign_constant_velocity_profile(msg, max_joint_vel)

        return msg
    #def _assign 
    def _assign_constant_velocity_profile(self, traj, max_joint_vel):
        """ Assigns a constant velocity profile to a moveit_msgs/RobotTrajectory """
        t = 0.0
        for i in range(1, len(traj.joint_trajectory.points)):
            p_prev = traj.joint_trajectory.points[i - 1]
            p = traj.joint_trajectory.points[i]

            num_dof = len(p_prev.positions)

            max_joint_dist = 0.0
            for j in range(num_dof):
                dist = math.fabs(angles.shortest_angular_distance(p_prev.positions[j],
                                                                  p.positions[j]))
                max_joint_dist = max(max_joint_dist, dist)

            dt = max_joint_dist / max_joint_vel

            p.velocities = num_dof * [0.0]
            for j in range(num_dof):
                dist = math.fabs(angles.shortest_angular_distance(p_prev.positions[j],
                                                                  p.positions[j]))
                p.velocities[j] = dist / dt

            t += dt
            p.time_from_start = rospy.Duration(t)
