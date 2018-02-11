#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionFK

import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg

import rospkg
import kinova_msgs.msg
import actionlib
from spacial_location import Pose
import numpy as np
from grasp_planner import ARTrackPlanner

class RobotPlanner:
    #TODO consider renaming this - it has expanded beyond a planner
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")

        self.group.set_planner_id("RRTConnectkConfigDefault") 
        rospy.sleep(1)
        self.group.set_goal_tolerance(0.001)

        self.group.allow_replanning(True)
        

        self.add_scene_items()
        self.solved_plan = None
        self.eef_name = "j2s7s300_end_effector"

        # Set up the FK service
        rospy.wait_for_service('/compute_fk')
        self.compute_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)


        # Set up the IK service
        rospy.wait_for_service('/compute_ik')
        self.compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    def add_scene_items(self):
        """
        Add obstacles into the scene for planning with moveit
        """
        #TODO add scene item for tripod and bookshelf
        self.scene.remove_world_object("ground")
        #self.scene.remove_world_object("table")

        rospy.sleep(1)

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "world"
        p.pose.position.x = -1
        p.pose.position.y = 0.5
        p.pose.position.z = -0.11
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        print("add box")
        
        #add the desk
        #TODO get orientation and size correct
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('jaco_manipulation')
        model_folder_path = package_path + "/models/"

        #self.scene.add_mesh("table",p,model_folder_path+"Desk.stl",size = (0.01, 0.01, 0.01))

        p.pose.position.y = 0
        p.pose.position.x = 0
        #TODO add ros param to turn off ground
        self.scene.add_box("ground", p, (3, 3, 0.02)) #add a "ground plane"


        p.pose.position.z = -0.05
        self.scene.add_box("base",p,(0.15,0.15,0.15)) #add a base for safety

    def plan(self, pose_input):
        """ find a plan to the position
        Params
        ---
        pose_input: The pose to plan to. `string` or `Pose`
        Returns
        ---
        1 if plan succeeded 0 otherwise
        """
        if isinstance(pose_input, basestring):
            pose = Pose.load(pose_input)
        elif isinstance(pose_input, Pose):
            pass
        else:
            raise ValueError("plan only supports pose objects")

        self.group.clear_pose_targets()
        
        self.group.set_pose_target(pose.ros_message)
        rospy.loginfo("Planning to {}".format(pose))
        self.group.set_start_state_to_current_state()
        rospy.sleep(1)
        self.solved_plan = self.group.plan() #automatically displays trajectory

        cur_joints = list(self.solved_plan.joint_trajectory.points[-1].positions)
        posefk = self.fk(cur_joints).pose_stamped[0].pose
        final_pose_res = Pose([posefk.position.x, posefk.position.y, posefk.position.z],[posefk.orientation.x, posefk.orientation.y, posefk.orientation.z, posefk.orientation.w])
        return pose, final_pose_res 
        #TODO add return value for success or failure
    
    def fk(self, joints, links=['j2s7s300_end_effector']):
        """ Computes the forward kinematics 
        joints: an array of joint positions
        """
        header = std_msgs.msg.Header()
        robot_state = moveit_msgs.msg.RobotState()
        robot_state.joint_state.name = ['j2s7s300_joint_{}'.format(i) for i in range(1,8)]
        robot_state.joint_state.position = joints
        return self.compute_fk(header, links, robot_state)

    
    # def ik(self, pose_stamped, group_name='arm'):
    #     """ Computes the inverse kinematics """
    #     req = PositionIKRequest()
    #     req.group_name = group_name
    #     req.pose_stamped = pose_stamped
    #     req.timeout.secs = 0.1
    #     req.avoid_collisions = False

    #     try:
    #         res = self.compute_ik(req)
    #         return res
    #     except rospy.ServiceException, e:
    #         print("IK service call failed: {}".format(e))

    #TODO add waypoint planning for pregrasp -> grasp
    def plan_waypoints(self, waypoints):
        """
        No more than jump_threshold is allowed as change in distance 
        in the configuration space of the robot (this is to prevent 'jumps' in IK solutions).
        """
        #TODO check waypoints input

        jump_threshold = 0.0
        eef_step = 0.01
        (self.solved_plan, percent_solved) = self.group.compute_cartesian_path(waypoints, eef_step, jump_threshold, avoid_collisions = True)
        rospy.loginfo("{} percent of path planned as described by the waypoints".format(percent_solved))

    def execute(self):
        """ Execute a planned path
        """
        if self.solved_plan is None:
            rospy.logwarn("Plan is None, planning might not have been completed or failed")
            return
        self.group.execute(self.solved_plan)
        self.solved_plan = None #TODO is it appropriate to set solved_plan to none after its been executed
        #Do we want to solve the same plan twice in a row? I think no
        print("execution complete")
    
    @property
    def current_pose(self):
        cur_pose = self.group.get_current_pose(self.eef_name)
        position = [cur_pose.pose.position.x, cur_pose.pose.position.y, cur_pose.pose.position.z]
        orientation = [cur_pose.pose.orientation.x, cur_pose.pose.orientation.y, cur_pose.pose.orientation.z, cur_pose.pose.orientation.w]
        pose = Pose(position,orientation)
        return pose


class GripController:
    def __init__(self):
        self.finger_maxDist = 18.9/2/1000  # max distance for one finger
        self.finger_maxTurn = 6800  # max thread rotation for one finger
        
        action_address = '/j2s7s300_driver/fingers_action/finger_positions'

        self.grip_action_client = actionlib.SimpleActionClient(action_address,
                                            kinova_msgs.msg.SetFingersPositionAction)

    def grip(self, units, finger_positions):
        """ 
        Params
        ---
        units: a string representing units ()
        finger_values: an array like object of dim 3
        Returns
        ---
        None if the grip timed out
        """
        turn, meter, percent = self._convert_units(units, finger_positions)
        positions_temp1 = [max(0.0, n) for n in turn]
        positions_temp2 = [min(n, self.finger_maxTurn) for n in positions_temp1]
        positions = [float(n) for n in positions_temp2]
        return self._execute_grip(turn)

    def _execute_grip(self, finger_positions):
        """ move the gripper to the finger position
        """
        if len(finger_positions) != 3:
            raise ValueError("wrong number of finger specified")
        
        print("wating for action client")
        self.grip_action_client.wait_for_server()

        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_positions[0])
        goal.fingers.finger2 = float(finger_positions[1])
        goal.fingers.finger3 = float(finger_positions[2])
        print("sending grip goal {}".format(goal))
        self.grip_action_client.send_goal(goal)
        if self.grip_action_client.wait_for_result(rospy.Duration(5.0)):
            return self.grip_action_client.get_result()
        else:
            self.grip_action_client.cancel_all_goals()
            rospy.logwarn('        the gripper action timed-out')
            return None

    def _convert_units(self, unit_, finger_value_):
        """
        converts units from the input unit (unit_) to all units (turn_, meter_, percent_)
        ---
        unit_: a string representing the input units (turn, mm, percent)
        finger_value_: a  value associated with the grasp
        Returns
        ---
        a tuple containing all units turn units, finger meter, finger percent
        """
        # transform between units
        if unit_ == 'turn':
            # get absolute value
            finger_turn_ = finger_value_
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]

        elif unit_ == 'mm':
            # get absolute value
            finger_turn_ = [x/1000 * self.finger_maxTurn / self.finger_maxDist for x in finger_value_]
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]
        
        elif unit_ == 'percent':
            # get absolute value
            finger_turn_ = [x/100.0 * self.finger_maxTurn for x in finger_value_]
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]
        else:
            raise Exception("Finger value have to be in turn, mm or percent")

        return finger_turn_, finger_meter_, finger_percent_

def run_pick_place():
    rospy.sleep(2)
    robot_planner = RobotPlanner()
    grip_controller = GripController()

    #Get the ARTracker Pose
    grasp_planner = ARTrackPlanner()
    grasp_pose = grasp_planner.get_grasp_plan("cup")

    #home grip location
    robot_planner.plan("home_grip")
    grip_controller.grip("percent",[0,0,0])
    raw_input("plan to home_grip commplete, anykey and enter to execute")
    robot_planner.execute()

    #pre grip location
    robot_planner.plan("grasp_pre_hardcode")
    raw_input("plan to grasp_pre_hardcode commplete, anykey and enter to execute")
    robot_planner.execute()

    #grip location
    robot_planner.plan("grasp_hardcode")
    raw_input("plan to grasp_hardcode commplete, anykey and enter to execute")
    robot_planner.execute()
    
    #grip object
    raw_input("grip object? anykey to execute")
    grip_controller.grip("percent",[75,75,75])
    
    #target pre
    robot_planner.plan("grasp_target_pre")
    raw_input("plan to grasp_target_pre commplete, anykey and enter to execute")
    robot_planner.execute()

    #target location
    robot_planner.plan("grasp_target")
    raw_input("plan to grasp_target commplete, anykey and enter to execute")
    robot_planner.execute()

    #release object
    raw_input("release object? anykey to execute")
    grip_controller.grip("percent",[0,0,0])

    #go bak to pre grip location
    robot_planner.plan("grasp_target_after")
    raw_input("plan to back to grasp_pre_hardcode commplete, anykey and enter to execute")
    robot_planner.execute()

    #home grip location
    robot_planner.plan("home_grip")
    raw_input("plan to home_grip commplete, anykey and enter to execute")
    robot_planner.execute()


def test_plan_waypoints():
    rospy.sleep(2)
    robot_planner = RobotPlanner()
    grip_controller = GripController()
    
    downstream_error = []
    rrt_error = []
    for i in range(5):
        #home grip location
        downstream, rrt = move_to_position("pos_1", robot_planner)
        downstream_error.append(downstream)
        rrt_error.append(rrt)

        #pre grip location
        downstream, rrt = move_to_position("pos_2", robot_planner)
        downstream_error.append(downstream)
        rrt_error.append(rrt)
    print "\n********RRT Error*********\n", rrt_error
    print "\n*******DOWNSTREAM*********\n", downstream_error

def move_to_position(position_name, robot_planner):
    #home grip location
    pose_request, pose_final_plan = robot_planner.plan(position_name)

    raw_input("plan to {} commplete, anykey and enter to execute".format(position_name))
    robot_planner.execute()
    rospy.sleep(2)
    
    act_pose = robot_planner.current_pose
    downstream_error = act_pose.position - pose_final_plan.position
    rrt_error = pose_request.position - pose_final_plan.position
    
    
    print "actual pose: ", act_pose
    print "requested pose:  ", pose_request
    print "final pose of plan:  ", pose_final_plan
    
    print "rrt_error: ", rrt_error
    rrt_norm = np.linalg.norm(np.asarray(rrt_error[0:3]))
    print "rrt_norm:   ", rrt_norm

    downstream_norm = np.linalg.norm(np.asarray(downstream_error[0:3]))
    print "downstream_error: ", downstream_error
    print "downstream_norm: ", downstream_norm 
    return downstream_norm, rrt_norm

if __name__ == '__main__':
    rospy.init_node('moveit_interface',
                        anonymous=True)

    test_plan_waypoints()
