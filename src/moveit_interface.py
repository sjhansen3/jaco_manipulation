#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg
import rospkg
import kinova_msgs.msg
import actionlib

class RobotPlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.group.set_goal_joint_tolerance(0.001)
        self.group.allow_replanning(True)
        self.group.set_planner_id("RRTConnectConfigDefault") 
        #self.group.set_goal_position_tolerance(0.005)

        self.add_scene_items()
        self.solved_plan = None

    def add_scene_items(self):
        self.scene.remove_world_object("ground")
        self.scene.remove_world_object("table")
        rospy.sleep(1)
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "world"
        p.pose.position.x = -1
        p.pose.position.y = 0.5
        p.pose.position.z = -0.05
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        print("add box")
        
        self.scene.add_mesh("table",p,"Desk.stl",size = (0.01, 0.01, 0.01))

        p.pose.position.y = 0
        p.pose.position.x = 0
        #TODO add ros param to turn off ground
        self.scene.add_box("ground", p, (3, 3, 0.02))

        p.pose.position.z = 0
        self.scene.add_box("base",p,(0.15,0.15,0.15))

    def plan(self, position):
        self.group.clear_pose_targets()

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 0.707
        pose_target.orientation.y = 0.707
        pose_target.position.x = position[0]
        pose_target.position.y = position[1]
        pose_target.position.z = position[2]
        
        self.group.set_pose_target(pose_target)
        print("planning")
        self.group.set_start_state_to_current_state()
        rospy.sleep(1)
        self.solved_plan = self.group.plan() #automatically displays trajectory
        print("displaying")
        

    def execute(self):
        if self.solved_plan is None:
            rospy.logwarn("Plan is None, planning might not have been completed or failed")
            return
        self.group.execute(self.solved_plan)

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

if __name__ == '__main__':
    rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)
    rospy.sleep(2)
    robot_planner = RobotPlanner()
    grip_controller = GripController()
    while True:
        #pre grip location
        robot_planner.plan([0.3,0.3,0.4])
        grip_controller.grip("percent",[0,0,0])
        raw_input("plan to start pose commplete, anykey to execute")
        robot_planner.execute()

        #grip location
        robot_planner.plan([0.7,0.3,0.4])
        raw_input("plan to grip pose commplete, anykey to execute")
        robot_planner.execute()
        
        #grip object
        raw_input("grip object? anykey to execute")
        grip_controller.grip("percent",[75,75,75])
        

        #pull back
        robot_planner.plan([0.3,0.3,0.3])
        robot_planner.execute()
        print("moving back to start")
        rospy.sleep(3)
        #release object
        grip_controller.grip("percent",[0,0,0])
        if rospy.is_shutdown():
            break

    rospy.spin()