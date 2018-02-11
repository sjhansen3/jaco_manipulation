#! /usr/bin/env python

import rospy

import actionlib
import ros_utils

import pid_controller
import actionlib_tutorials.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import kinova_msgs

PREFIX_KINOVA_TRAJ_CONTROL = "/j2s7s300_driver/trajectory_controller"
PREFIX_MOVE_GROUP = "/j2s7s300/follow_joint_trajectory"

# anca@anca-Ubuntu:~/workspace_steve$ rosnode info /j2s7s300_joint_trajectory_action_server 
# --------------------------------------------------------------------------------
# Node [/j2s7s300_joint_trajectory_action_server]
# Publications: 
#  * /j2s7s300_driver/trajectory_controller/command [trajectory_msgs/JointTrajectory]
#  * /j2s7s300/follow_joint_trajectory/result [control_msgs/FollowJointTrajectoryActionResult]
#  * /j2s7s300/follow_joint_trajectory/feedback [control_msgs/FollowJointTrajectoryActionFeedback]
#  * /j2s7s300/follow_joint_trajectory/status [actionlib_msgs/GoalStatusArray]
#  * /rosout [rosgraph_msgs/Log]

# Subscriptions: 
#  * /j2s7s300_driver/trajectory_controller/state [control_msgs/FollowJointTrajectoryFeedback]
#  * /j2s7s300/follow_joint_trajectory/goal [control_msgs/FollowJointTrajectoryActionGoal] -> pub from /move_group
#  * /j2s7s300/follow_joint_trajectory/cancel [actionlib_msgs/GoalID] -> pub from /move_group

# Services: 
#  * /j2s7s300_joint_trajectory_action_server/get_loggers #TODO ignore these?
#  * /j2s7s300_joint_trajectory_action_server/set_logger_level

# contacting node http://anca-Ubuntu:43597/ ...
# Pid: 12315
# Connections:
#  * topic: /rosout
#     * to: /rosout
#     * direction: outbound
#     * transport: TCPROS
#  * topic: /j2s7s300_driver/trajectory_controller/command
#     * to: /j2s7s300_driver
#     * direction: outbound
#     * transport: TCPROS
#  * topic: /j2s7s300/follow_joint_trajectory/result
#     * to: /move_group
#     * direction: outbound
#     * transport: TCPROS
#  * topic: /j2s7s300/follow_joint_trajectory/feedback
#     * to: /move_group
#     * direction: outbound
#     * transport: TCPROS
#  * topic: /j2s7s300/follow_joint_trajectory/status
#     * to: /move_group
#     * direction: outbound
#     * transport: TCPROS
#  * topic: /j2s7s300_driver/trajectory_controller/state
#     * to: /j2s7s300_driver (http://anca-Ubuntu:46101/)
#     * direction: inbound
#     * transport: TCPROS
#  * topic: /j2s7s300/follow_joint_trajectory/goal
#     * to: /move_group (http://anca-Ubuntu:34975/)
#     * direction: inbound
#     * transport: TCPROS
#  * topic: /j2s7s300/follow_joint_trajectory/cancel
#     * to: /move_group (http://anca-Ubuntu:34975/)
#     * direction: inbound
#     * transport: TCPROS


class JointTrajectoryAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(PREFIX_MOVE_GROUP, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._feedback_sub = rospy.Subscriber(PREFIX_MOVE_GROUP+'/state',control_msgs.msg.FollowJointTrajectoryActionFeedback, self.feedback_cb)
        self._controller = pid_controller.PIDController()
        self._vel_pub = rospy.Publisher('/j2s7s300_driver/in/joint_velocity',
                                       kinova_msgs.msg.JointVelocity,
                                       queue_size=1)
        self._as.start()


    def feedback_cb(self, feedback):
        self._feedback = feedback

    def execute_cb(self, goal):
        #TODO goal can be cancel?

        # g = control_msgs.msg.FollowJointTrajectoryGoal()
        # g.trajectory
        # g.goal_time_tolerance
        # g.goal_tolerance
        # g.path_tolerance
    
        # helper variables
        success = True
        
        # append the seeds for the fibonacci sequence
        
        # publish info to the console for the user
        rospy.loginfo('Starting Joint Action Server')
        
        # start executing the action
        self._controller.load_trajectory(goal.trajectory)

        r = rospy.Rate(100)
        while not rospy.is_shutdown() and not (self._controller.reached_goal and
                                               self._controller.reached_start):
            if self._controller.is_shutdown:
                success = False
                self._as.set_preempted()
                break

            self._vel_pub.publish(ros_utils.cmd_to_JointVelocityMsg(self._controller.cmd))


            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #TODO there are other reasons to shutdown
                self._controller.shutdown_controller()
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            
            # publish the feedback
            self._as.publish_feedback(self._feedback)

            r.sleep()
          
        if success:
            self._result.error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('follow_joint_trajectory')
    server = JointTrajectoryAction(rospy.get_name())
    rospy.spin()
