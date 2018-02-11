#!/usr/bin/env python

import rospy
from moveit_interface import RobotPlanner
from spacial_location import Pose

def save_poses():
    """ Code for saving a specific pose to file
    Intended usage: jog the robot to a position,
    type the name of the pose to save and save it for use in future applications
    """
    rospy.sleep(2)
    robot_planner = RobotPlanner()
    while not rospy.is_shutdown():
        pose_name = raw_input("Type frame name to save, q to exit >  ")
        if pose_name == "q":
            return        
        else:
            pose = robot_planner.current_pose
            print(pose, "cur_pose")
            pose.save(pose_name)
            print("Saved pose: \n{}\n under name {}".format(pose,pose_name))


if __name__ == '__main__':
    rospy.init_node('store_pose',
                        anonymous=True)

    save_poses()
    #sample code for p