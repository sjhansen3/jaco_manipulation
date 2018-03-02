#!/usr/bin/env python

import numpy as np
import tf
import rospy
import spacial_location
from spacial_location import Pose #TODO replace this with full reference, too many poses to use simply Pose
import utils
import geometry_msgs.msg

class GraspPlanner:
    """ Abstract clss for grasp planning.
    """
    def __init__(self):
        self.listener = tf.TransformListener()
    
    def get_grasp_plan(self, object_name):
        assert False

class ARTrackPlanner(GraspPlanner):
    """ One small step above hard coding - uses AR trackers and a dictionary to find poses
    #TODO implement dictionary with pose offsets based on object type
    """
    def __init__(self):
        GraspPlanner.__init__(self)
        #self.object_to_marker_num = {"cup": 8,"target": 2)}
        self.object_dict = {"cup": (8,-0.02),"target": (2,0.07)} #(marker number, z offset)

    def get_grasp_plan(self, object_name, marker_num = None):
        """ The grasp plan for the AR tracker is simply based on a fixed offset for each object
        Params
        ---
        object_name: The name of the object to be grasped. e.g. cup
        marker_num: the associated marker for the object. If you would like
        to override the parameter put in the dictionary
        Returns
        ---
        A tuple containing pregrasp pose and grasp pose
        """
        if object_name not in self.object_dict and not marker_num:
            raise ValueError("no marker assigned for marker {}".format(object_name))
        if marker_num:
            self.object_dict[object_name][0] = marker_num
        ar_frame = "/ar_marker_{}".format(self.object_dict[object_name][0])
        world_frame = "/world"

        self.listener.waitForTransform(ar_frame, world_frame, rospy.Time(0), rospy.Duration(4.0))
        try:
            (trans, rot) = self.listener.lookupTransform(world_frame, ar_frame, rospy.Time(0))
            #break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Did not get for AR tracker tf frame {}".format(ar_frame))
        hard_code_pose = Pose.load("opposite_grasp")
        print "z: ", hard_code_pose.position.z
        
        
        rospy.loginfo("AR tracker translation: {}, rotation: {}".format(trans, rot))
        rospy.loginfo("Hardcoded pose {}".format(hard_code_pose))

        cup_position = np.asarray([trans[0], trans[1], trans[2] + self.object_dict[object_name][1]])
        #cup_position = np.asarray([trans[0], trans[1], (hard_code_pose.position.z+self.object_dict[object_name][1])])

        grasp_pose = Pose(cup_position, hard_code_pose.orientation)
        rospy.loginfo("found tracker positon {}".format(grasp_pose))

        pre_grasp_pose = self._offset_hand(grasp_pose, offset_dist=0.1)
        return pre_grasp_pose, grasp_pose

    def _offset_hand(self, grasp_pose, offset_dist=0.1):
        """ Find the pre grasp pose by offsetting the hand backwards from its current position
        grasp_pose: the pose of the grasp location
        offset_dist: the amount to offset off the object
        Returns
        ---
        pre_grasp_pose: the offsetted pose of the object
        """
        #use the unit z vector because thats the direction out of the hand
        unit_z_vector = np.asarray([0,0,1])
        direction = spacial_location.qv_mult(grasp_pose.orientation[0:4], unit_z_vector)
        #print "direction", direction
        print "grasp_pose.position: ", grasp_pose.position

        grasp_pose.show_position_marker(ident = 1, label = "grasp pose")

        pre_grasp_position = grasp_pose.position - direction*offset_dist

        pre_grasp_pose = Pose(pre_grasp_position, grasp_pose.orientation)
        pre_grasp_pose.show_position_marker(ident = 2, label = "pregrasp pose")

        return pre_grasp_pose

def test_offset_hand():
    rospy.sleep(5)
    grasp_pose = Pose([0.5,0.5,0.5], [0,0,0,1])
    planner = ARTrackPlanner()
    planner._offset_hand(grasp_pose)



if __name__ == '__main__':
    rospy.init_node("Grasp_planner_test_node")
    test_offset_hand()
    rospy.spin()
    #planner = ARTrackPlanner()
    #planner.get_grasp_plan("cup")