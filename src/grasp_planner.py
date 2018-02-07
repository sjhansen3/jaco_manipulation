#!/usr/bin/env python

import numpy as np
import tf
import rospy
from spacial_location import Pose

class GraspPlanner:
    def __init__(self):
        self.listener = tf.TransformListener()
    
    def get_grasp_plan(self, object_name):
        assert False

class ARTrackPlanner(GraspPlanner):
    def __init__(self):
        GraspPlanner.__init__(self)
        self.object_to_marker_num = {"cup": 8, "spoon":99}

    def get_grasp_plan(self, object_name, marker_num = None):
        if object_name not in self.object_to_marker_num and not marker_num:
            raise ValueError("no marker assigned for marker {}".format(object_name))
        if marker_num:
            self.object_to_marker_num[object_name] = marker_num
        ar_frame = "/ar_marker_{}".format(self.object_to_marker_num[object_name])
        world_frame = "/world"
        for num_trys in range(20):
            try:
                (trans, rot) = self.listener.lookupTransform(ar_frame, world_frame, rospy.Time(0))
                rospy.loginfo("found")
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Waiting for AR tracker tf frame {}".format(ar_frame))
                rospy.sleep(0.1)

        print np.asarray(trans),np.asarray(rot)
        grasp_pose = Pose(np.asarray(trans),np.asarray(rot))
        return grasp_pose

if __name__ == '__main__':
    planner = ARTrackPlanner()
    planner.get_grasp_plan("cup")