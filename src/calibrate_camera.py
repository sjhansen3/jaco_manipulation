#!/usr/bin/env python

import tf
import tf2_ros
import rospy
import numpy as np
import geometry_msgs

class Calibrator:
    def __init__(self, trans, markernumber=6):
        """ Calibrates the position of a camera using an AR tracker
        The AR tracker is placed in a fixed translation from the /world frame of the robot

        Params
        ---
        markernumber: The number of the ar_marker
        trans: The translation from the world frame to the AR tracker frame
        """
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.markernumber = markernumber
        self.listener = tf.TransformListener()
        self.trans_world2ar = trans
        self.calibration_counter = 0

        #default T_camera_world
        self.trans = (0,0,0)
        self.orientation = (0,0,0,1)

    def send_static_transform(self):
        """Broadcast the transformation between /world and the RGBD camera
        """

        if not any(self.trans):
            rospy.logerr("Calibration did not succeed")

        static_transformStamped = geometry_msgs.msg.TransformStamped()
    
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "/world"
        static_transformStamped.child_frame_id = "/map"
    
        static_transformStamped.transform.translation.x = self.trans[0]
        static_transformStamped.transform.translation.y = self.trans[1]
        static_transformStamped.transform.translation.z = self.trans[2]
    
        static_transformStamped.transform.rotation.x = self.orientation[0]
        static_transformStamped.transform.rotation.y = self.orientation[1]
        static_transformStamped.transform.rotation.z = self.orientation[2]
        static_transformStamped.transform.rotation.w = self.orientation[3]
    
        self.broadcaster.sendTransform(static_transformStamped)
    
    def get_static_transform(self):
        """ Finds the transformation from RGBD camera to the ar tag
        After :calibration counter: successful tf reads it stores the most recent value

        The AR tracker is assumed to have no rotation compared to the world frame
        The translation is specified from the world to the AR marker
        """
        for trys in range(20):
            try:
                #Providing rospy.Time(0) will just get us the latest available transform
                rospy.loginfo("Calibrating positions {}".format(self.calibration_counter))
                ar_frame = "/ar_marker_{}".format(markernumber)
                camera_frame = "/map"

                (trans, rot) = self.listener.lookupTransform(ar_frame, camera_frame, rospy.Time(0))

                trans = np.asarray(trans)
                trans += self.trans_world2ar #adjust T_ar_camera to match T_world_camera

                self.trans = trans
                self.orientation = rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

                rospy.logwarn("Waiting for AR tracker tf frame for calibration")
                rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('calibrate_camera')
    markernumber = 6
    translation = [0, -0.062, -0.035] #trnslation of QR tracker in robot reference frame
    calibrator = Calibrator(translation, markernumber)

    calibrator.get_static_transform()
    calibrator.send_static_transform()

    rospy.spin()



