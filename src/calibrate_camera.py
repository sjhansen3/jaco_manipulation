#!/usr/bin/env python

import tf
import rospy
import numpy as np
from autolab_core import RigidTransform
import numpy as np

import std_msgs.msg

class Calibrator:
    def __init__(self, markernumber=6):
        self.br = tf.TransformBroadcaster()
        self.markernumber = markernumber
        self.listener = tf.TransformListener()

    
    def broadcast_transform(self, transform):
        """Define the trasformation between 
        the end effector and the QR tag on the end effector
        """

        quaternion = transform.quaternion
        translation = transform.translation
                
        self.br.sendTransform(translation, quaternion, rospy.Time.now(), transform.to_frame, transform.from_frame)
    
    def get_transfom(self, target_frame, source_frame):
        """ Finds the 4x4 transformation matrix from source frame to target frame
        Params
        ---
        target_frame: a string label of the target frame
        source_frame: a string label of the source frame
        Returns
        ---
        4x4 matrix transform
        """
        try:
            #Providing rospy.Time(0) will just get us the latest available transform
            (trans, rot) = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            
            h_mat = self.listener.fromTranslationRotation(trans,rot)

            return RigidTransform(rotation=h_mat[0:3,0:3], translation=trans, from_frame=target_frame, to_frame=source_frame)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return RigidTransform(rotation=np.eye(3), translation = np.array([0,0,0]), from_frame=target_frame, to_frame=source_frame)

if __name__ == '__main__':
    rospy.init_node('camera_calibration')
    markernumber = 6
    calibrator = Calibrator(markernumber)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print("***Kinect to AR6")
        
        T_kinect_ar = calibrator.get_transfom("/ar_marker_{}".format(markernumber), "/kinect2_rgb_optical_frame")

        T_eef_world = calibrator.get_transfom("/world", "/j2s7s300_end_effector")
        T_ar_eef = RigidTransform(rotation=np.eye(3) , translation=[0,0,0], from_frame="/j2s7s300_end_effector", to_frame="/ar_marker_{}".format(markernumber))

        T_world_kinect = T_kinect_ar*T_ar_eef*T_eef_world
        print("resulting",T_world_kinect.from_frame, T_world_kinect.to_frame)

        print(T_world_kinect)
        calibrator.broadcast_transform(T_world_kinect)
        rate.sleep()


    #rospy.Timer(rospy.Duration(1./30), calibrator.fix_eef_to_fudcuial)
    print("fiducial")
    rospy.spin()