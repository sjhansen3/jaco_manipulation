#!/usr/bin/env python

import tf
import rospy
import numpy as np
import autolab_core
import numpy as np

class Calibrator:
    def __init__(self, markernumber=6):
        self.br = tf.TransformBroadcaster()
        self.markernumber = markernumber
        self.listener = tf.TransformListener()

    
    def broadcast_transform(self, h_transform, from_frame, to_frame):
        """Define the trasformation between 
        the end effector and the QR tag on the end effector
        """
        translation = tf.transformations.translation_from_matrix(h_transform)

        quaternion = tf.transformations.quaternion_from_matrix(h_transform)
                
        self.br.sendTransform(translation, quaternion, rospy.Time.now(), from_frame, to_frame)
    
    def get_transfom(self, from_frame, to_frame):
        try:
            #Providing rospy.Time(0) will just get us the latest available transform
            (trans, rot) = self.listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
    
            #print(trans, rot)
            rot_matrix = tf.transformations.quaternion_matrix(rot)
            trans_matrix = tf.transformations.translation_matrix(trans)
            h_mat = np.dot(trans_matrix,rot_matrix)
            return h_mat
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return np.eye(4)

if __name__ == '__main__':
    rospy.init_node('camera_calibration')
    markernumber = 6
    calibrator = Calibrator(markernumber)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print("***Kinect to AR6")
        #TODO which is from frame?
        T_kinect_ar = calibrator.get_transfom("/kinect2_rgb_optical_frame","ar_marker_{}".format(markernumber))

        T_world_eef = calibrator.get_transfom("/world","/j2s7s300_end_effector")
        T_eef_ar = [[1, 0, 0, 0],[0, -1, 0, 0],[0, 0, -1, 0],[0, 0, 0.045, 1]]


        T_ar_kinect = np.linalg.inv(T_kinect_ar) #from ar tag to kinect
        T_world_kinect = np.dot(np.dot(T_world_eef, T_eef_ar), T_ar_kinect)

        print(T_world_kinect, T_world_eef, T_eef_ar, T_ar_kinect)
        calibrator.broadcast_transform(T_world_kinect, '/kinect2_rgb_optical_frame', '/world')
        rate.sleep()


    #rospy.Timer(rospy.Duration(1./30), calibrator.fix_eef_to_fudcuial)
    print("fiducial")
    rospy.spin()