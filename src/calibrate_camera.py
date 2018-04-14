#!/usr/bin/env python

import tf
import rospy
import numpy as np
import signal

class Calibrator:
    def __init__(self, trans, markernumber=6):
        """ Calibrates the position of a camera using an AR tracker
        The AR tracker is placed in a fixed translation from the /world frame of the robot

        Params
        ---
        markernumber: The number of the ar_marker
        trans: The translation from the world frame to the AR tracker frame
        """
        self.br = tf.TransformBroadcaster()
        self.markernumber = markernumber
        self.listener = tf.TransformListener()
        self.trans_world2ar = trans
        self.calibration_counter = 0

        #default T_kinect_world
        # -0.7 0 1.4 0 0.45 0
        self.trans = (-0.7,0,1.4)
        self.orientation = (0,-0.223,0,0.975)

    def send_static_transform(self):
        """Broadcast the transformation between /world and the kinect
        """
        self.br.sendTransform(self.trans, self.orientation, rospy.Time.now(), "/camera_link", "/world" )
    
    def get_static_transform(self):
        """ Finds the transformation from kinect to the ar tag
        After :calibration counter: successful tf reads it stores the most recent value

        The AR tracker is assumed to have no rotation compared to the world frame
        The translation is specified from the world to the AR marker
        """

        try:
            #Providing rospy.Time(0) will just get us the latest available transform
            if self.calibration_counter<20:
                rospy.loginfo("Calibrating kinect {}".format(self.calibration_counter))
                ar_frame = "/ar_marker_{}".format(markernumber)
                kinect_frame = "/kinect2_rgb_optical_frame"

                (trans, rot) = self.listener.lookupTransform(ar_frame, kinect_frame, rospy.Time(0))

                trans = np.asarray(trans)
                trans += self.trans_world2ar #adjust T_ar_kinect to match T_world_kinect

                self.calibration_counter+=1
                self.trans = trans
                self.orientation = rot
                if self.calibration_counter == 19:
                    rospy.loginfo("Calibration complete")
            return (self.trans, self.orientation)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

            rospy.loginfo("Waiting for AR tracker tf frame for calibration")

if __name__ == '__main__':
    rospy.init_node('calibrate_camera')
    markernumber = 6
    translation = [-0.06, -0.072, -0.025] #trnslation of QR tracker in robot reference frame
    calibrator = Calibrator(translation, markernumber)

    # setup safe termination
    def handler(signum, frame):
        rospy.loginfo('caught CTRL+C, exiting...')         
        exit(0)
    signal.signal(signal.SIGINT, handler)

    #rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        # calibrator.get_static_transform()
        calibrator.send_static_transform()

        #rate.sleep()

    rospy.spin()