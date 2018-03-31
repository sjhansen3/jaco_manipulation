#!/usr/bin/env python

import numpy as np
import tf
import rospy
import signal

import spacial_location
from spacial_location import Pose #TODO replace this with full reference, too many poses to use simply Pose

from std_msgs.msg import Header
from gqcnn.srv import GQCNNGraspPlanner
from gqcnn.msg import GQCNNGrasp, BoundingBox
import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo

from perception import CameraIntrinsics, ColorImage, DepthImage
from perception import RgbdDetectorFactory, RgbdSensorFactory

from autolab_core import YamlConfig
from autolab_core import Box

from cv_bridge import CvBridge, CvBridgeError

from gqcnn import Visualizer as vis

import cv2
import matplotlib.pyplot as plt
import copy

import matplotlib.pyplot as pyplot


class GraspPlanner:
    """ Abstract clss for grasp planning.
    """
    def __init__(self):
        self.listener = tf.TransformListener()
    
    def get_grasp_plan(self, object_name):
        assert False

class GQCNNPlanner(GraspPlanner):
    """ pass an image to the GQCNN
    """
    def __init__(self, camera_intrinsics, config):
        # wait for Grasp Planning Service and create Service Proxy
        rospy.loginfo("Waiting for GQCNN to spin up")
        rospy.wait_for_service('plan_gqcnn_grasp')
        self.plan_grasp = rospy.ServiceProxy('plan_gqcnn_grasp', GQCNNGraspPlanner)
        rospy.loginfo("GQCNN service Initialized")

        self.config = config
        # get camera intrinsics
        self.camera_intrinsics = camera_intrinsics

    def _bbox_to_msg(self, bbox):
        """
        Params
        ---
        bbox: numpy array [minX, minY, maxX, maxY] in pixels around the image 
        Returns
        ---
        a bondingBox message type
        """       
        boundingBox = BoundingBox()
        boundingBox.minX = bbox[0]
        boundingBox.minY = bbox[1]
        boundingBox.maxX = bbox[2]
        boundingBox.maxY = bbox[3]
        return boundingBox

    def get_grasp_plan(self, bounding_box, color_image, depth_image):
        """ finds the highest quality score grasp associated with the object

        """
        #grab frames for depth image and color image
        rospy.loginfo("grabbing frames")

        boundingBox = self._bbox_to_msg(bounding_box)
        
        pyplot.figure()
        pyplot.subplot(2,3,1)
        pyplot.title("color image")
        pyplot.imshow(color_image.data)

        pyplot.subplot(2,3,2)
        pyplot.title("depth image")
        pyplot.imshow(depth_image.data)

        pyplot.subplot(2,3,3)
        pyplot.title("non finite locations on the image")
        pyplot.imshow(np.isfinite(depth_image.data))

        # inpaint to remove holes
        inpainted_color_image = color_image.inpaint(self.config["inpaint_rescale_factor"]) #TODO make rescale factor in config
        inpainted_depth_image = depth_image.inpaint(self.config["inpaint_rescale_factor"])
        print("Is the inpainted depth image finite?", np.all(np.isfinite(inpainted_depth_image.data)))


        pyplot.subplot(2,3,4)
        pyplot.title("inpaint color")
        pyplot.imshow(inpainted_color_image.data)

        pyplot.subplot(2,3,5)
        pyplot.title("inpaint depth")
        pyplot.imshow(inpainted_depth_image.data)
        pyplot.show()
        
        try:
            rospy.loginfo("Sending grasp plan")
            planned_grasp_data = self.plan_grasp(inpainted_color_image.rosmsg, inpainted_depth_image.rosmsg, self.camera_intrinsics.rosmsg, boundingBox)
            rospy.loginfo("grasp plan complete")

            grasp = planned_grasp_data.grasp
            rospy.loginfo('Processing Grasp')

            rotation_quaternion = np.asarray([grasp.pose.orientation.w, grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z]) 
            translation = np.asarray([grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
            
            grasp_pose = Pose(translation, rotation_quaternion)
            rospy.loginfo("found tracker positon {}".format(grasp_pose))

            pre_grasp_pose = offset_hand(grasp_pose, offset_dist=0.1)
            return pre_grasp_pose, grasp_pose

            print planned_grasp_data
            # lift_gripper_width, T_gripper_world = process_GQCNNGrasp(planned_grasp_data, robot, left_arm, right_arm, subscriber, home_pose, config)

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: \n %s" % e)  
    
class ARTrackPlanner(GraspPlanner):
    """ One small step above hard coding - uses AR trackers and a dictionary to find poses
    #TODO implement dictionary with pose offsets based on object type
    """
    def __init__(self):
        GraspPlanner.__init__(self)
        self.object_to_marker_num = {"cup": 8, "spoon":99}

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
        if object_name not in self.object_to_marker_num and not marker_num:
            raise ValueError("no marker assigned for marker {}".format(object_name))
        if marker_num:
            self.object_to_marker_num[object_name] = marker_num
        ar_frame = "/ar_marker_{}".format(self.object_to_marker_num[object_name])
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

        cup_position = np.asarray([trans[0], trans[1], hard_code_pose.position.z])

        grasp_pose = Pose(cup_position, hard_code_pose.orientation)
        rospy.loginfo("found tracker positon {}".format(grasp_pose))

        pre_grasp_pose = offset_hand(grasp_pose, offset_dist=0.1)
        return pre_grasp_pose, grasp_pose

def offset_hand(grasp_pose, offset_dist=0.1):
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

def test_GQCNN():

    config = YamlConfig('/home/baymax/catkin_ws/src/jaco_manipulation/cfg/grasp_test.yaml')

    # create rgbd sensor
    rospy.loginfo('Creating RGBD Sensor')
    sensor_cfg = config['sensor_cfg']
    sensor_type = sensor_cfg['type']
    sensor = RgbdSensorFactory.sensor(sensor_type, sensor_cfg)
    sensor.start()
    rospy.loginfo('Sensor Running')

    # setup safe termination
    def handler(signum, frame):
        rospy.loginfo('caught CTRL+C, exiting...')        
        if sensor is not None:
            sensor.stop()
        if robot is not None:
            robot.stop()
        if subscriber is not None and subscriber._started:
            subscriber.stop()            
        exit(0)
    signal.signal(signal.SIGINT, handler)

    planner = GQCNNPlanner(sensor, config)
    # rospy.sleep(10)
    planner.get_grasp_plan("cup")

if __name__ == '__main__':
    rospy.init_node("Grasp_planner_test_node", log_level=rospy.DEBUG)
    test_GQCNN()
    
    rospy.spin()