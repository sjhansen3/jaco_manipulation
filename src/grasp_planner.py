#!/usr/bin/env python

import numpy as np
import tf
import rospy
import spacial_location
from spacial_location import Pose #TODO replace this with full reference, too many poses to use simply Pose
import geometry_msgs.msg

from std_msgs.msg import Header
from gqcnn.srv import GQCNNGraspPlanner
from gqcnn.msg import GQCNNGrasp, BoundingBox

from sensor_msgs.msg import Image, CameraInfo

from perception import CameraIntrinsics, ColorImage, DepthImage

from cv_bridge import CvBridge, CvBridgeError

from gqcnn import Visualizer as vis
import cv2
import matplotlib.pyplot as plt
import copy

from autolab_core import Box

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
    def __init__(self):
        # wait for Grasp Planning Service and create Service Proxy
        rospy.loginfo("Waiting for GQCNN to spin up")
        rospy.wait_for_service('plan_gqcnn_grasp')
        self.plan_grasp = rospy.ServiceProxy('plan_gqcnn_grasp', GQCNNGraspPlanner)
        rospy.loginfo("GQCNN service Initialized")

        self.image_frame = "zed_center" #TODO make this a param
        # get camera intrinsics
        self.camera_intrinsics = CameraIntrinsics(self.image_frame, fx=1, fy=1, cx=0.0, cy=0.0, skew=0.0, height=720, width=1280) #TODO set height and width with param
        
        self.depth_image = None
        self.color_image = None

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/zed/left/image_rect_color", Image, self.cache_image) #TODO put camera topic in YAML
        self.depth_sub = rospy.Subscriber("/zed/depth/depth_registered", Image, self.cache_depth) #TODO put camera topic in YAML
        


    def get_bounding_box(self, object_name):
        boundingBox = BoundingBox()
        boundingBox.minY = 0
        boundingBox.minX = 0
        boundingBox.maxY = 1280
        boundingBox.maxX = 720
        return boundingBox

    def get_grasp_plan(self, object_name):
        boundingBox = self.get_bounding_box(object_name)
        
        if not self.color_image or not self.depth_image:
            rospy.logwarn("Grasp plan called before image data was available")

        print("Is the original depth image finite?", np.all(np.isfinite(self.depth_image.data)))
        self.depth_image.data[np.isinf(self.depth_image.data)] = 1000
        np.where(np.inf)
        
        pyplot.figure()

        pyplot.subplot(2,3,1)
        pyplot.title("depth image")
        pyplot.imshow(self.depth_image.data)

        pyplot.subplot(2,3,2)
        pyplot.title("where greater than 1000")
        pyplot.imshow(np.isfinite(self.depth_image.data))

        pyplot.subplot(2,3,3) 
        pyplot.title("color image")
        pyplot.imshow(self.color_image.data)
        # max_xy_coords = np.array([720, 1280])
        # min_xy_coords = np.array([0, 0])
        # bbox = Box(min_xy_coords, max_xy_coords)
        # pyplot.box(bbox)

        # rospy.loginfo("Is the original depth image finite?", np.all(np.isfinite(self.depth_image.data)))

        # inpaint to remove holes
        inpainted_color_image = self.color_image.inpaint(rescale_factor=0.5) #TODO make rescale factor in config
        inpainted_depth_image = self.depth_image.inpaint(rescale_factor=0.5)

        pyplot.subplot(2,3,4)
        pyplot.title("depth image")
        pyplot.imshow(self.depth_image.data)

        pyplot.subplot(2,3,5)
        pyplot.title("inpaint color")
        pyplot.imshow(inpainted_color_image.data)

        pyplot.subplot(2,3,6)
        pyplot.title("inpaint depth")
        pyplot.imshow(inpainted_depth_image.data)
        pyplot.show()
        


        print("Is the inpainted depth image finite?", np.all(np.isfinite(inpainted_depth_image.data)))



        try:
            rospy.loginfo("Sending grasp plan")
            planned_grasp_data = self.plan_grasp(inpainted_color_image.rosmsg, inpainted_depth_image.rosmsg, self.camera_intrinsics.rosmsg, boundingBox)
            rospy.loginfo("grasp plan complete")

            print planned_grasp_data
            # lift_gripper_width, T_gripper_world = process_GQCNNGrasp(planned_grasp_data, robot, left_arm, right_arm, subscriber, home_pose, config)

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: \n %s" % e)  
    
    def cache_image(self, image_msg):
        """ subscribe to image topic and keep it up to date
        """
        try:
            color_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        color_arr = copy.deepcopy(np.array(color_image)) #TODO should this specify type as well?
        self.color_image = ColorImage(color_arr[:,:,:3], self.image_frame)

    def cache_depth(self, image_msg):
        """ subscribe to depth image topic and keep it up to date
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(e)

        depth = copy.deepcopy(np.array(cv_image, np.float32))
        depth[np.isinf(depth)] = 1000
        self.depth_image = DepthImage(depth, self.image_frame)

    # def process_GQCNNGrasp(grasp, robot, left_arm, right_arm, subscriber, home_pose, config):
    # """ Processes a ROS GQCNNGrasp message and executes the resulting grasp on the ABB Yumi """
    
    #     grasp = grasp.grasp
    #     rospy.loginfo('Processing Grasp')

    #     rotation_quaternion = np.asarray([grasp.pose.orientation.w, grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z]) 
    #     translation = np.asarray([grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])

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

def test_GQCNN():
    # rospy.sleep(5)
    planner = GQCNNPlanner()
    rospy.sleep(10)
    planner.get_grasp_plan("cup")

if __name__ == '__main__':
    rospy.init_node("Grasp_planner_test_node", log_level=rospy.DEBUG)
    test_GQCNN()
    
    #planner = ARTrackPlanner()
    #planner.get_grasp_plan("cup")
    rospy.spin()