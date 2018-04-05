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
import std_msgs.msg
from sensor_msgs.msg import Image, CameraInfo

from perception import CameraIntrinsics, ColorImage, DepthImage
from perception import RgbdDetectorFactory, RgbdSensorFactory

from autolab_core import YamlConfig, RigidTransform
from autolab_core import Box
# import autolab_core.rigid_transformations.RigidTransform as RigidTransform

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
        self.listener = tf.TransformListener()

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

    def _get_new_pose(self, pose_stamped, target_frame="/world"):
        """ Transform a pose from its frame to the desired frame
        Params
        ----
        pose_stamped: A ros stamped pose with its reference frame specified
        target_frame: The desired frame of the robot
        Returns
        ----
        pose_world_stamped: The pose in the world frame
        """
        self.listener.waitForTransform(target_frame, self.camera_intrinsics.frame, rospy.Time(0), rospy.Duration(4.0))
        try:
            pose_world_stamped = self.listener.transformPose(target_frame, pose_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tf was not able to find a transformation between the world frame and the grasp point")
            return False

        return pose_world_stamped

    def _get_transformation(self, from_frame, to_frame="world"):
        target_frame = to_frame
        source_frame = from_frame
        self.listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
        try:
            pos, quat_out = self.listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("tf was not able to find a transformation from {} to {}".format(from_frame, to_frame))
            return False
        #import pdb; pdb.set_trace()
        quat = [quat_out[3], quat_out[0], quat_out[1], quat_out[2]] #the quaternion of ros has a different representation than other sys
        return RigidTransform(quat, pos, from_frame, to_frame)
        
    def get_grasp_plan(self, bounding_box, color_image, depth_image):
        """ finds the highest quality score grasp associated with the object

        """
        #grab frames for depth image and color image
        rospy.loginfo("grabbing frames")

        boundingBox = self._bbox_to_msg(bounding_box)
        
        pyplot.figure()
        pyplot.subplot(2,2,1)
        pyplot.title("color image")
        pyplot.imshow(color_image.data)

        pyplot.subplot(2,2,2)
        pyplot.title("depth image")
        pyplot.imshow(depth_image.data)

        # inpaint to remove holes
        inpainted_color_image = color_image.inpaint(self.config["inpaint_rescale_factor"]) #TODO make rescale factor in config
        inpainted_depth_image = depth_image.inpaint(self.config["inpaint_rescale_factor"])

        pyplot.subplot(2,2,3)
        pyplot.title("inpaint color")
        pyplot.imshow(inpainted_color_image.data)

        pyplot.subplot(2,2,4)
        pyplot.title("inpaint depth")
        pyplot.imshow(inpainted_depth_image.data)
        pyplot.show()
        
        try:
            rospy.loginfo("Sending grasp plan request to gqcnn server")
            planned_grasp_data = self.plan_grasp(inpainted_color_image.rosmsg, inpainted_depth_image.rosmsg, self.camera_intrinsics.rosmsg, boundingBox)

            planned_grasp_pose_msg = planned_grasp_data.grasp.pose
            grasp_succes_prob = planned_grasp_data.grasp.grasp_success_prob
            grasp = planned_grasp_data.grasp

            rospy.loginfo("Grasp service request response: {}".format(planned_grasp_data))

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: \n %s" % e)  

        rotation_quaternion = np.asarray([grasp.pose.orientation.w, grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z]) 
        translation = np.asarray([grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
        T_grasp_camera = RigidTransform(rotation_quaternion, translation, "grasp_point_test", self.camera_intrinsics.frame)
        # T_grasp_camera.publish_to_ros()
        # T_world_camera = RigidTransform.rigid_transform_from_ros(self.camera_intrinsics.frame, "world")
        T_world_camera = self._get_transformation(self.camera_intrinsics.frame)

        #ValueError: To frame of right hand side (grasp_point_test) must match from frame of left hand side (world)

        # rospy.loginfo("T_camera_world {}".format(T_world_camera))
        # rospy.loginfo("T_grasp_world {}".format(T_grasp_camera))

        T_grasp_world = T_world_camera * T_grasp_camera
        # T_grasp_world.publish_to_ros()

        # # grasp_pose = Pose(translation, rotation_quaternion, frame=self.camera_intrinsics.frame)
        # grasp_pose_stamped = geometry_msgs.msg.PoseStamped()

        # header = std_msgs.msg.Header()
        # header.frame_id = self.camera_intrinsics.frame
        # header.stamp = rospy.Time(0)

        # grasp_pose_stamped.pose = planned_grasp_pose_msg
        # grasp_pose_stamped.header = header

        # grasp_pose_world_stamped = self._get_new_pose(grasp_pose_stamped)

        # # rospy.loginfo("The planned grasp with a rigid transform {}".format(T_grasp_world))
        # rospy.loginfo("The planned grasp was converted into the world frame: {}".format(grasp_pose_world_stamped))

        # g_pos = grasp_pose_world_stamped.pose.position
        # g_quat = grasp_pose_world_stamped.pose.orientation
        
        # grasp_pos = np.array([g_pos.x, g_pos.y, g_pos.z])
        # grasp_quat = np.array([g_quat.x, g_quat.y, g_quat.z, g_quat.w])

        # grasp_quat_inv = tf.transformations.quaternion_inverse(grasp_quat)

        # unit_vector = np.asarray([1,0,0])

        # approach_direction = spacial_location.qv_mult(grasp_quat_inv, unit_vector)
        # pre_grasp_pos = grasp_pos - approach_direction*0.1

        import pdb; pdb.set_trace()
        t_quat = [T_grasp_world.quaternion[1], T_grasp_world.quaternion[2], T_grasp_world.quaternion[3], T_grasp_world.quaternion[0]]
        grasp_pose_world = Pose(T_grasp_world.position, t_quat, frame="/world")
        grasp_pose_world.show_position_marker("grasp_world_transform",ident=3)

        # grasp_pose_world = Pose(grasp_pos, grasp_quat, frame=self.camera_intrinsics.frame)
        pre_grasp_pose = offset_hand(grasp_pose_world)

        # rospy.loginfo("grasp_pose_world {}".format(grasp_pose_world))
        
        # pre_grasp_pose.show_position_marker(ident = 1, label = "pre grasp")
        # grasp_pose_world.show_position_marker(ident = 2, label = "grasp")

        return pre_grasp_pose, grasp_pose_world

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

def offset_hand(pose_input, offset_dist=0.1):
    """ Find the pre grasp pose by offsetting the hand backwards from its current position
    grasp_pose: the pose of the grasp location
    offset_dist: the amount to offset off the object
    Returns
    ---
    pre_grasp_pose: the offsetted pose of the object
    """
    #use the unit z vector because thats the direction out of the hand
    unit_vector = np.asarray([1,0,0])
    quat = pose_input.orientation[0:4]

    quat_inv = tf.transformations.quaternion_inverse(quat)
    direction = spacial_location.qv_mult(quat, unit_vector)
    pre_grasp_position = pose_input.position - direction*offset_dist

    pre_grasp_pose = Pose(pre_grasp_position, pose_input.orientation)

    pose_input.show_position_marker(ident = 1, label = "grasp pose")
    pre_grasp_pose.show_position_marker(ident = 2, label = "pregrasp pose")
    # import pdb; pdb.set_trace()

    return pre_grasp_pose

def test_offset_hand():
    rospy.sleep(5)
    quat = tf.transformations.random_quaternion()
    print quat
    grasp_pose = Pose([0.5,0.5,0.5], quat)
    offset_hand(grasp_pose)

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
    test_offset_hand()
    
    #rospy.spin()