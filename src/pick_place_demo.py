#!/usr/bin/env python
import rospy

from moveit_interface import RobotPlanner, GripController
from grasp_planner import GQCNNPlanner
import numpy as np
import signal

from autolab_core import YamlConfig

import perception

from sensor_msgs.msg import Image

class PickPlaceDemo:
    def __init__(self, grasp_planner, path_planner, grip_controller, config):
        self.grasp_planner = grasp_planner
        self.path_planner = path_planner
        self.grip_controller = grip_controller

        self.config = config
        self.image_sub = rospy.Subscriber(config["sensor_cfg"]["color_topic"], Image, self._cache_image)
        self.depth_sub = rospy.Subscriber(config["sensor_cfg"]["depth_topic"], Image, self._cache_depth)
    
        rospy.loginfo("Waiting for camera color_topic: {} to come up".format(config["sensor_cfg"]["color_topic"]))
        rospy.wait_for_message(config["sensor_cfg"]["color_topic"], Image)
        
        rospy.loginfo("Waiting for camera color_topic: {} to come up".format(config["sensor_cfg"]["depth_topic"]))
        rospy.wait_for_message(config["sensor_cfg"]["depth_topic"], Image)

        rospy.loginfo("Pick place demo initialized")

    def _plan_and_execute(self, location, pause_message=None):
        """ plan and execute a path to a location with optional paus message
        """
        self.robot_planner.plan(location)
        if pause_message:
            data = raw_input(pause_message)
            if data == 'q':
                return False
        return self.robot_planner.execute()

    def _cache_image(self, image_msg):
        """ subscribe to image topic and keep it up to date
        """
        try:
            color_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
 
        color_arr = copy.deepcopy(np.array(color_image)) #TODO should this specify type as well?
        self.color_image = ColorImage(color_arr[:,:,:3], self.image_frame)
 
    def _cache_depth(self, image_msg):
        """ subscribe to depth image topic and keep it up to date
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(e)
 
        depth = copy.deepcopy(np.array(cv_image, np.float32))
        depth[np.isinf(depth)] = 1000
        self.depth_image = DepthImage(depth, self.image_frame)

    def _get_bounding_box(self, object_name, color_image):
        """ Find the bounding box for the object
        params
        ---
        object_name: the string name of the object
        color_image: the color image for the object

        returns
        ---
        bounding_box: numpy array [minX, minY, maxX, maxY] in pixels around the image 
        on the depth image
        """
        return np.array([120,120,280,280])

    def move_object(self, object_name):
        """ A somewhat ugly function which runs pick and place using the AR trackers
        """
        # color_image, depth_image, _ = self.camera.frames()
        color_image = self.color_image
        depth_image = self.depth_image
        
        bounding_box = self._get_bounding_box(object_name, color_image)
        pregrasp_pose, grasp_pose = grasp_planner.get_grasp_plan(bounding_box, color_image, depth_image)
        
        self.grip_controller.grip("percent",[0,0,0])

        home_grip_msg = "plan to grasp_pre_hardcode commplete, anykey and enter to execute"
        self._plan_and_execute("home_grip", pause_message=home_grip_msg)

        self._plan_and_execute(pregrasp_pose)

        self._plan_and_execute(grasp_pose)

        #grip object
        #raw_input("grip object? anykey to execute")
        self._grip_controller.grip("percent",[75,75,75])
        
        #target pre
        self._plan_and_execute("grasp_target_pre")

        #target location
        self._plan_and_execute("grasp_target")

        #release object
        #raw_input("release object? anykey to execute")
        self._grip_controller.grip("percent",[0,0,0])
        rospy.sleep(1)

        self._plan_and_execute("home_grip")

if __name__ == '__main__':
    rospy.init_node("pick_place_demo", log_level=rospy.DEBUG)
    config = YamlConfig('/home/baymax/catkin_ws/src/jaco_manipulation/cfg/grasp_test.yaml')

    # create rgbd sensor
    rospy.loginfo('Creating RGBD Sensor')
    sensor_cfg = config['sensor_cfg']
    sensor_type = sensor_cfg['type']
    camera = perception.RgbdSensorFactory.sensor(sensor_type, sensor_cfg)
    camera.start()
    rospy.loginfo('Sensor Running')
    
    sensor = RgbdSensorFactory.sensor(sensor_type, sensor_cfg)

    # setup safe termination
    def handler(signum, frame):
        rospy.loginfo('caught CTRL+C, exiting...')        
        # if camera is not None:
        #     # camera.stop()
        #TODO fix subscriber to be real
        if subscriber is not None and subscriber._started:
            subscriber.stop()            
        exit(0)
    signal.signal(signal.SIGINT, handler)

    frame = config['sensor_cfg']['frame']

    camera_intrinsics = perception.CameraIntrinsics(frame, fx=365.46, fy=365.46, cx=254.9, cy=205.4, skew=0.0, height=424, width=512) #TODO set height and width with param
    grasp_planner = GQCNNPlanner(camera_intrinsics, config)

    path_planner = RobotPlanner()
    grip_controller = GripController()
    object_name = "cup"

    picker = PickPlaceDemo(grasp_planner, path_planner, grip_controller, config)
    picker.move_object(object_name)

    rospy.spin()