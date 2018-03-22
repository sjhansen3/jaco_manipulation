#!/usr/bin/env python

import rospy
import perception
from autolab_core import YamlConfig

from perception import RgbdDetectorFactory, RgbdSensorFactory

import signal
import matplotlib.pyplot as pyplot



def test(sensor):
    # get the images from the sensor
    rospy.loginfo("grabbing frames")
    color_image, depth_image, _ = sensor.frames()
    pyplot.figure()
    pyplot.subplot(2,2,1)
    pyplot.title("depth image")
    pyplot.imshow(depth_image.data)

    pyplot.subplot(2,2,2) 
    pyplot.title("color image")
    pyplot.imshow(color_image.data)
    pyplot.show()

if __name__ == '__main__':
    # initialize the ROS node
    rospy.init_node('Grasp_planning_test_node', log_level=rospy.DEBUG)

    config = YamlConfig('/home/baymax/catkin_ws/src/jaco_manipulation/cfg/grasp_test.yaml')
    rospy.loginfo("config init")

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

    #Run test
    test(sensor)

    rospy.spin()