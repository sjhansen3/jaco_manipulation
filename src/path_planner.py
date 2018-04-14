#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotState, PlanningScene, CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header

import sys
import numpy as np

class PathPlanner(object):
    def __init__(self):
        """
        Path Planner Class.
        References:
        dynamicreplanning.weebly.com,
        moveit python interface tutorial,
        trac_ik python tutorial
        jaco_manipulation
        """
        rospy.loginfo("To stop project CTRL + C")
        rospy.on_shutdown(self.shutdown)

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Instatiate the move group
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.group.set_planning_time(5)
        self.group.set_workspace([-3, -3, -3, 3, 3, 3])

        # This publishes trajectories to RVIZ for visualization
        self.display_planned_path_publisher = rospy.Publisher('arm/display_planned_path', DisplayTrajectory, queue_size=10)

        # This publishes updates to the planning scene
        self.planning_scene_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # Planning scene
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # RobotCommander
        self.robot = moveit_commander.RobotCommander()

        # IK Solver with trac_ik
        # NOTE: Get this from the MoveGroup so it's adaptable to other robots
        self.ik_solver = IK('root', 'm1n6s300_end_effector')
        self.ik_default_seed = [0.0] * self.ik_solver.number_of_joints
        
        # FK Solver
        rospy.wait_for_service('/compute_fk')
        self.fk_solver = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        rospy.sleep(2)        

        rate = rospy.Rate(10)

    def shutdown(self):
        rospy.loginfo("Stopping project")
        rospy.sleep(1)

    def plan_to_config(self, end_state):
        """
        Uses MoveIt to plan a path from the current state to end_state and returns it
        end_state: list of 6 joint values
        """

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        # the robot has a dumb base_link joint that we don't want
        joint_state.name = self.group.get_joints()[1:-1]
        print joint_state.name
        joint_state.position = end_state

        self.group.set_start_state_to_current_state()

        try:
            self.group.set_joint_value_target(joint_state)
        except moveit_commander.MoveItCommanderException:
            pass
        
        # Plan the path
        plan = self.group.plan()

        return plan

    def execute_path(self, path, wait_bool = True):
        """
        Executes a provided path.
        Note that the current position must be the same as the path's initial position.
        This is currently not checked.
        """

        self.group.execute(path, wait=wait_bool)

        # def collision_free_move_pose(self, end_pose):
        #     """
        #     Uses MoveIt to plan a path from the current state to end effector pose end_pose
        #     end_pose: a PoseStamped object for the end effector
        #     """

        #     self.group.set_start_state_to_current_state()
        #     self.group.set_joint_value_target(end_pose)

        #     self.group.set_workspace([-3, -3, -3, 3, 3, 3])

        #     plan = self.group.plan()

        #     return plan

    def move_home(self):
        """
        Uses MoveIt to plan a path from the current state to the home position
        """

        self.group.set_start_state_to_current_state()
        self.group.set_named_target('Home')

        self.group.set_workspace([-3, -3, -3, 3, 3, 3])

        plan = self.group.plan()

        return plan

    def visualize_plan(self, plan):
        """
        Visualize the plan in RViz
        """

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = plan.points[0]
        display_trajectory.trajectory.extend(plan.points)
        self.display_planned_path_publisher.publish(display_trajectory)

    def make_pose(self, position, orientation, frame):
        """
        Creates a PoseStamped message based on provided position and orientation
        position: list of size 3
        orientation: list of size 4 (quaternion) (wxyz)
        frame: string (the reference frame to which the pose is defined)
        returns pose: a PoseStamped object
        """

        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = orientation[0]
        pose.pose.orientation.x = orientation[1]
        pose.pose.orientation.y = orientation[2]
        pose.pose.orientation.z = orientation[3]
        return pose

    def get_ik(self, pose, seed_state = None, xyz_bounds = None, rpy_bounds = None):
        """
        get_ik returns a joint configuration from an end effector pose by using the trac_ik solver
        pose: PoseStamped object. The header.frame_id should be "root", or it won't work
        seed_state: a list of size 6. Initial joint positions for the solver. Default is [0,0,0,0,0,0]
        xyz_bounds: a list of size 3. xyz bounds of the end effector in meters. This allows an approximate ik solution. Default is None
        rpy_bounds: a list of size 3. roll pitch yaw bounds of the end effector in radians. This allows an approximate ik solution. Default is None 
        returns state: a list of joint values
        """

        if seed_state is None:
            seed_state = self.ik_default_seed

        if pose.header.frame_id != self.ik_solver.base_link:
            raise ValueError("Frame ID is not the root link")

        position = pose.pose.position
        orientation = pose.pose.orientation

        print seed_state, position.x, position.y, \
            position.z, orientation.x, orientation.y, \
            orientation.z, orientation.w \

        if xyz_bounds is None or rpy_bounds is None:
            state = self.ik_solver.get_ik(seed_state, position.x, 
                position.y, position.z, orientation.x, orientation.y, 
                orientation.z, orientation.w)
        else:
            state = self.ik_solver.get_ik(seed_state, position.x, 
                position.y, position.z, orientation.x, orientation.y, 
                orientation.z, orientation.w, xyz_bounds[0], 
                xyz_bounds[1], xyz_bounds[2], rpy_bounds[0], 
                rpy_bounds[1], rpy_bounds[2])

        return state

    def get_fk(self, joints):
        """
        Gets forward kinematics to the end effector
        joints: size 6 list. Joint angles for desired pose
        returns pose: StackedPose of the end effector in the 'root' frame
        """

        header = Header()
        header.frame_id = self.ik_solver.base_link

        robot_state = RobotState()
        robot_state.joint_state.name = self.ik_solver.joint_names
        robot_state.joint_state.position = joints

        links = [self.ik_solver.tip_link]

        return self.fk_solver(header, links, robot_state).pose_stamped[0]

def add_arbitrary_obstacle(size, id, pose, planning_scene_publisher, scene, robot, operation):
    """
    Adds an arbitrary rectangular prism obstacle to the planning scene.
    This is currently only for the ground plane
    size: numpy array of size 3 (x,y,z dimensions)
    id: string (object id/name)
    pose: PoseStamped objecct (objects pose with respect to world frame)
    planning_scene_publisher: ros Publisher('/collision_object', CollisionObject)
    scene: PlanningSceneInterface
    robot: RobotCommander
    operation: currently just use CollisionObject.ADD
    """

    co = CollisionObject()
    co.operation = operation
    co.id = id
    co.header = pose.header
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = size
    co.primitives = [box]
    co.primitive_poses = [pose.pose]
    planning_scene_publisher.publish(co)

def test_planning():
    """
    Move back and forth between two points
    """

    joints1 = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]
    joints2 = [4.80, 2.92, 1.00, 4.20, 1.45, 1.32]


    path_planner = PathPlanner()

    raw_input("Press Enter to move to home")
    plan = path_planner.move_home()
    path_planner.execute_path(plan)
    rospy.sleep(0.5)

    while True:
        raw_input("Press Enter to move to position 1")
        plan = path_planner.plan_to_config(joints1)
        path_planner.execute_path(plan)
        rospy.sleep(0.5)

        raw_input("Press Enter to move to position 2")
        plan = path_planner.plan_to_config(joints2)
        path_planner.execute_path(plan)
        rospy.sleep(0.5)

def test_ik():
    """
    Tests the IK solver
    """

    path_planner = PathPlanner()

    raw_input("Press Enter to move to home")
    plan = path_planner.move_home()
    path_planner.execute_path(plan)
    rospy.sleep(0.5)

    joints_old = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]
    pose = path_planner.get_fk(joints_old)

    raw_input("Press Enter to run inverse kinematics")
    joints = path_planner.get_ik(pose)

    print 'input:' + str(joints_old)
    print 'solution:' + str(joints)

    raw_input("Press Enter to move to position")
    plan = path_planner.plan_to_config(joints)
    path_planner.execute_path(plan)
    rospy.sleep(0.5)

def test_fk():
    """
    Tests that you can get a pose from a known valid configuration
    """

    joints = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0]

    path_planner = PathPlanner()

    pose = path_planner.get_fk(joints)

    print pose

if __name__ == '__main__':
    rospy.init_node('kinova_controller')

    # if len(sys.argv) > 1:
    #     filename = sys.argv[1]

    test_ik()





