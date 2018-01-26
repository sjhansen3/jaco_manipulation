#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import spacial_location

class GraspRequest:
    def __init__(self, target_location, grasp_object):
        """ request to grab an object and move it to a new target location
        params
        ---
        grasp_object: a string representing the object to be grasped
        target_object: a target pose that the object should arrive at
        """
        assert isinstance(target_location, spacial_location.Pose)
        self.grasp_object = grasp_object
        self.target_location = target_location

# define state Foo
class GetGraspRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded','aborted'],
                             output_keys=['grasp_request'])

    def execute(self, userdata):
        rospy.loginfo('Getting grasp request')
        userdata.grasp_request = 
        if  :
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter = %f'%userdata.bar_counter_in)        
        return 'outcome1'
        

def main():
    rospy.init_node('manipulation_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
