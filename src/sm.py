#!/usr/bin/env python

from geometry_msgs.msg import Twist
import roslib
import os
import rospy
import smach
import smach_ros
from smach_ros import ServiceState

from sm_states import *

# # main


def main():

    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(
        outcomes=[])#['land', 'finished', 'succeeded', 'aborted', 'preempted'])

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('interface', routine_interface(),
                                transitions={'execute': 'takeoff_routine',
                                             'takeoff': 'takeoff',
                                             'custom':'visual_servoing_land',
                                             'land':'land_now'})
        
        smach.StateMachine.add('visual_servoing_land', face_recharge_station(),
                               transitions={'done': 'interface'}) #goes to the side of the shelf 
        
        smach.StateMachine.add('takeoff', takeoff(),
                               transitions={'done': 'interface',
                                            'error': 'land_now'})
        
        smach.StateMachine.add('takeoff_routine', takeoff(),
                               transitions={'done': 'follow_routine',
                                            'error': 'land_now'})

    
        smach.StateMachine.add('follow_routine', follow_routine(),
                               transitions={'done': 'land_routine',
                                            'error':'land_now'}) 

        smach.StateMachine.add('land_routine', land_now(),
                               transitions={'done': 'interface'})



        # smach.StateMachine.add('inventory1', inventory(1),
        #                        transitions={'done': 'land_now','error': 'land_now'})

        # smach.StateMachine.add('face_flag', face_flag(),
        #                        transitions={'done': 'face_flag',
        #                                     'error':'land_now'})

        # flag_subsm = smach.StateMachine(outcomes=['success', 'error'])
        # with flag_subsm:

        #     smach.StateMachine.add('align_flag', align_flag(),
        #                            transitions={'flag_aligned': 'success',
        #                                         'reference_lost': 'change_view',
        #                                         'too_many_attempts': 'error'})

        #     smach.StateMachine.add('change_view', change_view_flag(),
        #                            transitions={'done': 'align_flag'})

        # smach.StateMachine.add('flag', flag_subsm,
        #                        transitions={'success': 'drop_box',
        #                                     'error': 'land_now'})
        
        
        
        smach.StateMachine.add('land_now', land_now(),
                               transitions={'done': 'interface'})

        # smach.StateMachine.add('face_box', face_box(),
        #                        transitions={'done': 'pickup_box'})

        # smach.StateMachine.add('pickup_box', pickup_box(),
        #                        transitions={'done': 'land_now'})
                               
        # smach.StateMachine.add('capture_flag', capture_flag(),
        #                        transitions={'done': 'face_shelf',
        #                                     'error': 'face_shelf'})


        # smach.StateMachine.add('pass_through_shelf', pass_through_shelf(),
        #                        transitions={'done': 'land_now'})

        
        # smach.StateMachine.add('drop_box', drop_box(),
        #                        transitions={'done': 'land_now'})


        # smach.StateMachine.add('inventory2', inventory(2),
        #                        transitions={'done': 'land_now','error': 'land_now'})

        
        # move_to_shelf_subsm = smach.StateMachine(outcomes=['success', 'error'])
        # with move_to_shelf_subsm:

        #     smach.StateMachine.add('align_shelf', align_shelf(),
        #                            transitions={'shelf_aligned': 'success',
        #                                         'reference_lost': 'change_view_shelf',
        #                                         'too_many_attempts': 'error'})

        #     smach.StateMachine.add('change_view_shelf', change_view_shelf(),
        #                            transitions={'done': 'align_shelf'})


        # smach.StateMachine.add('move_to_shelf', move_to_shelf_subsm ,
        #                        transitions={'success': 'face_box',
        #                                     'error': 'land_now'})


        # Create the sub SMACH state machine
        # sm_sub = smach.StateMachine(outcomes=['no_window', 'yes_window'])
        # # # Open the container
        # with sm_sub:

        #     # Add states to the container
        #     smach.StateMachine.add('find_window', find_window(),
        #                            transitions={'window found': 'yes_window',
        #                                         'not on view': 'change_view',
        #                                         'too many attempts': 'no_window'})

        #     smach.StateMachine.add('change_view', change_view(),
        #                            transitions={'view_changed': 'find_window'})

        # smach.StateMachine.add('WINDOW', sm_sub,
        #                        transitions={'no_window': 'land',
        #                                     'yes_window': 'through_window_state'})
        # smach.StateMachine.add('through_window_state', through_window_state(),
        #                        transitions={'completed window': 'land_now'})
        # smach.StateMachine.add('find_rope', find_rope(),
        #                        transitions={'rope_found': 'reset'})  # finished'})

        # smach.StateMachine.add('reset',
        #                        ServiceState('gazebo/reset_world',
        #                                     Empty),
        #                        transitions={'succeeded': 'takeoff'})

     # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()

    # rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
