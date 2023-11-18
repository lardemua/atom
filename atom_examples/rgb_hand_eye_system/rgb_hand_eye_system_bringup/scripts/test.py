#!/usr/bin/env python3

'''
example on how to control specific arm joints with moveit
'''

import sys
import rospy
import moveit_commander

rospy.init_node('move_joints_test', anonymous=False)

try:
    rospy.loginfo('waiting for move_group action server')
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    # robot.arm.set_planning_time(20.0)
    # robot.arm.set_goal_tolerance(0.01)
    rospy.loginfo('found move_group action server')
except RuntimeError:
    # moveit_commander.roscpp_initialize overwrites the signal handler,
    # so if a RuntimeError occurs here, we have to manually call
    # signal_shutdown() in order for the node to properly exit.
    rospy.logfatal('could not connect to Moveit in time, exiting! \n')
    rospy.signal_shutdown('fatal error')

# to test if arm and moveit work
# arm_posture_name = 'observe100cm_right'
# robot.arm.set_named_target(arm_posture_name)
# robot.arm.go()

# give only the arm joints that we want to affect, NOTE: for some reason this does not work!
# arm_dic = {'mobipick/ur5_shoulder_pan_joint':-1.25, 'mobipick/ur5_elbow_joint':1.8}

# get current joint angles
joint_states = robot.arm.get_current_state().joint_state

arm_joints_dic = {}


joints_of_interest = ['robot/shoulder_pan_joint', 'robot/shoulder_lift_joint',
                      'robot/elbow_joint', 'robot/wrist_1_joint', 'robot/wrist_2_joint', 'robot/wrist_3_joint']
for joint_of_interest in joints_of_interest:
    # find index
    for i, joint_name in enumerate(joint_states.name):
        if joint_of_interest == joint_name:
            # print(f'{joint_of_interest}:{joint_states.position[i]}') # uncomment for debug purposes
            arm_joints_dic[joint_of_interest] = joint_states.position[i]

# rospy.logwarn(f'current arm state: {robot.arm.get_current_state().joint_state}')

# modify required joints
arm_joints_dic['robot/shoulder_pan_joint'] = 1.7

robot.arm.set_joint_value_target(arm_joints_dic)
robot.arm.go()

rospy.spin()
