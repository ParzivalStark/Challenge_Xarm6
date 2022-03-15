#!/usr/bin/env python

import string
import sys
import copy

from numpy import size
import tf_conversions
import tf2_ros
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from tf.transformations import *

from math import pi, radians

tau = 2.0 * pi

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from path_planner.srv import RequestGoal

def call_RequestGoal_service(action):
    try:
        request_goal = rospy.ServiceProxy("/RequestGoal", RequestGoal)
        objective = request_goal(action)
        rospy.loginfo(objective)
        return objective
    except rospy.ServiceException as e:
        rospy.logwarn(e)

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test')

    rospy.wait_for_service("/RequestGoal")

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "xarm6"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.allow_replanning(True)
    move_group.set_goal_position_tolerance(0.005)
    move_group.set_goal_orientation_tolerance(0.05)

    gripper = "xarm_gripper"
    move_gripper = moveit_commander.MoveGroupCommander(gripper)

    #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    result = tfBuffer.lookup_transform("world", "GreenBox", rospy.Time(), rospy.Duration(0.5))
    rospy.loginfo(result.transform)

    #print(robot.get_current_state())

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "RedBox"
    box_name = "red_box"
    scene.add_box(box_name, box_pose, size=(0.06,0.06,0.06))

    '''planning_frame = move_group.get_planning_frame()
    print("========= Planning frame: %s" % planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("========= End effector link: %s" %eef_link)

    group_names = robot.get_group_names()
    print("========= Available Planning Groups:", robot.get_group_names())
    print(robot.get_current_state())
    print("")

    joint_goal = move_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = 0
    joint_goal[1] = radians(-20)
    joint_goal[2] = radians(-51)
    joint_goal[3] = 0
    joint_goal[4] = radians(71)
    joint_goal[5] = 0

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    response = call_RequestGoal_service('pick')
    rospy.loginfo(response.goal)
    response = call_RequestGoal_service('place')
    


    pose_goal = geometry_msgs.msg.Pose()
    #pose_goal.orientation = result.transform.rotation
    pose_goal.orientation.x = 1.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.0
    pose_goal.position = result.transform.translation
    #pose_goal.position.x = result.transform.translation.x
    #pose_goal.position.y = result.transform.translation.y
    #pose_goal.position.z = result.transform.translation.z

    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    print("Move 1 done")

    print(robot.get_current_state())

    joint_goal = move_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = 0    #joint1
    joint_goal[1] = radians(-20)    #joint2
    joint_goal[2] = radians(-51)    #joint3
    joint_goal[3] = 0   #joint4
    joint_goal[4] = radians(71) #joint5
    joint_goal[5] = 0   #joint6

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    print("Move 2 done")'''

    gripper_goal = move_gripper.get_current_joint_values()
    print(gripper_goal)
    gripper_goal[0] = radians(15)   #drive_joint
    gripper_goal[1] = radians(15)   #left_finger
    gripper_goal[2] = radians(15)   #left_inner
    gripper_goal[3] = radians(15)   #right_inner
    gripper_goal[4] = radians(15)   #right_outter
    gripper_goal[5] = radians(15)   #right_finger

    move_gripper.go(gripper_goal, wait=True)

    move_gripper.stop()

    print("Gripper closed")
