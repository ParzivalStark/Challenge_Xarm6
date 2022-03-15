#!/usr/bin/env python
import string
import rospy
import sys
import tf_conversions
import tf2_ros
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
from path_planner.srv import *
from tf.transformations import *
from moveit_msgs.msg import Grasp

class Planner():

  def __init__(self):
    #TODO: Initialise move it interface
    moveit_commander.roscpp_initialize(sys.argv)

    self.scene = moveit_commander.PlanningSceneInterface()
    self.arm_group = moveit_commander.MoveGroupCommander("xarm6")
    self.eef_group = moveit_commander.MoveGroupCommander("xarm_gripper")

    self.arm_group.allow_replanning(True)
    self.arm_group.set_goal_position_tolerance(0.005)

    self.attach_service = rospy.ServiceProxy("/AttachObject", AttachObject)

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):

    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
    pass

  def addObstacles(self):

    #TODO: Add obstables in the world
    rospy.loginfo("Adding Obstacles")
    #Cargo names
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]
    #goal names
    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"]

    '''self.box_pose = PoseStamped()
    self.box_pose.header.frame_id = "RedBox"
    box_name = "red_box"
    self.scene.add_box(box_name, self.box_pose, size=(0.06,0.06,0.06))'''

    rospy.sleep(2)

    for target in targets:
      box_pose = PoseStamped()
      box_pose.header.frame_id = target
      self.scene.add_box(target.lower(), box_pose, size=(0.06,0.06,0.06))

    for box in boxes:
      box_pose = PoseStamped()
      direction = 1
      for i in range(4):
        box_pose = PoseStamped()
        box_pose.header.frame_id = box
        len_ = [0.359288, 0.017976]
        wid_ = [0.017976, 0.157]
        if i % 2 == 0:
          box_pose.pose.position.y += (wid_[1]/2-len_[1]/2)*direction
        else:
          box_pose.pose.position.x += (len_[0]/2+len_[1]/2)*direction
          direction *= -1
        self.scene.add_box(box.lower()+str(i), box_pose, size=(len_[i%2],wid_[i%2],0.109964))

  def goToPose(self,pose_goal):

    #TODO: Code used to move to a given position using move it
    self.arm_group.set_pose_target(pose_goal)
    self.arm_group.go(wait=True)
    self.arm_group.stop()
    self.arm_group.clear_pose_targets()


  def detachBox(self,box_name):
    #TODO: Open the gripper and call the service that releases the box
    self.attach_service(False, box_name)


  def attachBox(self,box_name):
    #TODO: Close the gripper and call the service that releases the box
    self.attach_service(True, box_name)



class myNode():
  def __init__(self):
    #TODO: Initialise ROS and create the service calls
    rospy.init_node("Solution")
    # Good practice trick, wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    self.planner  = Planner()
    self.home     = self.planner.arm_group.get_current_pose()

  def getGoal(self,action):

    #TODO: Call the service that will provide you with a suitable target for the movement
    try:
        request_goal = rospy.ServiceProxy("/RequestGoal", RequestGoal)
        self.objective = request_goal(action)
    except rospy.ServiceException as e:
        rospy.logwarn(e)


  def tf_goal(self, goal):

    #TODO:Use tf2 to retrieve the position of the target with respect to the proper reference frame
    result = self.tfBuffer.lookup_transform("world", goal, rospy.Time(), rospy.Duration(0.5))
    return result.transform.translation

  def goal_pos(self, goal, height = 0):
    pose_goal = Pose()
    pose_goal.position = self.tf_goal(goal)
    pose_goal.position.z += height
    pose_goal.orientation = self.home.pose.orientation
    return pose_goal


  def main(self):
    #TODO: Main code that contains the aplication
    self.planner.addObstacles()
    print(self.home)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
      self.getGoal('pick')
      if(self.objective.goal != "End"):
        cube = self.objective.goal
        transform = self.goal_pos(cube, -0.02)
        self.planner.goToPose(transform)
        self.planner.attachBox(cube)

        self.getGoal('place')
        transform = self.goal_pos(self.objective.goal)
        self.planner.goToPose(transform)
        self.planner.detachBox(cube)
      else:
        self.planner.goToPose(self.home.pose)
        finish = self.home.pose
        finish.position.z = 0.01
        self.planner.goToPose(finish)
        yeeeet = self.goal_pos("DepositBoxBlue")
        self.planner.goToPose(yeeeet)
        rospy.signal_shutdown("Task Completed")
      rate.sleep()

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
