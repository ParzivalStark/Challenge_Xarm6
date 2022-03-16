#!/usr/bin/env python

# Import neccesary libraries
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
from math import radians


# Class for moveit commander
class Planner():
  # Method to initialize Planner class
  def __init__(self):
    # Initialize move it interface
    moveit_commander.roscpp_initialize(sys.argv)

    # Instantiate a RobotCommander object. (Information of the robot)
    self.robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object. (Information of the surrounding)
    self.scene = moveit_commander.PlanningSceneInterface()

    # Instantiate MoveGroupCommander objects (Plan and execute motions)
    self.arm_group = moveit_commander.MoveGroupCommander("xarm6")
    self.eef_group = moveit_commander.MoveGroupCommander("xarm_gripper")

    # Allow replanning of the motions
    self.arm_group.allow_replanning(True)
    # Adjust precision of the planning
    self.arm_group.set_goal_position_tolerance(0.00001)

    # Initialize client for the service AtthacObject
    self.attach_service = rospy.ServiceProxy("/AttachObject", AttachObject)

    # Save initial joint values of the gripper
    self.openGripper = self.eef_group.get_current_joint_values()

  def wait_for_state_update(self,box_name, box_is_known=False, box_is_attached=False, timeout=0.5):

    #TODO: Whenever we change something in moveit we need to make sure that the interface has been updated properly
    pass

  # Planner method designed for adding the obstacles inside the rviz interface
  # for moveit to avoid them when path planning
  def addObstacles(self):

    rospy.loginfo("Adding Obstacles")

    #Cargo names
    targets = ["RedBox",
               "BlueBox",
               "GreenBox"]

    #goal names
    boxes = ["DepositBoxGreen",
               "DepositBoxRed",
               "DepositBoxBlue"]

    # Sleep for 2 seconds in order to allow the planning scene interface to load correctly
    rospy.sleep(2)

    # For loop to create all the target boxes
    for target in targets:
      # Pose variable for the target box in the tf tree
      box_pose = PoseStamped()
      box_pose.header.frame_id = target
      # Add box with specified position and size to the planning scene interface
      self.scene.add_box(target.lower(), box_pose, size=(0.06,0.06,0.06))

    # For loop to create the goal containers
    for box in boxes:
      # Pose variable for the goal container in the tf tree
      box_pose = PoseStamped()
      # Variable for correct wall positioning
      direction = 1
      # Loop for creating all 4 container's walls
      for i in range(4):
        box_pose = PoseStamped()
        box_pose.header.frame_id = box
        # Walls' dimensions
        len_ = [0.359288, 0.017976]
        wid_ = [0.017976, 0.157]
        # Condition to know if the current wall is vertical or horizontal
        if i % 2 == 0:
          box_pose.pose.position.y += (wid_[1]/2-len_[1]/2)*direction
        else:
          box_pose.pose.position.x += (len_[0]/2+len_[1]/2)*direction
          direction *= -1
        # Add container with specified position and size to the planning scene interface
        self.scene.add_box(box.lower()+str(i), box_pose, size=(len_[i%2],wid_[i%2],0.109964))
    
    # All obstacles added correctly
    rospy.loginfo("Obstacles Added")

  # Planner method that plans the path for the robot's motion and executes
  # the corresponding joints motions to allow it to reach a specific frame
  def goToPose(self,pose_goal):
    # Instruction that sets the current target
    self.arm_group.set_pose_target(pose_goal)
    # Instruction to start the robot's motion and wait until it has finished
    # the planed trajectory
    self.arm_group.go(wait=True)
    # Instruction that stops the robot's motion
    self.arm_group.stop()
    # Instruction for clearing the recently reached target
    self.arm_group.clear_pose_targets()

  # Planner method to detach boxes from the gripper
  def detachBox(self,box_name):
    # Open the gripper using initial joint values
    self.eef_group.go(self.openGripper, wait=True)

    # Instruction that stops the robot's motion
    self.eef_group.stop()
    
    # Call the service to deatach box in gazebo
    self.attach_service(False, box_name)

    # Use moveit to detach box in rviz
    self.scene.remove_attached_object('xarm_gripper_base_link', name=box_name.lower())

  # Planner method to attach boxes to the gripper
  def attachBox(self,box_name):
    # Get joint values
    gripper_goal = self.eef_group.get_current_joint_values()
    # Modify joint values to close the gripper
    gripper_goal[0] = radians(10)   #drive_joint
    gripper_goal[1] = radians(10)   #left_finger
    gripper_goal[2] = radians(10)   #left_inner
    gripper_goal[3] = radians(10)   #right_inner
    gripper_goal[4] = radians(10)   #right_outter
    gripper_goal[5] = radians(10)   #right_finger

    # Instruction to start the gripper's motion and wait until it has finished
    # the planed trajectory
    self.eef_group.go(gripper_goal, wait=True)

    # Instruction that stops the gripper's motion
    self.eef_group.stop()

    # Call the service that attach the box in gazebo
    self.attach_service(True, box_name)

    # Use moveit to attach box in rviz
    touch_links = self.robot.get_link_names(group='xarm_gripper')
    self.scene.attach_box('xarm_gripper_base_link', box_name.lower(), touch_links=touch_links)


# Class for the ros node
class myNode():
  # Method to initialize Planner class
  def __init__(self):
    # Initialization of the ros node "Solution"
    rospy.init_node("Solution")
    # Wait until the required services are online before continuing with the aplication
    rospy.wait_for_service('RequestGoal')
    rospy.wait_for_service('AttachObject')

    # Instance of ros tf2 buffer
    self.tfBuffer = tf2_ros.Buffer()
    # Listener for the recently instanciated tf2 buffer
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # Instance of the planner class
    self.planner  = Planner()
    # Saving the starting position for returning home at the end
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
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
      self.getGoal('pick')
      if(self.objective.goal != "End"):
        cube = self.objective.goal
        transform = self.goal_pos(cube, 0.1)
        self.planner.goToPose(transform)
        transform = self.goal_pos(cube, -0.02)
        self.planner.goToPose(transform)
        self.planner.attachBox(cube)
        transform = self.goal_pos(cube, 0.1)
        self.planner.goToPose(transform)

        self.getGoal('place')
        transform = self.goal_pos(self.objective.goal, 0.1)
        self.planner.goToPose(transform)
        transform = self.goal_pos(self.objective.goal)
        self.planner.goToPose(transform)
        self.planner.detachBox(cube)
        transform = self.goal_pos(self.objective.goal, 0.1)
        self.planner.goToPose(transform)
        
      else:
        self.planner.goToPose(self.home.pose)
        '''finish = self.home.pose
        finish.position.z = 0.01
        self.planner.goToPose(finish)
        yeeeet = self.goal_pos("DepositBoxBlue")
        self.planner.goToPose(yeeeet)'''
        rospy.signal_shutdown("Task Completed")
      rate.sleep()

if __name__ == '__main__':
  try:
    node = myNode()
    node.main()

  except rospy.ROSInterruptException:
    pass
