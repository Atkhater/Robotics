#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

try:
   from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
   from math import pi, fabs, cos, sqrt


   tau = 2.0 * pi


   def dist(p, q):
       return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
   """
   Convenience method for testing if the values in two lists are within a tolerance of each other.
   For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
   between the identical orientations q and -q is calculated correctly).
   @param: goal       A list of floats, a Pose or a PoseStamped
   @param: actual     A list of floats, a Pose or a PoseStamped
   @param: tolerance  A float
   @returns: bool
   """
   if type(goal) is list:
       for index in range(len(goal)):
           if abs(actual[index] - goal[index]) > tolerance:
               return False


   elif type(goal) is geometry_msgs.msg.PoseStamped:
       return all_close(goal.pose, actual.pose, tolerance)


   elif type(goal) is geometry_msgs.msg.Pose:
       x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
       x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
       # Euclidean distance
       d = dist((x1, y1, z1), (x0, y0, z0))
       # phi = angle between orientations
       cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
       return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)


   return True




class MoveGroupPythonInterfaceTutorial(object):
   """MoveGroupPythonInterfaceTutorial"""


   def __init__(self):
       super(MoveGroupPythonInterfaceTutorial, self).__init__()


       ## BEGIN_SUB_TUTORIAL setup
       ##
       ## First initialize `moveit_commander`_ and a `rospy`_ node:
       moveit_commander.roscpp_initialize(sys.argv)
       rospy.init_node("move_group_python_interface_tutorial", anonymous=True)


       ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
       ## kinematic model and the robot's current joint states
       robot = moveit_commander.RobotCommander()


       ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
       ## for getting, setting, and updating the robot's internal understanding of the
       ## surrounding world:
       scene = moveit_commander.PlanningSceneInterface()


       ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
       ## to a planning group (group of joints).  In this tutorial the group is the primary
       ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
       ## If you are using a different robot, change this value to the name of your robot
       ## arm planning group.
       ## This interface can be used to plan and execute motions:
       group_name = "panda_arm"
       move_group = moveit_commander.MoveGroupCommander(group_name)

       ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
       ## trajectories in Rviz:
       display_trajectory_publisher = rospy.Publisher(
           "/move_group/display_planned_path",
           moveit_msgs.msg.DisplayTrajectory,
           queue_size=20,
       )


       ## END_SUB_TUTORIAL


       ## BEGIN_SUB_TUTORIAL basic_info


       ## Getting Basic Information
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^
       ##
       ## We can get the name of the reference frame for this robot:
       planning_frame = move_group.get_planning_frame()
       print("============ Planning frame: %s" % planning_frame)


       ## We can also print the name of the end-effector link for this group:
       eef_link = move_group.get_end_effector_link()
       print("============ End effector link: %s" % eef_link)


       ## We can get a list of all the groups in the robot:
       group_names = robot.get_group_names()
       print("============ Available Planning Groups:", robot.get_group_names())


       ## Sometimes for debugging it is useful to print the entire state of the
       ## robot:
       print("============ Printing robot state")
       print(robot.get_current_state())
       print("")
       ## END_SUB_TUTORIAL


       # Misc variables
       self.box_name = ""
       self.robot = robot
       self.scene = scene
       self.move_group = move_group
       self.display_trajectory_publisher = display_trajectory_publisher
       self.planning_frame = planning_frame
       self.eef_link = eef_link
       self.group_names = group_names
       self.move_group.set_max_velocity_scaling_factor(1)
       self.move_group.set_max_acceleration_scaling_factor(1)

   def go_to_joint_inter(self):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       move_group = self.move_group


       ## BEGIN_SUB_TUTORIAL plan_to_joint_state
       ##
       ## Planning to a Joint Goal
       ## ^^^^^^^^^^^^^^^^^^^^^^^^
       ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
       ## thing we want to do is move it to a slightly better configuration.
       ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
       # We get the joint values from the group and change some of the values:
       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = 128/360*tau
       joint_goal[1] = 27/360*tau
       joint_goal[2] = -134/360*tau
       joint_goal[3] = -125/360*tau
       joint_goal[4] = 20/360*tau
       joint_goal[5] = 104/360*tau
       joint_goal[6] = -66/360*tau


       # The go command can be called with joint values, poses, or without any
       # parameters if you have already set the pose or joint target for the group
       move_group.go(joint_goal, wait=True)


       # Calling ``stop()`` ensures that there is no residual movement
       move_group.stop()


       # For testing:
       current_joints = move_group.get_current_joint_values()
       return all_close(joint_goal, current_joints, 0.01)

   def go_to_joint_state1(self):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       move_group = self.move_group


       ## BEGIN_SUB_TUTORIAL plan_to_joint_state
       ##
       ## Planning to a Joint Goal
       ## ^^^^^^^^^^^^^^^^^^^^^^^^
       ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
       ## thing we want to do is move it to a slightly better configuration.
       ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
       # We get the joint values from the group and change some of the values:
       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = 131/360*tau
       joint_goal[1] = -23/360*tau
       joint_goal[2] = -151/360*tau
       joint_goal[3] = -132/360*tau
       joint_goal[4] = -22/360*tau
       joint_goal[5] = 150/360*tau
       joint_goal[6] = -54/360*tau


       # The go command can be called with joint values, poses, or without any
       # parameters if you have already set the pose or joint target for the group
       move_group.go(joint_goal, wait=True)


       # Calling ``stop()`` ensures that there is no residual movement
       move_group.stop()


       # For testing:
       current_joints = move_group.get_current_joint_values()
       return all_close(joint_goal, current_joints, 0.01)
  
   def go_to_joint_state2(self):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       move_group = self.move_group


       ## BEGIN_SUB_TUTORIAL plan_to_joint_state
       ##
       ## Planning to a Joint Goal
       ## ^^^^^^^^^^^^^^^^^^^^^^^^
       ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
       ## thing we want to do is move it to a slightly better configuration.
       ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
       # We get the joint values from the group and change some of the values:
       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = 140/360*tau
       joint_goal[1] = -20/360*tau
       joint_goal[2] = -149/360*tau
       joint_goal[3] = -138/360*tau
       joint_goal[4] = -23/360*tau
       joint_goal[5] = 153/360*tau
       joint_goal[6] = -42/360*tau


       # The go command can be called with joint values, poses, or without any
       # parameters if you have already set the pose or joint target for the group
       move_group.go(joint_goal, wait=True)


       # Calling ``stop()`` ensures that there is no residual movement
       move_group.stop()


       # For testing:
       current_joints = move_group.get_current_joint_values()
       return all_close(joint_goal, current_joints, 0.01)
  
   def go_to_joint_state3(self):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       move_group = self.move_group


       ## BEGIN_SUB_TUTORIAL plan_to_joint_state
       ##
       ## Planning to a Joint Goal
       ## ^^^^^^^^^^^^^^^^^^^^^^^^
       ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
       ## thing we want to do is move it to a slightly better configuration.
       ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
       # We get the joint values from the group and change some of the values:
       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = 150/360*tau
       joint_goal[1] = -22/360*tau
       joint_goal[2] = -147/360*tau
       joint_goal[3] = -136/360*tau
       joint_goal[4] = -26/360*tau
       joint_goal[5] = 152/360*tau
       joint_goal[6] = 158/360*tau


       # The go command can be called with joint values, poses, or without any
       # parameters if you have already set the pose or joint target for the group
       move_group.go(joint_goal, wait=True)


       # Calling ``stop()`` ensures that there is no residual movement
       move_group.stop()


       # For testing:
       current_joints = move_group.get_current_joint_values()
       return all_close(joint_goal, current_joints, 0.01)
  
   def go_to_joint_state4(self):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       move_group = self.move_group


       ## BEGIN_SUB_TUTORIAL plan_to_joint_state
       ##
       ## Planning to a Joint Goal
       ## ^^^^^^^^^^^^^^^^^^^^^^^^
       ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
       ## thing we want to do is move it to a slightly better configuration.
       ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
       # We get the joint values from the group and change some of the values:
       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = 90/360*tau
       joint_goal[1] = 56/360*tau
       joint_goal[2] = -71/360*tau
       joint_goal[3] = -129/360*tau
       joint_goal[4] = 71/360*tau
       joint_goal[5] = 124/360*tau
       joint_goal[6] = 105/360*tau


       # The go command can be called with joint values, poses, or without any
       # parameters if you have already set the pose or joint target for the group
       move_group.go(joint_goal, wait=True)


       # Calling ``stop()`` ensures that there is no residual movement
       move_group.stop()


       # For testing:
       current_joints = move_group.get_current_joint_values()
       return all_close(joint_goal, current_joints, 0.01)
   
   def go_to_joint_state5(self):
       move_group = self.move_group

       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = -117/360*tau
       joint_goal[1] = -59/360*tau
       joint_goal[2] = -142/360*tau
       joint_goal[3] = -75/360*tau
       joint_goal[4] = -37/360*tau
       joint_goal[5] = 122/360*tau
       joint_goal[6] = -121/360*tau

       move_group.go(joint_goal, wait=True)
       move_group.stop()


       # For testing:
       current_joints = move_group.get_current_joint_values()
       return all_close(joint_goal, current_joints, 0.01)
   
   def go_to_joint_state7(self):
       move_group = self.move_group

       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = -119/360*tau
       joint_goal[1] = -52/360*tau
       joint_goal[2] = -138/360*tau
       joint_goal[3] = -72/360*tau
       joint_goal[4] = -34/360*tau
       joint_goal[5] = 112/360*tau
       joint_goal[6] = 53/360*tau

       move_group.go(joint_goal, wait=True)
       move_group.stop()


       # For testing:
       current_joints = move_group.get_current_joint_values()
       return all_close(joint_goal, current_joints, 0.01)
   
   def go_to_joint_state6(self):
       move_group = self.move_group

       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = 48/360*tau
       joint_goal[1] = 95/360*tau
       joint_goal[2] = 99/360*tau
       joint_goal[3] = -69/360*tau
       joint_goal[4] = -97/360*tau
       joint_goal[5] = 84/360*tau
       joint_goal[6] = -111/360*tau

       move_group.go(joint_goal, wait=True)
       move_group.stop()


       # For testing:
       current_joints = move_group.get_current_joint_values()
       return all_close(joint_goal, current_joints, 0.01)
   
   def go_to_joint_state8(self):
       move_group = self.move_group

       joint_goal = move_group.get_current_joint_values()
       joint_goal[0] = 62/360*tau
       joint_goal[1] = 49/360*tau
       joint_goal[2] = 42/360*tau
       joint_goal[3] = -68/360*tau
       joint_goal[4] = -32/360*tau
       joint_goal[5] = 106/360*tau
       joint_goal[6] = 50/360*tau

       move_group.go(joint_goal, wait=True)
       move_group.stop()


       # For testing:
       current_joints = move_group.get_current_joint_values()
       return all_close(joint_goal, current_joints, 0.01)
   
   def wait_for_state_update(
       self, box_is_known=False, box_is_attached=False, timeout=4
   ):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = self.box_name
       scene = self.scene


       ## BEGIN_SUB_TUTORIAL wait_for_scene_update
       ##
       ## Ensuring Collision Updates Are Received
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
       ## or dies before actually publishing the scene update message, the message
       ## could get lost and the box will not appear. To ensure that the updates are
       ## made, we wait until we see the changes reflected in the
       ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
       ## For the purpose of this tutorial, we call this function after adding,
       ## removing, attaching or detaching an object in the planning scene. We then wait
       ## until the updates have been made or ``timeout`` seconds have passed.
       ## To avoid waiting for scene updates like this at all, initialize the
       ## planning scene interface with  ``synchronous = True``.
       start = rospy.get_time()
       seconds = rospy.get_time()
       while (seconds - start < timeout) and not rospy.is_shutdown():
           # Test if the box is in attached objects
           attached_objects = scene.get_attached_objects([box_name])
           is_attached = len(attached_objects.keys()) > 0


           # Test if the box is in the scene.
           # Note that attaching the box will remove it from known_objects
           is_known = box_name in scene.get_known_object_names()


           # Test if we are in the expected state
           if (box_is_attached == is_attached) and (box_is_known == is_known):
               return True


           # Sleep so that we give other threads time on the processor
           rospy.sleep(0.1)
           seconds = rospy.get_time()


       # If we exited the while loop without returning then we timed out
       return False
   
   def add_box(self, timeout=4):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = self.box_name
       scene = self.scene


       ## BEGIN_SUB_TUTORIAL add_box
       ##
       ## Adding Objects to the Planning Scene
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## First, we will create a box in the planning scene between the fingers:
       box_pose = geometry_msgs.msg.PoseStamped()
       box_pose.header.frame_id = "world"
       box_pose.header.frame_id = "world"
       box_pose.pose.orientation.w = 1.0
       box_pose.pose.position.z = 0.025  # above the panda_hand frame
       box_pose.pose.position.x = 0.5  # above the panda_hand frame
       box_name = "box"
       scene.add_box(box_name, box_pose, size=(0.25, 0.5, 0.05))


       ## END_SUB_TUTORIAL
       # Copy local variables back to class variables. In practice, you should use the class
       # variables directly unless you have a good reason not to.
       self.box_name = box_name
       return self.wait_for_state_update(box_is_known=True, timeout=timeout)
  
   def add_box2(self, timeout=4):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = self.box_name
       scene = self.scene


       ## BEGIN_SUB_TUTORIAL add_box
       ##
       ## Adding Objects to the Planning Scene
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## First, we will create a box in the planning scene between the fingers:
       box_pose = geometry_msgs.msg.PoseStamped()
       box_pose.header.frame_id = "world"
       box_pose.header.frame_id = "world"
       box_pose.pose.orientation.w = 1.0
       box_pose.pose.position.z = 0.025  # above the panda_hand frame
       box_pose.pose.position.y = 0.75  # above the panda_hand frame
       box_name = "box2"
       scene.add_box(box_name, box_pose, size=(0.5, 0.25, 0.05))


       ## END_SUB_TUTORIAL
       # Copy local variables back to class variables. In practice, you should use the class
       # variables directly unless you have a good reason not to.
       self.box_name = box_name
       return self.wait_for_state_update(box_is_known=True, timeout=timeout)
   
   def add_box3(self, timeout=4):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = self.box_name
       scene = self.scene


       ## BEGIN_SUB_TUTORIAL add_box
       ##
       ## Adding Objects to the Planning Scene
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## First, we will create a box in the planning scene between the fingers:
       box_pose = geometry_msgs.msg.PoseStamped()
       box_pose.header.frame_id = "world"
       box_pose.header.frame_id = "world"
       box_pose.pose.orientation.w = 1.0
       box_pose.pose.position.z = 0.075  # above the panda_hand frame
       #box_pose.pose.position.y = 0.75  # above the panda_hand frame
       box_pose.pose.position.x = 0.5  # above the panda_hand frame
       box_pose.pose.position.y = 0.055  # above the panda_hand frame
       box_name = "box3"
       scene.add_box(box_name, box_pose, size=(0.05, 0.035, 0.05))


       ## END_SUB_TUTORIAL
       # Copy local variables back to class variables. In practice, you should use the class
       # variables directly unless you have a good reason not to.
       self.box_name = box_name
       return self.wait_for_state_update(box_is_known=True, timeout=timeout)
   
   def add_box4(self, timeout=4):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = self.box_name
       scene = self.scene


       ## BEGIN_SUB_TUTORIAL add_box
       ##
       ## Adding Objects to the Planning Scene
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## First, we will create a box in the planning scene between the fingers:
       box_pose = geometry_msgs.msg.PoseStamped()
       box_pose.header.frame_id = "world"
       box_pose.header.frame_id = "world"
       box_pose.pose.orientation.w = 1.0
       box_pose.pose.position.z = 0.075  # above the panda_hand frame
       #box_pose.pose.position.y = 0.75  # above the panda_hand frame
       box_pose.pose.position.x = 0.5  # above the panda_hand frame
       box_pose.pose.position.y = 0.15  # above the panda_hand frame
       box_name = "box4"
       scene.add_box(box_name, box_pose, size=(0.025, 0.055, 0.05))


       ## END_SUB_TUTORIAL
       # Copy local variables back to class variables. In practice, you should use the class
       # variables directly unless you have a good reason not to.
       self.box_name = box_name
       return self.wait_for_state_update(box_is_known=True, timeout=timeout)

   def add_box5(self, timeout=4):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = self.box_name
       scene = self.scene


       ## BEGIN_SUB_TUTORIAL add_box
       ##
       ## Adding Objects to the Planning Scene
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## First, we will create a box in the planning scene between the fingers:
       box_pose = geometry_msgs.msg.PoseStamped()
       box_pose.header.frame_id = "world"
       box_pose.header.frame_id = "world"
       box_pose.pose.orientation.w = 1.0
       box_pose.pose.position.z = 0.075  # above the panda_hand frame
       #box_pose.pose.position.y = 0.75  # above the panda_hand frame
       box_pose.pose.position.x = 0.5  # above the panda_hand frame
       box_pose.pose.position.y = -0.055  # above the panda_hand frame
       box_name = "box5"
       scene.add_box(box_name, box_pose, size=(0.035, 0.045, 0.05))


       ## END_SUB_TUTORIAL
       # Copy local variables back to class variables. In practice, you should use the class
       # variables directly unless you have a good reason not to.
       self.box_name = box_name
       return self.wait_for_state_update(box_is_known=True, timeout=timeout)
   
   def add_box6(self, timeout=4):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = self.box_name
       scene = self.scene


       ## BEGIN_SUB_TUTORIAL add_box
       ##
       ## Adding Objects to the Planning Scene
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## First, we will create a box in the planning scene between the fingers:
       box_pose = geometry_msgs.msg.PoseStamped()
       box_pose.header.frame_id = "world"
       box_pose.header.frame_id = "world"
       box_pose.pose.orientation.w = 1.0
       box_pose.pose.position.z = 0.075  # above the panda_hand frame
       #box_pose.pose.position.y = 0.75  # above the panda_hand frame
       box_pose.pose.position.x = 0.5  # above the panda_hand frame
       box_pose.pose.position.y = -0.17  # above the panda_hand frame
       box_name = "box6"
       scene.add_box(box_name, box_pose, size=(0.045, 0.09, 0.05))


       ## END_SUB_TUTORIAL
       # Copy local variables back to class variables. In practice, you should use the class
       # variables directly unless you have a good reason not to.
       self.box_name = box_name
       return self.wait_for_state_update(box_is_known=True, timeout=timeout)

   def attach_box4(self, timeout=4):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = "box4"
       robot = self.robot
       scene = self.scene
       eef_link = self.eef_link
       group_names = self.group_names


       ## BEGIN_SUB_TUTORIAL attach_object
       ##
       ## Attaching Objects to the Robot
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
       ## robot be able to touch them without the planning scene reporting the contact as a
       ## collision. By adding link names to the ``touch_links`` array, we are telling the
       ## planning scene to ignore collisions between those links and the box. For the Panda
       ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
       ## you should change this value to the name of your end effector group name.
       grasping_group = "panda_hand"
       touch_links = robot.get_link_names(group=grasping_group)
       scene.attach_box(eef_link, box_name, touch_links=touch_links)
       ## END_SUB_TUTORIAL


       # We wait for the planning scene to update.
       return self.wait_for_state_update(
           box_is_attached=True, box_is_known=False, timeout=timeout
       )
   
   def attach_box3(self, timeout=4):
       
       box_name = "box3"
       robot = self.robot
       scene = self.scene
       eef_link = self.eef_link
       group_names = self.group_names

       grasping_group = "panda_hand"
       touch_links = robot.get_link_names(group=grasping_group)
       scene.attach_box(eef_link, box_name, touch_links=touch_links)
       
       return self.wait_for_state_update(
           box_is_attached=True, box_is_known=False, timeout=timeout
       )
   
   def attach_box2(self, timeout=4):
       
       box_name = "box5"
       robot = self.robot
       scene = self.scene
       eef_link = self.eef_link
       group_names = self.group_names

       grasping_group = "panda_hand"
       touch_links = robot.get_link_names(group=grasping_group)
       scene.attach_box(eef_link, box_name, touch_links=touch_links)
       
       return self.wait_for_state_update(
           box_is_attached=True, box_is_known=False, timeout=timeout
       )
   
   def attach_box1(self, timeout=4):
       
       box_name = "box6"
       robot = self.robot
       scene = self.scene
       eef_link = self.eef_link
       group_names = self.group_names

       grasping_group = "panda_hand"
       touch_links = robot.get_link_names(group=grasping_group)
       scene.attach_box(eef_link, box_name, touch_links=touch_links)
       
       return self.wait_for_state_update(
           box_is_attached=True, box_is_known=False, timeout=timeout
       )


   def detach_box4(self, timeout=4):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = "box4"
       scene = self.scene
       eef_link = self.eef_link


       ## BEGIN_SUB_TUTORIAL detach_object
       ##
       ## Detaching Objects from the Robot
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## We can also detach and remove the object from the planning scene:
       scene.remove_attached_object(eef_link, name=box_name)
       ## END_SUB_TUTORIAL


       # We wait for the planning scene to update.
       return self.wait_for_state_update(
           box_is_known=True, box_is_attached=False, timeout=timeout
       )
   
   def detach_box3(self, timeout=4):
       box_name = "box3"
       scene = self.scene
       eef_link = self.eef_link

       scene.remove_attached_object(eef_link, name=box_name)
       
       return self.wait_for_state_update(
           box_is_known=True, box_is_attached=False, timeout=timeout
       )
   
   def detach_box2(self, timeout=4):
       box_name = "box5"
       scene = self.scene
       eef_link = self.eef_link

       scene.remove_attached_object(eef_link, name=box_name)
       
       return self.wait_for_state_update(
           box_is_known=True, box_is_attached=False, timeout=timeout
       )
   
   def detach_box1(self, timeout=4):
       box_name = "box6"
       scene = self.scene
       eef_link = self.eef_link

       scene.remove_attached_object(eef_link, name=box_name)
       
       return self.wait_for_state_update(
           box_is_known=True, box_is_attached=False, timeout=timeout
       )



   def remove_box(self, timeout=4):
       # Copy class variables to local variables to make the web tutorials more clear.
       # In practice, you should use the class variables directly unless you have a good
       # reason not to.
       box_name = self.box_name
       scene = self.scene


       ## BEGIN_SUB_TUTORIAL remove_object
       ##
       ## Removing Objects from the Planning Scene
       ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
       ## We can remove the box from the world.
       scene.remove_world_object("box3")
       scene.remove_world_object("box5")
       scene.remove_world_object("box4")
       scene.remove_world_object("box6")   

       ## **Note:** The object must be detached before we can remove it from the world
       ## END_SUB_TUTORIAL
  
def main():   
   try:
       print("")
       print("----------------------------------------------------------")
       print("Welcome to the Fred MoveGroup Python Interface")
       print("----------------------------------------------------------")
       print("Press Ctrl-D to exit at any time")
       print("")
       input(
           "============ Press `Enter` to begin the execution by setting up the moveit_commander ..."
       )
       tutorial = MoveGroupPythonInterfaceTutorial()
        #box order 1, 2, 3, 4: 6 - 5 - 3 - 4

       input("============ Press `Enter` to add boxes to the scene ...")
       tutorial.add_box()
       tutorial.add_box2()
       tutorial.add_box3()
       tutorial.add_box4()
       tutorial.add_box5() 
       tutorial.add_box6()

        # pick up box 4 and place it on table
       i: int = 0
       b: int = 0
       done: list[str] = []
    

       while i < 4:
        inp = input("Which box would you like to pick up? from left to right: 1, 2, 3, 4 ")
        
        if inp in done:
            print("You have already picked up this box. Please choose another box.")
            continue

        done.append(inp) #add inp to done list

        if inp == "4":
            b = 4
            input("============ Press `Enter` to go to initial state ...")
            tutorial.go_to_joint_state4()
            
            
            input("============ Press `Enter` to attach box 4...")
            tutorial.attach_box4()
            tutorial.go_to_joint_inter()


        elif inp == "3":
            
            #pick up box 3 and place it on top of box 4
            b = 3
            input("============ Press `Enter` to pick up box 3...")
            tutorial.go_to_joint_state3()

            input("============ Press `Enter` to attach box 3...")
            tutorial.attach_box3()
            tutorial.go_to_joint_inter()
        
            # pick up box 2 and place it on box 3

        elif inp == "2":
            b = 2
            input("============ Press `Enter` to pick up box 2 ...")
            tutorial.go_to_joint_state2()
            
            input("============ Press `Enter` to attach box 2...")
            tutorial.attach_box2()
            tutorial.go_to_joint_inter()

        elif inp == "1":
            b = 1
            #pick up box 1 and place it on top of box 2

            input("============ Press `Enter` to pick up box 1...")
            tutorial.go_to_joint_state1()

            input("============ Press `Enter` to attach box 1...")
            tutorial.attach_box1()
            tutorial.go_to_joint_inter()

        else: 
            print("Please enter a valid input")
            continue


        if i == 0:
            input("============ Press `Enter` to place the box on the table...")
            tutorial.go_to_joint_state5()
            i = i + 1

            if b == 4:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box4()
            elif b == 3:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box3()
            elif b == 2:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box2()
            elif b == 1:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box1()
            
        elif i == 1:
            input("============ Press `Enter` to place the box on the tower...")
            tutorial.go_to_joint_state6()
            i = i + 1

            if b == 4:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box4()
            elif b == 3:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box3()
            elif b == 2:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box2()
            elif b == 1:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box1()
       
        elif i == 2:
            input("============ Press `Enter` to place the box on the tower...")
            tutorial.go_to_joint_state7()
            i = i + 1

            if b == 4:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box4()
            elif b == 3:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box3()
            elif b == 2:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box2()
            elif b == 1:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box1()       

        elif i == 3:
            input("============ Press `Enter` to place the box on the tower...")
            tutorial.go_to_joint_state8()
            i = i + 1

            if b == 4:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box4()
            elif b == 3:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box3()
            elif b == 2:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box2()
            elif b == 1:
                input("============ Press `Enter` to detach the box..")
                tutorial.detach_box1()

       input(
           "============ Press `Enter` to remove the box from the planning scene ..."
       )
       tutorial.remove_box()

       print("============ Fred complete!")


   except rospy.ROSInterruptException:
       return
   except KeyboardInterrupt:
       return
  
if __name__ == "__main__":
   main()