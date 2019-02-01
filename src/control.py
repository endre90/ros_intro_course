#!/usr/bin/env python

#----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
#----------------------------------------------------------------------------------------------------------------------#
    # Endre Eros
    # ROS Intro Course Exercise
    # V.1.0.0.
#----------------------------------------------------------------------------------------------------------------------#

import rospy
import roslib
import sys
import time
import numpy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander as mgc
from moveit_commander import PlanningSceneInterface as psi
from ros_intro_course.msg import Command
from ros_intro_course.msg import State

class control():

    def __init__(self):

        # ROS node initializer:
        rospy.init_node('control', anonymous=False)

        # Move Group specifier:
        self.robot = mgc("manipulator")
        self.scene = psi()
      
        # Subscribers and Publishers:
        rospy.Subscriber("/state", State, self.state_callback)
        self.main_publisher = rospy.Publisher("/command", Command, queue_size=10)

        # Message type initializers:
        self.state_msg = State()
        self.command_msg = Command()
        self.joints = JointState()
        self.pose_stamped = PoseStamped()

        # State message value inits:
        self.current_pose = ''
        self.moving = False
        self.scene_objects = []
        self.attached_objects = []
        self.got_command = ''
        self.got_object_name = ''
        self.got_pose = ''
        self.got_speed_factor = 0.0
        self.got_acc_factor = 0.0
        self.got_goal_tolerance = 0.0

        # Command message value inits:
        self.command = ''
        self.object_name = ''
        self.pose = ''
        self.speed_factor = 0.0
        self.acc_factor = 0.0
        self.goal_tolerance = 0.0

        # State tracking:
        self.item_pos = "POSE1"

    
        # Publisher rates:
        self.main_pub_rate = rospy.Rate(10)
       
        # Some time to assure initialization:
        rospy.sleep(2)

        self.added_objects = self.scene.get_known_object_names()

        #self.main()

        # only for simulation testing:
        self.executeseq()


    def executeseq(self):

        # This is the wrong way to do it since there is no feedback, and it is only good for simulation:
        self.assign_and_publish("MOVE", "", "PRE_TOOL", 0.5, 0.5, 0.01)
        time.sleep(8)
        self.assign_and_publish("MOVE", "", "AT_TOOL", 0.5, 0.5, 0.01)
        time.sleep(4)
        self.assign_and_publish("ATTACH", "TOOL", "", 0.5, 0.5, 0.01)
        time.sleep(3)
        self.assign_and_publish("MOVE", "", "PRE_TOOL", 0.5, 0.5, 0.01)
        time.sleep(8)
        self.assign_and_publish("MOVE", "", "PRE_PICKUP_ITEM", 0.5, 0.5, 0.01)
        time.sleep(8)
        self.assign_and_publish("MOVE", "", "PICKUP_ITEM", 0.5, 0.5, 0.01)
        time.sleep(5)
        self.assign_and_publish("ATTACH", "ITEM", "", 0.5, 0.5, 0.01)
        time.sleep(5)
        self.assign_and_publish("MOVE", "", "LEAVE_ITEM", 0.5, 0.5, 0.01)
        time.sleep(7)
        self.assign_and_publish("DETACH", "ITEM", "", 0.5, 0.5, 0.01)
        time.sleep(5)
        self.assign_and_publish("MOVE", "", "POST_LEAVE_ITEM", 0.5, 0.5, 0.01)
        time.sleep(5)
        self.assign_and_publish("MOVE", "", "PRE_TOOL", 0.5, 0.5, 0.01)
        time.sleep(10)
        self.assign_and_publish("MOVE", "", "AT_TOOL", 0.5, 0.5, 0.01)
        time.sleep(5)
        self.assign_and_publish("DETACH", "TOOL", "", 0.5, 0.5, 0.01)
        time.sleep(3)
        self.assign_and_publish("MOVE", "", "PRE_TOOL", 0.5, 0.5, 0.01)
        

    def assign_and_publish(self, cmd, obj, pose, spd, acc, goal):

        self.command_msg.command = cmd
        self.command_msg.object_name = obj
        self.command_msg.pose = pose
        self.command_msg.speed_factor = spd
        self.command_msg.acc_factor = acc
        self.command_msg.goal_tolerance = goal

        self.main_publisher.publish(self.command_msg)



    def main(self):

        while not rospy.is_shutdown():
            pass
        
        rospy.spin()

        

    def state_callback(self, data):
       
        self.current_pose = data.current_pose
        self.moving = data.moving
        self.scene_objects = data.scene_objects
        self.attached_objects = data.attached_objects
        self.got_command = data.got_command.command
        self.got_object_name = data.got_command.object_name
        self.got_pose = data.got_command.pose
        self.got_speed_factor = data.got_command.speed_factor
        self.got_acc_factor = data.got_command.acc_factor
        self.got_goal_tolerance = data.got_command.goal_tolerance

        # it should be done like this by knowing the actual state:
        # what are we heeping track of:
            # 1. robot pose (unknown, at_tool, ...)
            # 2. tool state (attached or not)
            # 3. item pose (pose2, pose2)
            # 4. item state (attached or not)
            # 5. it would be nice to keep track of the robot velocity but we dont have that in the demo sim (use official ur sim)
        # This makes a total of 2 x 2 x 2 x nr.of_robot_poses = a lot of states...


        # Ignore this or use parts... (not ready)
        # if self.item_pos == "POSE1":
            # if "TOOL" not in self.attached_objects:  # and self.moving == False: (not usable in the demo sim)
                # self.assign_and_publish("MOVE", "", "PRE_TOOL", 0.5, 0.5, 0.01)
                # if self.current_pose == "PRE_TOOL" and "TOOL" not in self.attached_objects:
                    # self.assign_and_publish("MOVE", "", "AT_TOOL", 0.5, 0.5, 0.01)
                    # time.sleep(2)
                    # self.assign_and_publish("ATTACH", "", "AT_TOOL", 0.5, 0.5, 0.01)
                    # time.sleep(2)
                    # self.assign_and_publish("MOVE", "", "PRE_TOOL", 0.5, 0.5, 0.01)
                # if self.current_pose == "PRE_TOOL" and "TOOL" in self.attached_objects:
                    # self.assign_and_publish("MOVE", "", "PRE_PICKUP_ITEM", 0.5, 0.5, 0.01)
                    # time.sleep(5)
                    # self.assign_and_publish("MOVE", "", "PICKUP_ITEM", 0.5, 0.5, 0.01)
                    # time.sleep(2)
                    # self.assign_and_publish("ATTACH", "", "ITEM", 0.5, 0.5, 0.01)
            # elif "TOOL" in self.attached_objects:
                # pass
                # elif self.current_pose == "PRE_TOOL":
                    # self.assign_and_publish("MOVE", "", "AT_TOOL", 0.1, 0.1, 0.01)
                    # time.sleep(2)
                    # self.assign_and_publish("DETTACH", "", "AT_TOOL", 0.1, 0.1, 0.01)
                    # time.sleep(2)
                    # self.assign_and_publish("MOVE", "", "PRE_TOOL", 0.1, 0.1, 0.01)
                    

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass