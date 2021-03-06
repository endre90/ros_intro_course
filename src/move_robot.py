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

class move_robot():

    def __init__(self):

        # ROS node initializer:
        rospy.init_node('move_robot', anonymous=False)

        # Move Group specifier:
        self.robot = mgc("manipulator")
        self.scene = psi()
      
        # Subscribers and Publishers:
        rospy.Subscriber("/command", Command, self.cmd_callback)
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        self.main_publisher = rospy.Publisher("/state", State, queue_size=10)

        # Message type initializers:
        self.state_msg = State()
        self.command_msg = Command()
        self.got_command = Command()
        self.joints = JointState()
        self.pose_stamped = PoseStamped()

        # Command message value inits:
        self.command = ''
        self.object_name = ''
        self.pose = ''
        self.speed_factor = 0.0
        self.acc_factor = 0.0
        self.goal_tolerance = 0.0

        # State message value inits:
        self.robot_moving = False

        # Saved Joint poses:
        self.PreTool = [-1.1983061769206886, -1.956906458242271, 2.09577828492735, -0.13871694150397382, 0.37028024897589895, -2.754694684064499e-05]
        self.AtTool = [-0.9996087360999348, -1.8358317085370144, 2.0142591175722195, -0.17827345426541943, 0.5689803411293279, -0.00018259849421303472]
        self.PrePickupItem = [0.9925944761423243, -1.7146224012555238, -2.519250614983209, -2.04929198322585, -0.5695560232069059, 1.3096161387452838e-05]
        self.PickupItem = [0.8383205206574562, -1.8015428992505043, -2.3960341753288197, -2.0856570501416853, -0.723777270553927, -1.1574922867361477e-05]
        self.PostLeaveItem = [-0.844894238580146, -2.0219979678119944, -2.033310626431399, -2.227815660099317, -2.3962839764864285, -1.492357469655035e-05]
        self.LeaveItem = [-0.7112198473299587, -2.0771468285895227, -1.9348112667827249, -2.2712588808397873, -2.2733827360979126, 8.931732245211308e-05]
        self.Up = [5.081929685547949e-05, -1.570774324260559, -8.439498278312385e-05, -1.5707146640149876, 1.4722079085186126e-05, 6.612974880263209e-05]

        # Robot joint identifiers:
        self.joints.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Pose list 
        self.joint_pose_list = [self.PreTool,
                                self.AtTool,
                                self.PrePickupItem,
                                self.PickupItem,
                                self.PostLeaveItem,
                                self.LeaveItem,
                                self.Up]
        
        # Pose name list to map to pose list
        self.joint_pose_name_list = ["PRE_TOOL",
                                     "AT_TOOL",
                                     "PRE_PICKUP_ITEM",
                                     "PICKUP_ITEM",
                                     "POST_LEAVE_ITEM",
                                     "LEAVE_ITEM",
                                     "UP"]

        # Pose tolerance:
        self.joint_tol = 0.01

        # Publisher rates:
        self.main_pub_rate = rospy.Rate(10)
       
        # Some time to assure initialization:
        rospy.sleep(2)

        # Add some objects in the scene
        self.add_object("TOOL", [0.5, -0.25, 1.5, 0, 0, 0], (0.1, 0.1, 0.1))
        self.add_object("TABLE", [-0.75, 0, 0.5, 0, 0, 0], (0.75, 2, 1))
        self.add_object("ITEM", [-0.7, -0.2, 1.1, 0, 0, 0], (0.2, 0.2, 0.2))
        #self.add_object("HUMAN", [0, 0.6, 1, 0, 0, 0], (0.6, 0.3, 2))

        rospy.sleep(1)

        self.added_objects = self.scene.get_known_object_names()

        self.main()


    def main(self):

        while not rospy.is_shutdown():

            # Assemle the got command part
            self.got_command.command = self.command
            self.got_command.object_name = self.object_name
            self.got_command.pose = self.pose
            self.got_command.speed_factor = self.speed_factor
            self.got_command.acc_factor = self.acc_factor
            self.got_command.goal_tolerance = self.goal_tolerance

            # Assemble the whole state message
            self.state_msg.current_pose = self.get_static_joint_pose()
            self.state_msg.moving = self.robot_moving
            self.state_msg.scene_objects = self.get_current_scene_objects()
            self.state_msg.attached_objects = self.get_current_attached_objects()
            self.state_msg.got_command = self.got_command

            # Publish the message and sleep a bit:
            self.main_publisher.publish(self.state_msg)
            self.main_pub_rate.sleep()
        
        rospy.spin()


    def switcher(self, what, case_list):

        for i in range(0, len(case_list), 1):
            if what == case_list[i]:
                return i
                break
            else:
                pass


    def get_current_scene_objects(self):

        return self.scene.get_known_object_names()


    def get_current_attached_objects(self):

        return list(set(self.added_objects) - set(self.scene.get_known_object_names()))


    def rpy_to_quat(self, x, y, z, roll, pitch, yaw):
     
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        qx = quaternion[0]
        qy = quaternion[1]
        qz = quaternion[2]
        qw = quaternion[3]

        return [x, y, z, qx, qy, qz, qw]
    

    def add_object(self, name, rpy_pose, size):

        quat = self.rpy_to_quat(rpy_pose[0], rpy_pose[1], rpy_pose[2], rpy_pose[3], rpy_pose[4], rpy_pose[5])

        self.pose_stamped.header.frame_id = "world"
        self.pose_stamped.pose.position.x = quat[0]
        self.pose_stamped.pose.position.y = quat[1]
        self.pose_stamped.pose.position.z = quat[2]
        self.pose_stamped.pose.orientation.x = quat[3]
        self.pose_stamped.pose.orientation.y = quat[4]
        self.pose_stamped.pose.orientation.z = quat[5]
        self.pose_stamped.pose.orientation.w = quat[6]

        self.scene.add_box(name, self.pose_stamped, size)


    def attach_object(self, name):

        self.robot.attach_object(name, "tool0")


    def detach_object(self, name):

        self.robot.detach_object(name)

    
    def planned_move(self, pose_name, speed_factor, acc_factor, goal_tolerance):

        self.robot.set_max_velocity_scaling_factor(self.speed_factor)
        self.robot.set_max_acceleration_scaling_factor(self.acc_factor)
        self.robot.set_goal_tolerance(self.goal_tolerance)

        if pose_name in self.joint_pose_name_list:
            pose = self.joint_pose_list[self.switcher(pose_name, self.joint_pose_name_list)]
            print(pose_name)
            print(pose)
        else:
            pass

        self.joints.position = pose
        self.robot.go(self.joints, wait = False)
        rospy.sleep(1)

         
    def get_static_joint_pose(self):

        current_pose = self.joint_pose

        for pose in self.joint_pose_list:
            if all(numpy.isclose(current_pose[i], pose[i], atol=self.joint_tol) for i in range(0, 6)):
                actual_joint_pose_name = self.joint_pose_name_list[self.switcher(pose, self.joint_pose_list)]
                break
            else:
                actual_joint_pose_name = "UNKNOWN"
                pass
       
        return actual_joint_pose_name
        

    def cmd_callback(self, data):
       
        self.command = data.command
        self.object_name = data.object_name
        self.pose = data.pose
        self.speed_factor = data.speed_factor
        self.acc_factor = data.acc_factor
        self.goal_tolerance = data.goal_tolerance

        if self.command == "MOVE":
            self.planned_move(self.pose, self.speed_factor, self.acc_factor, self.goal_tolerance)
        elif self.command == "ATTACH":
            self.attach_object(self.object_name)
        elif self.command == "DETACH":
            self.detach_object(self.object_name)
        else:
            pass


    def joint_callback(self, joint):

        self.joint_pose = joint.position

        if len(joint.velocity) != 0:
            if all(joint.velocity[i] == 0 for i in range(0, 6, 1)):
                self.moving = False
            else:
                self.moving = True
        else:
            self.moving = False


if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
