#!/usr/bin/env python

import rospy
import roslib
from ros_intro_course.msg import CustomMsg1

class custom_pub():

    def __init__(self):

        rospy.init_node('custom_pub', anonymous=False)

        self.main_pub = rospy.Publisher('/math', CustomMsg1, queue_size=10)

        self.pub_rate = rospy.Rate(10)

        self.msg = CustomMsg1()

        rospy.sleep(2)

        self.main()

    def main(self):

        while not rospy.is_shutdown():

            self.msg.operation = "add"
            self.msg.num1 = 1
            self.msg.num2 = 2

            self.main_pub.publish(self.msg)
            self.pub_rate.sleep()

        rospy.spin()

if __name__ == '__main__':
   try:
       custom_pub()
   except rospy.ROSInterruptException:
       pass