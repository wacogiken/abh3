#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from abh3.srv import abh3Com

class abh3JoyTwist:
    def __init__(self):

        rospy.init_node('abh3_joy', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.driver_item = rospy.get_param('~driver/item', 'abh3')
        self.driver_name = rospy.get_param('~driver/name', 'main')

        rospy.wait_for_service('/%s/%s/srv' % (self.driver_item, self.driver_name))
        self.service("SERVO OFF OFF")
        self.service("START ON ON")
        self.service("SELECT 0 0")
        self.service("SELVEL 0 0")
        self.service("TBL 0 CS A INT")
        self.service("TBL 0 CS B INT")

        self.sub = rospy.Subscriber('joy', Joy, self.joy_handler, queue_size=1)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rate = rospy.Rate(rospy.get_param('~rate', 20))

        self.joy = Joy()
        self.joy_flag = False
        svoff_flag = False
        svon_flag = False

        while not self.joy_flag:
            rate.sleep()

        while not rospy.is_shutdown():
            # Servo OFF
            if self.joy.buttons[14]:
                if svoff_flag == False:
                    self.service("SERVO OFF OFF")
                    svoff_flag = True
            else:
                svoff_flag = False

            # Servo ON
            if self.joy.buttons[13]:
                if svon_flag == False:
                    self.service("SERVO ON ON")
                    svon_flag = True
            else:
                svon_flag = False

            vel = Twist()
            vel.linear.x = self.joy.axes[1] * (0.5 + 0.5 * self.joy.buttons[11])
            vel.angular.z = self.joy.axes[0] * (0.25 + 0.25 * self.joy.buttons[11])
            self.pub.publish(vel)

            rate.sleep()

    def service(self, cmd):
        try:
            service = rospy.ServiceProxy('/%s/%s/srv' % (self.driver_item, self.driver_name), abh3Com)
            res = service(cmd)
        except rospy.ServiceException, e:
            print('Service call failed: %s' % e)

    def joy_handler(self, joy_msg):
        self.joy = joy_msg
        self.joy_flag = True
        
    def shutdown(self):
        pass
 
if __name__ == '__main__':
    try:
        manager = abh3JoyTwist()
    except rospy.ROSInterruptException:
            pass

