#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from abh3.msg import abh3Vel
from geometry_msgs.msg import Twist
from math import pi

class abh3Converter:
    def __init__(self):
        rospy.init_node('abh3_conv', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        driver_item = rospy.get_param('~driver/item', 'abh3')
        driver_name = rospy.get_param('~driver/name', 'no1')

        self.subAB = rospy.Subscriber('/%s/%s/vin/ab' % (driver_item, driver_name), abh3Vel, self.abh3velAB_handler)
        self.pubAB_YX = rospy.Publisher('/%s/%s/vout/ab_yx' % (driver_item, driver_name), abh3Vel, queue_size=1)
        self.pubAB_XA = rospy.Publisher('/%s/%s/vout/ab_xa' % (driver_item, driver_name), Twist, queue_size=1)

        self.subYX = rospy.Subscriber('/%s/%s/vin/yx' % (driver_item, driver_name), abh3Vel, self.abh3velYX_handler)
        self.pubYX_AB = rospy.Publisher('/%s/%s/vout/yx_ab' % (driver_item, driver_name), abh3Vel, queue_size=1)
        self.pubYX_XA = rospy.Publisher('/%s/%s/vout/yx_xa' % (driver_item, driver_name), Twist, queue_size=1)

        self.subXA = rospy.Subscriber('/%s/%s/vin/xa' % (driver_item, driver_name), Twist, self.abh3velXA_handler)
        self.pubXA_AB = rospy.Publisher('/%s/%s/vout/xa_ab' % (driver_item, driver_name), abh3Vel, queue_size=1)
        self.pubXA_YX = rospy.Publisher('/%s/%s/vout/xa_yx' % (driver_item, driver_name), abh3Vel, queue_size=1)

        self.rate = float(rospy.get_param('~machine/rateNum', 1.0)) / float(rospy.get_param('~machine/rateDen', 10.0))
        self.wheel = float(rospy.get_param('~machine/wheel', 0.1))
        self.width = float(rospy.get_param('~machine/width', 0.5))

    def shutdown(self):
        pass

    def abh3velAB_handler(self, msg):
        velYX = abh3Vel()
        velXA = Twist()

        velYX.velAY = (msg.velAY + msg.velBX) / 2
        velYX.velBX = msg.velAY - velYX.velAY

        velXA.linear.x = velYX.velAY / 60 * self.rate * self.wheel * pi
        velXA.angular.z = -velYX.velBX / 60 * self.rate * self.wheel * pi / (self.width / 2)

        self.pubAB_YX.publish(velYX)
        self.pubAB_XA.publish(velXA)
    
    def abh3velYX_handler(self, msg):
        velAB = abh3Vel()
        velXA = Twist()

        velAB.velAY = msg.velAY + msg.velBX
        velAB.velBX = msg.velAY - msg.velBX

        velXA.linear.x = msg.velAY / 60 * self.rate * self.wheel * pi
        velXA.angular.z = -msg.velBX / 60 * self.rate * self.wheel * pi / (self.width / 2)

        self.pubYX_AB.publish(velAB)
        self.pubYX_XA.publish(velXA)
    
    def abh3velXA_handler(self, msg):
        velAB = abh3Vel()
        velYX = abh3Vel()

        velYX.velAY = msg.linear.x * 60 / (self.rate * self.wheel * pi)
        velYX.velBX = -msg.angular.z * 60 / (self.rate * self.wheel * pi) * (self.width / 2)

        velAB.velAY = velYX.velAY + velYX.velBX
        velAB.velBX = velYX.velAY - velYX.velBX

        self.pubXA_AB.publish(velAB)
        self.pubXA_YX.publish(velYX)
    
if __name__ == '__main__':
    try:
        manager = abh3Converter()
        rospy.spin()
    except rospy.ROSInterruptException:
            pass

