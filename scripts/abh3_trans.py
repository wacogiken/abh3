#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import rospy
import ast

from abh3.srv import abh3Com

class abh3Transfer:
    def __init__(self):
        rospy.init_node('abh3_trans', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        driver_item = rospy.get_param('~driver/item', 'abh3')
        driver_name = rospy.get_param('~driver/name', 'no1')

        rospy.wait_for_service('/%s/%s/srv' % (driver_item, driver_name))

        port_name = rospy.get_param('~serial/port_name', '/dev/abh3trans')
        port_baud = rospy.get_param('~serial/baud_rate', 115200)
        self.ser = serial.Serial(port_name, port_baud, timeout=0.1)

        while not rospy.is_shutdown():
            flag, s = self.receive()
            if flag == True:
                try:
                    service = rospy.ServiceProxy('/%s/%s/srv' % (driver_item, driver_name), abh3Com)
                    res = service(s)
                    if res.response != '':
                        self.send(command=res.response)
                except rospy.ServiceException, e:
                    print('Service call failed: %s' % e)
        
    def shutdown(self):
        self.ser.close()

    def receive(self):
        s = ''
        loopf = True
        while loopf:
            c = self.ser.read()
            if (len(c) == 0):
                s = ''
                loopf = False
            else:
                if c == chr(0x02):
                    s=''
                elif c == chr(0x03):
                    loopf = False
                else:
                    s += c

        if s == '':
            return [False, s]
        else:
            sum = 0
            for i in s[:-2]:
                sum += ord(i)
            chk = ast.literal_eval('0x' + s[-2:])
            if (sum + chk) & 0xff == 0:
                return [True, s[:-2]]
            else:
                return [False, s]

    def send(self, command):
        i=0
        for t in command:
            i = i+ord(t)
        t=hex(65536-i)
        s = chr(0x02)+command+t[len(t)-2:].upper()+chr(0x03)
        self.ser.write(chr(0x02)+command+t[len(t)-2:].upper()+chr(0x03))

if __name__ == '__main__':
    try:
        manager = abh3Transfer()
    except rospy.ROSInterruptException:
            pass

