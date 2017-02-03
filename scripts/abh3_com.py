#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pkflt
import serial
import rospy
import ast

from abh3.msg import abh3Vel
from abh3.msg import abh3Diag
from abh3.srv import abh3Com
from abh3.srv import abh3ComResponse
from threading import Thread, BoundedSemaphore

abh3_header = 'FFFF'

vel_cmd = ['CP', 'CP']
vel_adrs = [0x5000, 0x5008]
vel_sel = [0, 0]
vel_val = [0, 0]
vel_num = 2
drv_cmd = ['CM', 'CM']
drv_adrs = [0x1012, 0x1022]
drv_val = [0, 0]
drv_num = 2

diag_cmd  = ['CM1054', 'CM1064', 'CM1012', 'CM1022',
             'CM1031', 'CM1041', 'CM103B', 'CM104B',
             'CM1004', 'CM1005', 'CM0002', 'CM0004',
             'CM0003', 'CM0005', 'CM0001', 'CM0000']
diag_pkflt = [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0]
diag_flag  = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
diag_value = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
diag_num = 16

cmd_dict = {'TBL':'CP', 'SERVO':'CP', 'START':'CP', 'SELECT':'CP', 'SELVEL':'', 'ID':'BI', 'DOUT':'CP', 'DIN':'CM'}
cmd_adrs_dict = {'SERVO' :[0x5100, 0x5120],
                 'START' :[0x5101, 0x5121],
                 'SELECT':[0x5107, 0x5127]}
tbl_dict   = {'CS':[0x50e0, 0x10, 0],
              'RV':[0x5000, 0x08, 1],
              'RC':[0x5010, 0x08, 1],
              'TU':[0x5020, 0x60, 1],
              'US':[0x5030, 0x60, 1],
              'TD':[0x5040, 0x60, 1],
              'DS':[0x5050, 0x60, 1],
              'KP':[0x5060, 0x60, 1],
              'KI':[0x5070, 0x60, 1]}
tbl_axis_dict = {'A':0, 'B':1, 'Y':0, 'X':1}
num_dict = {'INT':0, 'EXT':1, 'OFF':0x0000, 'ON':0x8000}

id_num = 6
id_tbl = [0, 2, 4, 5, 8, 0xc]

out_num = 8
out_dict = {'OPEN':0, 'CLOSE':(1<<32)-1}
out_bit = [24, 25, 26, 27, 28, 29, 30, 31]

in_num = 20
in_bit = [0, 1, 2, 3, 24, 25, 26, 27, 28, 29, 30, 31, 16, 17, 18, 19, 20, 21, 22, 23]

class abh3Manager:
    def __init__(self):
        rospy.init_node('abh3_com', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        driver_item = rospy.get_param('~driver/item', 'abh3')
        driver_name = rospy.get_param('~driver/name', 'no1')

        self.pub = rospy.Publisher('/%s/%s/vfbk' % (driver_item, driver_name), abh3Vel, queue_size=1)
        self.vel = rospy.Subscriber('/%s/%s/vcmd' % (driver_item, driver_name), abh3Vel, self.abh3Vel_handler)
        self.srv = rospy.Service('/%s/%s/srv' % (driver_item, driver_name), abh3Com, self.abh3Com_handler)

        port_name = rospy.get_param('~serial/port_name', '/dev/abh3')
        port_baud = rospy.get_param('~serial/baud_rate', 115200)
        self.ser = serial.Serial(port_name, port_baud, timeout=0.1)
        self.semaphore = semaphore
        
        self.diag_rate = rospy.get_param('~diag/rate', 16)
        self.diag_flag = rospy.get_param('~diag/flag', 0xffff)
        for i in range(diag_num):
            if 2**i & self.diag_flag:
                diag_flag[i]=1

        self.diag_pub = rospy.Publisher('/%s/%s/diag' % (driver_item, driver_name), abh3Diag, queue_size=1)
        if self.diag_rate > 0:
            Thread(target=self.diag).start()
        
    def shutdown(self):
        self.ser.close()

    def diag(self):
        rate = rospy.Rate(self.diag_rate)
        i = 0
        diag_sts = abh3Diag()

        while not rospy.is_shutdown():
            for i in range(diag_num):
                if diag_flag[i]==1:
                    self.semaphore.acquire()
                    cmd = abh3_header + diag_cmd[i]
                    flag, ret = self.trans(cmd)
                    self.semaphore.release()
                    if flag:
                        j = int(ret, 16)
                        if diag_pkflt[i] == 1:
                            diag_value[i] = pkflt.pkflt_to_flt(j)
                        else:
                            if j > 2147483647:
                                j -= 4294967296
                            diag_value[i] = j
    
                    diag_sts.posA, diag_sts.posB, \
                    diag_sts.velAY, diag_sts.velBX, \
                    diag_sts.trqA, diag_sts.trqB, \
                    diag_sts.loadA, diag_sts.loadB, \
                    diag_sts.voltM, diag_sts.voltC, \
                    diag_sts.inp, diag_sts.outp, \
                    diag_sts.incontrol, diag_sts.outcontrol, \
                    diag_sts.warning, diag_sts.alarm = diag_value
        
                    self.diag_pub.publish(diag_sts)
                    rate.sleep()

    def trans(self, command, flag=True):
        i=0
        for t in command:
            i = i+ord(t)
        t=hex(65536-i).upper()
        s = chr(0x02)+command+t[len(t)-2:].upper()+chr(0x03)
        self.ser.write(chr(0x02)+command+t[len(t)-2:].upper()+chr(0x03))
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
                if flag == False:
                    return [True, s[:len(s)-2]]
                else:
                    if s[4] == chr(0x15):
                        return [False, s[5:len(s)-2]]
                    else:
                        return [True, s[5:len(s)-2]]
            else:
                return [False, s]

    def sender(self, cmd, adrs, val, wflag):
        s = '0000'+hex(adrs)[2:].upper()
        s = abh3_header+cmd+s[len(s)-4:]
        if wflag: # write
            t = '00000000' + hex(val)[2:].upper()
            s += t[len(t)-8:]
        # send
        self.semaphore.acquire()
        flag, s = self.trans(command=s, flag=True)
        self.semaphore.release()
        return [flag, s]

    def abh3Com_handler(self, req):
        sp = req.command.split(' ')
        if len(sp) > 1:
            cmd = sp[0]
            if cmd in cmd_dict:
                code = cmd_dict[cmd]
                if cmd == 'TBL':
                    num = int(sp[1])
                    if num>7 or num<0:
                        return abh3ComResponse(response='invalid TBL Number')
                    if sp[2] in tbl_dict:
                        tbl = tbl_dict[sp[2]]
                        num += int(tbl[0])
                    else:
                        return abh3ComResponse(response='invalid TBL Command')
                    if sp[3] in tbl_axis_dict:
                        num += int(tbl_axis_dict[sp[3]]) * int(tbl[1])
                    else:
                        return abh3ComResponse(response='invalid TBL Axis')
                    if len(sp) == 4: # read
                        flag, s = self.sender(cmd=code, adrs=num, val=0, wflag=False)
                        if flag == True:
                            val = int('0x'+s[len(s)-8:], 16)
                            if tbl[2] == 1:
                                s = pkflt.pkflt_to_str(val)
                            else:
                                s = str(val)
                        else:
                            s = ''
                        return abh3ComResponse(response=s)
                    else: # write
                        if sp[4] in num_dict:
                            val = num_dict[sp[4]]
                        else:
                            if tbl[2] == 1:
                                val = pkflt.str_to_pkflt(sp[4])
                            else:
                                val = int(sp[4])
                        flag, s = self.sender(cmd=code, adrs=num, val=val, wflag=True)
                        if flag == False:
                            s = ''
                        return abh3ComResponse(response=s)
                elif cmd == 'SERVO' or cmd == 'START':
                    adrs = cmd_adrs_dict[cmd]
                    if sp[1] in num_dict:
                        val = num_dict[sp[1]]
                    else:
                        val = int(sp[1])
                    flag, s = self.sender(cmd=code, adrs=adrs[0], val=val, wflag=True)
                    if flag == True and s == '':
                        if sp[2] in num_dict:
                            val = num_dict[sp[2]]
                        else:
                            val = int(sp[2])
                        flag, s = self.sender(cmd=code, adrs=adrs[1], val=val, wflag=True)
                    else:
                        s = ''
                    return abh3ComResponse(response=s)
                elif cmd == 'SELECT':
                    adrs = cmd_adrs_dict[cmd]
                    for i in range(3):
                        # A
                        if int(sp[1]) & 2**i:
                            val = num_dict['OFF']
                        else:
                            val = num_dict['ON']
                        flag, s = self.sender(cmd=code, adrs=adrs[0]+i, val=val, wflag=True)
                        if flag != True or s != '':
                            s = ''
                            break;
                        # B
                        if int(sp[2]) & 2**i:
                            val = num_dict['OFF']
                        else:
                            val = num_dict['ON']
                        flag, s = self.sender(cmd=code, adrs=adrs[1]+i, val=val, wflag=True)
                        if flag != True or s != '':
                            s = ''
                            break;
                    return abh3ComResponse(response=s)
                elif cmd == 'SELVEL':
                    for i in range(vel_num):
                        num = int(sp[i+1])
                        if num>=0 and num<8:
                            vel_sel[i] = num
                    return abh3ComResponse(response='')
                elif cmd == 'ID':
                    num = int(sp[1])
                    if num>=0 and num<id_num:
                        flag, s = self.sender(cmd=code, adrs=id_tbl[num], val=0, wflag=False)
                        if num == 1:
                            ver = s.split('.')
                            s = ver[0]+'.'+ver[1]+'.'+ver[2]
                        return abh3ComResponse(response=s)
                    elif num == -1:
                        flag, s = self.sender(cmd=code, adrs=id_tbl[1], val=0, wflag=False)
                        return abh3ComResponse(response=s)
                    else:
                        return abh3ComResponse(response='invalid Number')
                elif cmd == 'DOUT':
                    num = int(sp[1])
                    if num>=out_num or num<0:
                        return abh3ComResponse(response='invalid OUT Number')
                    if len(sp) == 2: # read
                        flag, s = self.sender(cmd='CM', adrs=0x0004, val=0, wflag=False)
                        if flag == True:
                            val = int('0x'+s[len(s)-8:], 16)
                            if val & (1<<out_bit[num]):
                                s = 'CLOSE'
                            else:
                                s = 'OPEN'
                        else:
                            s = ''
                        return abh3ComResponse(response=s)
                    elif len(sp) == 3: # write
                        if sp[2] in out_dict:
                            val = out_dict[sp[2]]
                        else:
                            return abh3ComResponse(response='invalid OUT Value')
                        flag, s = self.sender(cmd=code, adrs=0x5180+num, val=val, wflag=True)
                        if flag == False:
                            s = ''
                        return abh3ComResponse(response=s)
                    else:
                        return abh3ComResponse(response='invalid Command')
                elif cmd == 'DIN':
                    num = int(sp[1])
                    if num>=in_num or num<0:
                        return abh3ComResponse(response='invalid IN Number')
                    if len(sp) == 2: # read
                        flag, s = self.sender(cmd=code, adrs=0x0002, val=0, wflag=False)
                        if flag == True:
                            val = int('0x'+s[len(s)-8:], 16)
                            if val & (1<<in_bit[num]):
                                s = 'CLOSE'
                            else:
                                s = 'OPEN'
                        else:
                            s = ''
                        return abh3ComResponse(response=s)
                    else:
                        return abh3ComResponse(response='invalid Command')
                else:
                    return abh3ComResponse(response='invalid Command')
            else:
                return abh3ComResponse(response='invalid Command')
        else:
            s = req.command
            self.semaphore.acquire()
            flag, s = self.trans(command=s, flag=False)
            self.semaphore.release()
            return abh3ComResponse(response=s)

    def abh3Vel_handler(self, msg):
        vel_val = msg.velAY, msg.velBX
        for i in range(vel_num):
            flag, s = self.sender(cmd=vel_cmd[i], adrs=vel_adrs[i]+vel_sel[i], val=pkflt.flt_to_pkflt(vel_val[i]), wflag=True)

        abh3 = abh3Vel()
        for i in range(drv_num):
            flag, s = self.sender(cmd=drv_cmd[i], adrs=drv_adrs[i], val=0, wflag=False)
            if flag == True:
                val = int('0x'+s[len(s)-8:], 16)
                drv_val[i] = pkflt.pkflt_to_flt(val)

        abh3.velAY, abh3.velBX = drv_val
        self.pub.publish(abh3)
    
if __name__ == '__main__':
    try:
        semaphore = BoundedSemaphore(1)
        manager = abh3Manager()
        rospy.spin()
    except rospy.ROSInterruptException:
            pass

