#! /usr/bin/env python

import numpy as np
import roslib
import rospy
import rosbag
from std_msgs.msg import Int32, String, Int64, Float32, Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from time import sleep
import serial
import sys

portName = "/dev/ttyUSB0"
rMotSpeed = 0
lMotSpeed = 0

if len(sys.argv) == 2:
    portName = sys.argv[1]

ser = serial.Serial(
    port=portName,
    baudrate=115200
)

def delay(pTime):
    sleep(pTime / 1000.0)

def parseSensData(pData):
    pass

def parseImuData(pData, qx_pub, qy_pub, qz_pub, qw_pub, imu_pub):
    parseData = pData.split('|')
    if len(parseData) == 6:
        qxVal = float(parseData[1])
        qyVal = float(parseData[2])
        qzVal = float(parseData[3])
        qwVal = float(parseData[4])

        qx_pub.publish(qxVal)
        qy_pub.publish(qyVal)
        qz_pub.publish(qzVal)
        qw_pub.publish(qwVal)

        imu_msg = Imu()
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "base_link"

        imu_msg.header = h

        imu_msg.orientation_covariance = (-1., )*9	
        imu_msg.angular_velocity_covariance = (-1., )*9
        imu_msg.linear_acceleration_covariance = (-1., )*9


        imu_msg.orientation.x = qxVal
        imu_msg.orientation.y = qyVal
        imu_msg.orientation.z = qzVal
        imu_msg.orientation.w = qwVal

        imu_pub.publish(imu_msg)


def parseEncData(l_enc, r_enc, pData):
    parseData = pData.split('|')
    if len(parseData) == 4:
        leftEnc = long(parseData[1])
        rightEnc = long(parseData[2])

        l_enc.publish(leftEnc)
        r_enc.publish(rightEnc)

def main(pSer):

    rospy.init_node('arduino_node')
    #rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    sens_pub = rospy.Publisher('/sensor_data', String, queue_size=1)
    qx_pub = rospy.Publisher('qx',Float32,queue_size = 10)
    qy_pub = rospy.Publisher('qy',Float32,queue_size = 10)
    qz_pub = rospy.Publisher('qz',Float32,queue_size = 10)
    qw_pub = rospy.Publisher('qw',Float32,queue_size = 10)
    lEncoder_pub = rospy.Publisher('lwheel', Int64, queue_size=10)
    rEncoder_pub = rospy.Publisher('rwheel', Int64, queue_size=10)
    rospy.Subscriber('/left_wheel_speed', Float32, update_left_wheel_speed)
    rospy.Subscriber('/right_wheel_speed', Float32, update_right_wheel_speed)
    imu_pub = rospy.Publisher('imu/data', Imu,queue_size = 10)

    print "**************************************"
    print "Starting to pub imu and sensor data..."
    print "**************************************"

    pSer.write("#imu|1|")
    pSer.write("#enc|1|")
    pSer.write("#man|0|")

    #Dont forget to disable manuelcontrol!!!!

    while not rospy.is_shutdown():

        if pSer.inWaiting() > 0:
            tData = pSer.readline()
            #print tData

            if tData[0] == '#':
                # sens_pub.publish(tData)
                parseSensData(tData)
            elif tData[0] == '$':
                parseImuData(tData, qx_pub, qy_pub, qz_pub, qw_pub, imu_pub)
                # imu_pub.publish(tData)
            elif tData[0] == 'e':
                parseEncData(lEncoder_pub, rEncoder_pub, tData)


        else:
            pass

    pSer.write("#imu|0|")
    pSer.write("#enc|0|")
    pSer.write("#man|1|")
    pSer.write("#res|")

def update_left_wheel_speed(pData):
    global lMotSpeed
    lMotSpeed = pData.data
    if lMotSpeed != 0: 
        if lMotSpeed > 0: lMotSpeed = lMotSpeed + 70
        if lMotSpeed < 0: lMotSpeed = lMotSpeed - 70
    if lMotSpeed > 255: lMotSpeed = 255
    if lMotSpeed < -255: lMotSpeed = -255
    sendStr = '#spd|' + str(rMotSpeed) + '|' + str(lMotSpeed) + '|'
    # rospy.loginfo("send left: (%d, %d)", lMotSpeed, rMotSpeed) 
    ser.write(sendStr)

def update_right_wheel_speed(pData):
    global rMotSpeed
    rMotSpeed = pData.data
    if rMotSpeed != 0: 
        if rMotSpeed > 0: rMotSpeed = rMotSpeed + 70
        if rMotSpeed < 0: rMotSpeed = rMotSpeed - 70
    if rMotSpeed > 255: rMotSpeed = 255
    if rMotSpeed < -255: rMotSpeed = -255
    sendStr = '#spd|' + str(rMotSpeed) + '|' + str(lMotSpeed) + '|'
    # rospy.loginfo("send right: (%d, %d)", lMotSpeed, rMotSpeed) 
    ser.write(sendStr)

# def cmd_vel_callback(p_data):
#     v_right = 0.0
#     v_left = 0.0
#     pass
    

if __name__ == '__main__':
    main(ser)