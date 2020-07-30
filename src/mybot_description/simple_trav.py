#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion
import math
import time
from pyproj import Geod
from geometry_msgs.msg import Twist
import sys 

yaw =0 
gps_angle=0
lat1=0
lon1=0
lat2 = sys.argv[1]
lon2 = sys.argv[2]




def callback(pose):
    global yaw
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    euler = euler_from_quaternion(quaternion)
    yaw= math.degrees(euler[2]) +180
    yaw = abs(yaw-360)
    yaw = yaw%360


def callback2(data):
    global lat1 
    lat1= data.latitude
    global lon1 
    lon1= data.longitude



def listener():
    rospy.init_node('bot_yaw','bot_gps', anonymous=True,disable_signals= True) 
    rospy.Subscriber("imu", Imu, callback)
    rospy.Subscriber("fix", NavSatFix, callback2)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    twist = Twist()  
    
    while 1:
	geodesic =Geod(ellps='WGS84')
	bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
        bearing = bearing +180
	angle_diff = yaw- bearing
        print("Yaw: ", yaw, "Bearing: ", bearing)
        print("Angle: ", angle_diff)       
        
        if angle_diff>1:
            if angle_diff<180:
                twist.angular.z=0.7
            elif angle_diff>180:
                twist.angular.z=-0.7
            pub.publish(twist)

        elif angle_diff<-1:
            angle_diff = abs(angle_diff)
            if angle_diff<180:
                twist.angular.z = -0.7
            elif angle_diff>180:
                twist.angular.z = 0.7
            pub.publish(twist) 
        if angle_diff<1 and angle_diff>-1:
            break
    while 1:
        bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
        if  dist>0.5:
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x=0
            twist.angular.y=0
            twist.angular.z=0
            twist.linear.x= 0.8
            print("Distance: ", dist)
            pub.publish(twist)
            flag=0
        
        elif dist<0.5:
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x=0
            twist.angular.y=0
            twist.angular.z=0
            twist.linear.x = 0
            pub.publish(twist)
            
    
    rospy.spin()

if __name__ == '__main__':
    listener()
