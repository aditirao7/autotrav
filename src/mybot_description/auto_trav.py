#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Imu, NavSatFix, LaserScan, Range
from tf.transformations import euler_from_quaternion
import math
import time
from pyproj import Geod
from geometry_msgs.msg import Twist
yaw =0 

lat1=0
lon1=0
lat2 = sys.argv[1]
lon2 = sys.argv[2]

regions={}

right=0
left=0

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
twist = Twist()

def imu(pose):
	global yaw
	quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

	euler = euler_from_quaternion(quaternion)
	yaw= math.degrees(euler[2])+180
	yaw = abs(yaw-360)
	yaw = yaw%360


def gps(data):
	global lat1 
	lat1= data.latitude
	global lon1 
	lon1= data.longitude

def laser(msg):
	global regions
	newranges=[]
	for i in range(0,640):
		newranges.append(msg.ranges[i])
		if math.isnan(msg.ranges[i]) is True:
			newranges[i]=10
	regions = {'right':  min(min(newranges[0:213]), 10),'front':  min(min(newranges[214:426]), 10),'left':   min(min(newranges[427:640]), 10)}


def rightclbk(data):
	global right
	right=data.range

def leftclbk(data):
	global left
	left=data.range

def align(angle):
	while 1:
		angle_diff = yaw-angle       
		if angle_diff>1:
		    if angle_diff<180:
		        twist.angular.z=1
		    elif angle_diff>180:
		        twist.angular.z=-1
		    pub.publish(twist)

		elif angle_diff<-1:
		    angle_diff = abs(angle_diff)
		    if angle_diff<180:
		        twist.angular.z = -1
		    elif angle_diff>180:
		        twist.angular.z = 1
		    pub.publish(twist) 
		if angle_diff<1 and angle_diff>-1:
		    break

def gostraight():
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x=0
	twist.angular.y=0
	twist.angular.z=0
	twist.linear.x= 1
	pub.publish(twist)

def stop():
	twist.linear.y = 0
	twist.linear.z = 0
	twist.angular.x=0
	twist.angular.y=0
	twist.angular.z=0
	twist.linear.x = 0
	pub.publish(twist) 



def autotrav():
	rospy.init_node('bot_yaw','bot_gps', anonymous=True,disable_signals= True) 
	rospy.Subscriber("imu", Imu, imu)
	rospy.Subscriber("fix", NavSatFix, gps)
	rospy.Subscriber("scan", LaserScan, laser)
	rospy.Subscriber("right", Range, rightclbk)
	rospy.Subscriber("left", Range, leftclbk)
    
	rate = rospy.Rate(10) # 10hz  

	geodesic =Geod(ellps='WGS84')
	while True:
		bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
		bearing = bearing+180
		if  dist>0.5:
			if bool(regions):
				print(regions, right, left)
				if regions['front'] > 1.5 and regions['left'] > 1.5 and regions['right'] > 1.5:
					print("go forward")
					while regions['front'] > 1.5 and regions['left'] > 1.5 and regions['right'] > 1.5 and dist>0.5:
						bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
						bearing = bearing+180
						align(bearing)
						gostraight()
				
				elif regions['front'] < 1.5 and regions['left'] > 1.5 and regions['right'] > 1.5:
					if regions['left']>=regions['right']:
						print("follow left wall")
						align(yaw-45)
						while right<2.5 and regions['front']>1.5:
							gostraight()
						stop()
						if right>2 and left>2:
							bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
							bearing = bearing+180
							align(bearing)

				                     
					else:
						print("follow right wall")
						align(yaw+45)
						while left<2.5 and regions['front']>1.5:
							gostraight()
						stop()
						if right>2 and left>2:
							bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
							bearing = bearing+180
							align(bearing)
		               
			
				elif regions['front'] > 1.5 and regions['left'] > 1.5 and regions['right'] < 1.5:
					print("follow left wall")
					align(yaw-30)
					while right<2.5 and regions['front']>1.5:
						gostraight()
					stop()
					if right>2 and left>2:
						bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
						bearing = bearing+180
						align(bearing)
				
				elif regions['front'] > 1.5 and regions['left'] < 1.5 and regions['right'] > 1.5:
					print("follow right wall")
					align(yaw+30)
					while left<2.5 and regions['front']>1.5:
						gostraight()
					stop()
					if right>2 and left>2:
						bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
						bearing = bearing+180
						align(bearing)
				
				elif regions['front'] < 1.5 and regions['left'] > 1.5 and regions['right'] < 1.5:
					print("follow left wall")
					align(yaw-45)
					while right<2.5 and regions['front']>1.5:
						gostraight()
					stop()
					if right>2 and left>2:
						bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
						bearing = bearing+180
						align(bearing)
				
				elif regions['front'] < 1.5 and regions['left'] < 1.5 and regions['right'] > 1.5:
					print("follow right wall")
					align(yaw+45)
					while left<2.5 and regions['front']>1.5:
						gostraight()
					stop()
					if right>2 and left>2:
						bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
						bearing = bearing+180
						align(bearing)

				elif regions['front'] < 1.5 and regions['left'] < 1.5 and regions['right'] < 1.5:
					if regions['left']>=regions['right']:
						print("follow left wall")
						align(yaw-65)
						while right<2.5 and regions['front']>1.5:
							gostraight()
						stop()
						if right>2 and left>2:
							bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
							bearing = bearing+180
							align(bearing)

					else:
						print("follow right wall")  
						align(yaw+65)
						while left<2.5 and regions['front']>1.5:
							gostraight()
						stop()
						if right>2 and left>2:
							bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
							bearing = bearing+180
							align(bearing)

				elif regions['front'] > 1.5 and regions['left'] < 1.5 and regions['right'] < 1.5:
					print("go forward")
					while right>1 and left>1 and regions['front']>1.5:
						gostraight()
						if regions['front']<1.5:
							stop()
							break
					stop()
					if right>2 and left>2:
						bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
						bearing = bearing+180
						align(bearing)
			
		
		if dist<0.5:
			bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
			bearing = bearing+180
			align(bearing)
			stop()
			exit()
		
        
	rospy.spin()

if __name__ == '__main__':
    autotrav()
