#!/usr/bin/python
import roslib
 
import csv
import rospy
import tf
import turtlesim.msg
import tf.msg
import geometry_msgs.msg
from math import sin, cos, pi
from numpy import *

from nav_msgs.msg import Odometry
utm_x=548702.612632
utm_y= 5804442.9321
yaw = 101.368818251*pi/180
scale=1.00693547981
writer_hector=0
with open('hector_utm.csv', 'w') as csvfile:
    fieldnames = ['time','x_utm', 'y_utm']
    writer_hector = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer_hector.writeheader()

with open('amcl_utm.csv', 'w') as csvfile:
    fieldnames = ['time','x_utm', 'y_utm','quat_x','quat_y','quat_z','quat_w','roll','pitch','yaw','cov_11','cov_12','cov_21','cov_22','cov_66']
    writer_amcl = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer_amcl.writeheader()

class DynamicTFBroadcaster:

    def getRotMatrix(self,yaw):
	rotmatrix = array([[cos(yaw), -sin(yaw)],[sin(yaw),cos(yaw)]])
	return rotmatrix
       

    def translateOdom_Amcl(self,data):
	global yaw,scale,utm_x,utm_y
	#init msg
	t = geometry_msgs.msg.PoseWithCovarianceStamped()
        t.header.frame_id = "odom"
	t.header.stamp = data.header.stamp
	
	#transform params
	xy= array([data.pose.pose.position.x,data.pose.pose.position.y])
	xy_tran=dot(xy,self.getRotMatrix(yaw))*scale+array([utm_x,utm_y])
	
	t.pose.pose.position.x=xy_tran[0]
	t.pose.pose.position.y=xy_tran[1]
	t.pose.pose.position.z=0
    	quaternion = (
		data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	current_yaw = euler[2]	
	newyaw = current_yaw + yaw
	
	quat=tf.transformations.quaternion_from_euler(roll,pitch,newyaw)
	#t.pose.pose.orientation=quat
	
	t.pose.pose.orientation.x = quat[0]
    	t.pose.pose.orientation.y = quat[1]
    	t.pose.pose.orientation.z = quat[2]
    	t.pose.pose.orientation.w = quat[3]
    	covar=data.pose.covariance
	t.pose.covariance=data.pose.covariance
	#tfm = tf.msg.tfMessage([t])
    	self.pub_odom_Amcl.publish(t)
	with open('amcl_utm.csv', 'a') as csvfile:
		fieldnames = ['time','x_utm', 'y_utm','quat_x','quat_y','quat_z','quat_w','roll','pitch','yaw','cov_11','cov_12','cov_21','cov_22','cov_66']
    		writer_hector = csv.DictWriter(csvfile, fieldnames=fieldnames)
		writer_hector.writerow({'time': data.header.stamp ,'x_utm': xy_tran[0], 'y_utm': xy_tran[1], 'quat_x':quat[0],'quat_y':quat[1],'quat_z':quat[2],'quat_w':quat[3], 'roll':roll, 'pitch':pitch,'yaw':newyaw, 'cov_11':covar[0],'cov_12':covar[1],'cov_21':covar[6],'cov_22':covar[7],'cov_66':covar[35]})
	
	
	
	

    def translateOdom_Hector(self,data):
	global yaw,scale,utm_x,utm_y,writer_hector
	#init msg
	t = geometry_msgs.msg.PoseStamped()
        t.header.frame_id = "odom"
	t.header.stamp = data.header.stamp
	
	#transform params
	xy= array([data.pose.position.x,data.pose.position.y])
	xy_tran=dot(xy,self.getRotMatrix(yaw))*scale+array([utm_x,utm_y])
	
	t.pose.position.x=xy_tran[0]
	t.pose.position.y=xy_tran[1]
	t.pose.position.z=0
	
	t.pose.orientation.x = 0.0
    	t.pose.orientation.y = 0.0
    	t.pose.orientation.z = 0.0
    	t.pose.orientation.w = 1.0
	#t.covariance=data.pose.covariance
	#tfm = tf.msg.tfMessage([t])
        self.pub_odom_Hector.publish(t)
	with open('hector_utm.csv', 'a') as csvfile:
		fieldnames = ['time','x_utm', 'y_utm']
    		writer_hector = csv.DictWriter(csvfile, fieldnames=fieldnames)
		writer_hector.writerow({'time': data.header.stamp ,'x_utm': xy_tran[0], 'y_utm': xy_tran[1]})
   

    def __init__(self):
        rospy.Subscriber('/amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self.translateOdom_Amcl)
	rospy.Subscriber('/slam_out_pose', geometry_msgs.msg.PoseStamped, self.translateOdom_Hector)
        self.pub_odom_Amcl = rospy.Publisher("/utm/amcl_pose", geometry_msgs.msg.PoseWithCovarianceStamped,queue_size=20)
	self.pub_odom_Hector = rospy.Publisher("/utm/slam_out_pose", geometry_msgs.msg.PoseStamped,queue_size=20)
 
  
 
           
 
if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    tfb = DynamicTFBroadcaster()
    rospy.spin()
