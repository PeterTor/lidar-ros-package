#!/usr/bin/python
from RiegelList import *
import roslib
import rospy
import tf
import turtlesim.msg
import tf.msg
import geometry_msgs.msg
import math
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi
from numpy import *
import os
currentPath=os.path.dirname(os.path.abspath(__file__))
#used to generate the output file for quadrant.py
text_file = open(currentPath+"/Output_Sectors.py", "w")
outputString =("UTM=[")

#initialising some values
right_list = []
left_list = []
left_list_all = []
left_list_transformed=[]
Calculated=False
counter = 0
yaw=0
scale=0
tx=0
ty=0
#x_test=548503.782313
#y_test=5804492.16207
#yaw_test = 13.7951457972*pi/180
#s_test=1.0069310283
class MapTransformator:

	# center of mass.
	def compute_center(self, point_list):
    		# Safeguard against empty list.
    		if not point_list:
        		return (0.0, 0.0)
    		# If not empty, sum up and divide.
    		sx = sum([p[0] for p in point_list])
    		sy = sum([p[1] for p in point_list])
    		return (float(sx) / len(point_list), float(sy) / len(point_list))
	def getRotMatrix(self,yaw):
		rotmatrix = array([[cos(yaw), -sin(yaw)],[sin(yaw),cos(yaw)]])
		return rotmatrix

	def Distance(self,x1,y1,x2,y2):
		return math.sqrt((x1-x2)**2 + (y1-y2)**2)
	
	def calculateError(self):
		global yaw,tx,ty,scale,left_list,left_list_transformed,right_list,text_file
		rmse=0
		rmseX=0
		rmseY=0
		for i in range(len(left_list)):
			xy= array([left_list[i][0],left_list[i][1]])
			#print xy
			xy_tran=dot(xy,self.getRotMatrix(yaw))*scale+array([tx,ty])
			left_list_transformed.append(xy_tran)
		print "left list ",left_list
		#print len(left_list_transformed)
		print "right list ",right_list

		for i in range(len(left_list_transformed)):
			x1=left_list_transformed[i][0]
			x2=right_list[i][0]
			y1=left_list_transformed[i][1]
			y2=right_list[i][1]
			#print "x_l x_r ",x1,x2
			#print "y_l y_r ",y1,y2
			rmse+=self.Distance(x1,y1,x2,y2)**2
			rmseX += (x1-x2)**2
			rmseY += (y1-y2)**2

		
		print "RMSE ",math.sqrt(rmse/4)
		print "RMSE X ",math.sqrt(rmseX/4)
		print "RMSE Y ",math.sqrt(rmseY/4)
		
	
	def start(self):
		global Calculated,yaw,tx,ty,scale,outputString
		# Schwerpunkte
		lc = self.compute_center(left_list)
		rc = self.compute_center(right_list)
		print "center ",rc
		# Initialisierung
		cs = 0.0
		ss = 0.0
		ll = 0.0
		rr = 0.0
		la = 1.0

		#seen_left = set()
		#uniq_left = [x for x in left_list if x not in seen_left and not seen_left.add(x)]
		#seen_right = set()
		#uniq_right = [x for x in right_list if x not in seen_right and not seen_right.add(x)]

		# reduzierte Punkte
		left_red = [(pt[0] - lc[0], pt[1] - lc[1]) for pt in left_list]
		right_red = [(pt[0] - rc[0], pt[1] - rc[1]) for pt in right_list]

		#
		for i in range(len(right_list)):
    			# cosin-Summe
    			cs += right_red[i][0] * left_red[i][0] + right_red[i][1] * left_red[i][1]
    			# sin-Summe
    			ss += -right_red[i][0] * left_red[i][1] + right_red[i][1] * left_red[i][0]
    			# reduzierte, quadrierte Punkte
    			rr += right_red[i][0] ** 2 + right_red[i][1] ** 2
    			ll += left_red[i][0] ** 2 + left_red[i][1] ** 2

		# Lambda
		la = sqrt(rr/ll)
		# cos
		c = cs / sqrt(cs ** 2 + ss ** 2)
		# sin
		s = ss / sqrt(cs ** 2 + ss ** 2)
		# Rotation	
		yaw = atan2(s, c)
		yaw_d = yaw * 180 / pi
		# Translation
		tx = rc[0] - la * (c * lc[0] - s * lc[1])
		ty = rc[1] - la * (s * lc[0] + c * lc[1])

		scale=la
		yaw=-1*yaw_d*pi/180
		#yaw=yaw_test
		#tx=x_test
		#ty=y_test
		#scale=s_test
		print "Scale (lambda) =", scale
		#print "Rotation c = ", c
		#print "Rotation s = ", s
		print "yaw = ", yaw_d
		print "tx = ", tx
		print "ty = ", ty
		outputString+=("["+str(rc[0])+","+str(rc[1])+","+str(yaw)+","+str(scale)+","+str(tx)+","+str(ty)+"],")
		Calculated=True
		self.calculateError()




	def setPoint(self,data):
		global left_list_all,Calculated,left_list_transformed
		left_list_all.append([data.point.x,data.point.y])
		le= len(left_list_all)
		print "clicked points ", le
		list12=len(right_list1)+len(right_list2)
		if le== list12 or le==list12+len(right_list3) or le ==list12+len(right_list3)+len(right_list4):
			left_list_transformed=[]
			Calculated=False

	def __init__(self):
		global left_list,left_list_all,right_list,right_list1,right_list2,right_list3,right_list4,Calculated,text_file,outputString
        	rospy.Subscriber('/clicked_point', geometry_msgs.msg.PointStamped, self.setPoint)
        	#self.pub_tf = rospy.Publisher("/global_pose", Odometry,queue_size=10)
 
        	while not rospy.is_shutdown() :
			if len(left_list_all)==len(right_list1) and not Calculated:
				left_list=left_list_all
				right_list=right_list1
				self.start()	

			if len(left_list_all)==len(right_list1)+len(right_list2) and not Calculated:
				left_list=[]
				for i in range(4):
					left_list.append(left_list_all[4+i])
				right_list=right_list2
				self.start()

			if len(left_list_all)==len(right_list1)+len(right_list2)+len(right_list3) and not Calculated:
				left_list=[]
				for i in range(4):
					left_list.append(left_list_all[8+i])
				right_list=right_list3
				self.start()

			if len(left_list_all)==len(right_list1)+len(right_list2)+len(right_list3)+len(right_list4) and not Calculated:
				left_list=[]
				for i in range(4):
					left_list.append(left_list_all[12+i])
				right_list=right_list4
				self.start()
				#finish output				
				outputString=outputString[:-1]
				outputString+="]"
				text_file.write(outputString)
				text_file.close()
if __name__ == '__main__':
    rospy.init_node('my_MapTransformator')
    tfb = MapTransformator()
    rospy.spin()



