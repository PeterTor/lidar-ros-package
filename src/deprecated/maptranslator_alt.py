#!/usr/bin/python
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

counter = 0


# Karte 1 UTM (Riegel)
#right_list = [[549037.035248, 5804393.386499],
#              [548792.392250, 5804241.108002],
#              [548694.462000, 5804288.638000],
#              [548758.238998, 5804495.077449]]
#
#right_list4 = [[548761.596748, 5804507.910500],
#              [548784.605003, 5804501.913002],
#              [548791.764252, 5804480.997253],
#              [548771.427498, 5804490.885750]]
#
#right_list3 = [[548726.054750, 5804249.485504],
#              [548731.943249, 5804273.946503],
#              [548682.896999, 5804296.583748],
#              [548690.255500, 5804323.631500]]
#
#right_list2 = [[548857.305756, 5804261.950256],
#              [548790.706001, 5804219.539001],
#              [548772.928497, 5804227.792999],
#              [548775.513000, 5804246.867752]]
#
#right_list1 = [[549037.035248, 5804393.386499],
#              [549036.272247, 5804389.348749],
#              [549068.605743, 5804400.832750],
#              [549052.933014, 5804380.619499]]
right_list4 = [[548761.194752, 5804508.533249],
              [548782.054253,5804502.248497],
              [548771.427498,5804490.885750],
              [548758.110249,5804494.836250]]

right_list3 = [[548690.255500,5804323.631500],
              [548727.200500,5804397.368500],
              [548694.530750,5804290.087502],
              [548681.550501,5804295.942749]]

right_list2 = [[548791.652748,5804241.255249],
              [548932.169495,5804345.271004],
              [548886.971756,5804294.472748],
              [548872.698257,5804283.831253]]

right_list1 = [[549036.164490,5804394.624001],
              [549057.634003,5804404.220250],
              [549068.605743,5804400.832750],
              [549034.971008,5804388.450001]]
right_list = []
# Karte 2 (Hector 25Hz)
left_list = []
left_list_all = []
left_list_transformed=[]
Calculated=False
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
		global yaw,tx,ty,scale,left_list,left_list_transformed,right_list
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
		global Calculated,yaw,tx,ty,scale
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
		Calculated=True
		self.calculateError()




	def setPoint(self,data):
		global left_list_all,Calculated,left_list_transformed
		left_list_all.append([data.point.x,data.point.y])
		le= len(left_list_all)
		print "clicked points ", le
		if le==8 or le==12 or le ==16:
			left_list_transformed=[]
			Calculated=False

	def __init__(self):
		global left_list,left_list_all,right_list,right_list1,right_list2,right_list3,right_list4,Calculated
        	rospy.Subscriber('/clicked_point', geometry_msgs.msg.PointStamped, self.setPoint)
        	#self.pub_tf = rospy.Publisher("/global_pose", Odometry,queue_size=10)
 
        	while not rospy.is_shutdown() :
			if len(left_list_all)==4 and not Calculated:
				left_list=left_list_all
				right_list=right_list1
				self.start()
				

			if len(left_list_all)==8 and not Calculated:
				left_list=[]
				for i in range(4):
					left_list.append(left_list_all[4+i])
				right_list=right_list2
				self.start()

			if len(left_list_all)==12 and not Calculated:
				left_list=[]
				for i in range(4):
					left_list.append(left_list_all[8+i])
				right_list=right_list3
				self.start()

			if len(left_list_all)==16 and not Calculated:
				left_list=[]
				for i in range(4):
					left_list.append(left_list_all[12+i])
				right_list=right_list4
				self.start()
if __name__ == '__main__':
    rospy.init_node('my_MapTransformator')
    tfb = MapTransformator()
    rospy.spin()



