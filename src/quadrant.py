from Output_Sectors import *
from math import sin, cos, pi
from numpy import *

#UTM consists of [UTM_X, UTM_Y, yaw ,scale , tx , ty
#reihenfolge 4,1,2,3
#UTM=[[548697.71887575, 5804475.527936251,-103.169032489,0.996443636592,548702.004246,5804446.8608],
#    [548768.0111275, 5804499.614494249 ,-102.389445745,1.01078305872,548700.818231,5804444.93789],
#    [548937.79930875, 5804447.9039385,-101.238563198,1.00029443052,548701.515945,5804443.13448 ],
#     [548976.486557, 5804428.604877501, -101.10467299,1.00715464332,548699.495529,5804441.909],
#     #[548835.9013520001, #5804290.7840974005,-100.669849723,0.999922290812,548700.596226,5804440.81986],
#     #[548714.44581025, 5804287.19437425,-100.02089665,1.00519004723,548698.036701,5804440.98124]]
#     [548870.8730639999, 5804291.2075635 ,-100.79819304,0.99827539525,548701.345598,5804440.10905],
#     [548698.3843127501, 5804326.75756275 ,-99.490476994,1.02567876732,548697.694572,5804443.9238]]

#put initial sectors into stack
stack=[UTM[0],UTM[1]]
counter = 2

def Distance(x1,y1,x2,y2):
	return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def getWeights(x,y,UTMList):
	globalWeight=0
	weights=[]
	for i in range(2):
		globalWeight+=Distance(x,y,UTMList[i][0],UTMList[i][1])
	for i in range(2):
		#weights.append((1-1/globalWeight*Distance(x,y,UTMList[i][0],UTMList[i][1]))**4)
		weights.append((1-1/globalWeight*Distance(x,y,UTMList[i][0],UTMList[i][1])))
	return weights/sum(weights)

def getRotMatrix(yaw):
	rotmatrix = array([[cos(yaw), -sin(yaw)],[sin(yaw),cos(yaw)]])
	return rotmatrix

def transform(point,weights,UTMstack):
	xy= array([point[0],point[1]])
	
	w1=weights[0]
	yaw1=UTMstack[0][2]*pi/180
	s1=UTMstack[0][3]
	tx1=UTMstack[0][4]
	ty1=UTMstack[0][5]	
	#print w1,yaw1,s1,tx1,tx1
	w2=weights[1]
	yaw2=UTMstack[1][2]*pi/180
	s2=UTMstack[1][3]
	tx2=UTMstack[1][4]
	ty2=UTMstack[1][5]
	#print w2,yaw2,s2,tx2,tx2
	xy_tran=w1*(dot(getRotMatrix(yaw1),xy)*s1+array([tx1,ty1])) + w2*(dot(getRotMatrix(yaw2),xy)*s2+array([tx2,ty2]))
	return xy_tran

def getTransformParams(weights,UTMstack):
	w1=weights[0]
	yaw1=UTMstack[0][2]*pi/180
	s1=UTMstack[0][3]
	tx1=UTMstack[0][4]
	ty1=UTMstack[0][5]	

	w2=weights[1]
	yaw2=UTMstack[1][2]*pi/180
	s2=UTMstack[1][3]
	tx2=UTMstack[1][4]
	ty2=UTMstack[1][5]
	
	return array([w1,yaw1,s1,tx1,ty1]),array([w2,yaw2,s2,tx2,ty2])

#returns transformed x,y 
def start(x,y,utm_x,utm_y):
	global counter
 	weights=getWeights(utm_x,utm_y,stack)
	if weights[1] >=0.95 and counter<len(UTM):
		print "pop weights"
		stack.pop(0)
		stack.append(UTM[counter])
		counter += 1
	weights=getWeights(utm_x,utm_y,stack)
	#print "weight ",weights
	#print stack
	#print stack
	#print "normal xy ",x,y
	xy_tran=transform([x,y],weights,stack)
	#print "transf.xy ",xy_tran[0],xy_tran[1]
	#print "stack ",stack
	(t_array1,t_array2)=getTransformParams(weights,stack)
	return (xy_tran[0],xy_tran[1],t_array1,t_array2)

#print "punkt4"
#start( 242.105453491,57.6422309875,548754.37387,5804490.39912)

#print "punkt1"
#start(541.669311523,32.2509155273,549041.216495,5804393.64287)

#print "punkt2"
#start(334.913879395,-174.562179565,548789.376709,5804241.0461)

#print "punkt3"
#start( 233.642974854,-146.174179077,548697.161421,5804293.12186)


