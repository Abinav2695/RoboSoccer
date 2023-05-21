# implement the helper functions similar to 'ssl_common/geometry.hpp'
import math
from ctypes import *
#from geometry_msgs.msg import Pose2D

PI = 3.14159265358979323
INF = 9999999

INVALID_X,INVALID_Y = 999,999
NOT_IN_RANGE_X,NOT_IN_RANGE_Y = 1999,1999

class Vector2D(Structure):
	
	_fields_ =	[ ("x", c_float),
			 ("y", c_float) ]

	def __init__(self,x = None,y = None):
		if x is None:
			self.x = self.y = INF
		elif isinstance(x,Vector2D):
			self.x, self.y = x.x, x.y
		else:
			self.x,self.y=x,y

	def valid(self):
		if math.fabs(self.x) == INF or math.fabs(self.y) == INF :
			return False
		else :
			return True

	# Normalizes the angle (in radians) to be in the range (-pi, pi]
	def normalizeAngle(self,angle):
		if angle > PI :
			return angle - 2 * PI
		elif angle <= -PI :
			return angle + 2 * PI
		else :
			return angle

	# Sets the vector using polar coordinates
	def fromPolar(self,r,theta):
		v = Vector2D(0,0)
		v.x = r*math.cos(theta)
		v.y = r*math.sin(theta)
		return v

	# Returns the absolute value of the vector
	def abs(self,v):
		return math.sqrt(v.x*v.x+v.y*v.y)

	# Returns the squared absolute value of the vector
	def absSq(self,v):
		return (v.x*v.x+v.y*v.y)

	# Returns the angle made by the vector (head - self) in the range -pi to pi
	def angle(self,head=None):
		if head is None:
			return math.atan2(self.y,self.x)
		else:
			return math.atan2(head.y-self.y,head.x-self.x)

	def tan_inverse(self):
		if math.atan2(self.y,self.x) < 1.5707963 and math.atan2(self.y,self.x) > -1.5707963:
			return math.atan2(self.y,self.x)
		if math.atan2(self.y,self.x) > 1.5707963 and math.atan2(self.y,self.x) < 3.14159265:
			return math.atan2(self.y,self.x)-3.14159265
		else:
			return math.atan2(self.y,self.x)+3.14159265

	# Returns the Eucledian distance between the 2 vectors
	def dist(self,another_point):
		return math.sqrt(math.pow(another_point.x-self.x,2)+math.pow(another_point.y-self.y,2))

	# Returns the squared Eucledian distane between 2 vectors
	def distSq(self,another_point):
		return math.pow(another_point.x-self.x,2)+math.pow(another_point.y-self.y,2)

	def dot(self,another_point):
		return self.x*another_point.x + self.y*another_point.y

	def __eq__(self,another_point):
		if self.x == another_point.x and self.y == another_point.y :
			return True
		else :
			return False

	def __ne__(self,another_point):
		if self.x == another_point.x and self.y == another_point.y :
			return False
		else :
			return True

	def __add__(self,another_point):
		return Vector2D(self.x+another_point.x,self.y+another_point.y)

	def __sub__(self,another_point):
		return Vector2D(self.x-another_point.x,self.y-another_point.y)

	def __mul__(self,scale):
		return Vector2D(self.x * scale , self.y * scale)

	def __truediv__(self,scale):
		if scale == 0 :
			raise Exception('Tried scaling down vector by zero')
		else :
			return Vector2D(self.x/scale,self.y/scale)

	def __str__(self):
		return str(self.x)+","+str(self.y)
	
	# self is the point to be checked if it is within the circle with the center and radius as provided
	def intersects(self,center,radius):
		if self.distSq(center) < radius * radius :
			return True
		else :
			return False
    
   
	#self is the center of the circle 
	def intersects(self,point1,point2,radius):
		# Source of algorithm used: http://stackoverflow.com/questions/1073336/circle-line-collision-detection 
		m,c = point1.slope_const(point2) #slope of line formed by point1 and point2
		p,q=self.x,self.y
		r=radius

		A=(m*m+1)
		B=2*((m*c)-(m*q)-(p))
		C=(q*q - r*r + p*p - 2*c*q + c*c)

		dis = B*B - 4*A*C
		if dis >=0 :
			dis = math.sqrt(dis)
			#print(a,b,c)
			x1 = (dis-B)/(2*A)
			x2 = (-dis-B)/(2*A)
			y1 = m*x1+c
			y2 = m*x2 +c
			if(x1>0):
				return Vector2D(x1,y1)
			elif(x2>0):
				return Vector2D(x2,y2)
			
		return Vector2D(INVALID_X,INVALID_Y)
	

	#self is the center of the circle
	def intersects_slope(self,m,c,radius):
		# Source of algorithm used: http://stackoverflow.com/questions/1073336/circle-line-collision-detection 
		#m,c = point1.slope_const(point2) #slope of line formed by point1 and point2
		p,q=self.x,self.y
		r=radius

		A=(m*m+1)
		B=2*((m*c)-(m*q)-(p))
		C=(q*q - r*r + p*p - 2*c*q + c*c)

		dis = B*B - 4*A*C
		if dis >=0 :
			dis = math.sqrt(dis)
			#print(a,b,c)
			x1 = (dis-B)/(2*A)
			x2 = (-dis-B)/(2*A)
			y1 = m*x1+c
			y2 = m*x2 +c
			if(x1>0 and x2>0):
				if(x1>x2):
					return Vector2D(x1,y1)
				else:
					return Vector2D(x2,y2)
			elif(x2>0):
				return Vector2D(x2,y2)
			elif(x1>0):
				return Vector2D(x1,y1)

		return Vector2D(-1,-1)

    #slope of line formedby point1 and point2
	def slope_const(self,another_point):
		slope = (another_point.y-self.y)/(another_point.x-self.x)
		constant =self.y-(slope * self.x)
		return slope,constant
	
	



