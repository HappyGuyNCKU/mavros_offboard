import math
import numpy as np

# rotate matrix creator
def r_matrix(deg, out=None):
	rad = (deg/360.0)*2*math.pi
	cos = math.cos(rad)
	sin = math.sin(rad)
	matri = np.array([[cos,sin],[-1*sin,cos]])
	if out != None:
		out = matri
	return matri



class local_q:
	w=1
	x=0
	y=0
	z=0

	def __init__(self,i=0,j=0,k=0,degree=0):
		mag = math.sqrt(i*i+j*j+k*k)
		half_t = (math.pi/2.0/180.0*degree)
		self.w = math.cos(half_t)
		self.x = i/mag*math.sin(half_t)
		self.y = j/mag*math.sin(half_t)
		self.z = k/mag*math.sin(half_t)

	def printq(self):
		print self.w        
		print self.x
		print self.y
		print self.z


class Quaternion:
	w=1
	x=0
	y=0
	z=0
	degree = 0

	vector = [1,0]

	angle = 5
    

	#rotation of quaternion is clockwise
	q_stable = local_q(0,1,1,1)
	q_shift_left = local_q(1,0,0,-angle)
	q_shift_right = local_q(1,0,0,angle) 
	q_forward = local_q(0,1,0,angle)
	q_backward = local_q(0,1,0,-angle)
	q_rotate_cw = local_q(0,0,1,90)
	q_rotate_cc = local_q(0,0,1,-90)

	def set_q(self,q_local):
		self.w = q_local.w
		self.x = q_local.x
		self.y = q_local.y
		self.z = q_local.z     

	def shift_left(self):
		self.roll = 1
		self.update_qua(0)


	def shift_right(self):
		self.roll = -1
		self.update_qua(0)

	def forward(self):
		self.pitch = 1
		self.update_qua(0)

	def backward(self):
		self.pitch = -1
		self.update_qua(0)

	def rotate_cw(self,deg):
		self.degree += deg
		m = rotate_matrix r_matrix(deg)
		self.vector = m.dot(self.vector)	
		self.update_qua(1)

	def rotate_cc(self,deg):
		self.degree -= deg
		m = rotate_matrix r_matrix(-deg)
		self.vector = m.dot(self.vector)
		self.update_qua(1)

	def update_qua(self,option)
		if option == 1:
			print "update option 1"
			deg = math	
			pass
		else if option == 0:
			print "update option 0"
			pass
		else:
			print "invalid option"
			pass


	def multipy(self,local):
		#(Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
		#(Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
		#(Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
		#(Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)
		temp = Quaternion()
		temp.w = (local.w * self.w - local.x * self.x - local.y * self.y - local.z * self.z)
		temp.x = (local.w * self.x + local.x * self.w + local.y * self.z - local.z * self.y)
		temp.y = (local.w * self.y - local.x * self.z + local.y * self.w + local.z * self.x)
		temp.z = (local.w * self.z + local.x * self.y - local.y * self.x + local.z * self.w)

		self.w = temp.w
		self.x = temp.x
		self.y = temp.y
		self.z = temp.z
#self.printq()

	def quaternion(self,i,j,k,degree):
		mag = math.sqrt(i*i+j*j+k*k)
		print mag
		half_t = (math.pi/2.0/180.0*degree)
		print half_t
		self.w = math.cos(half_t)
		self.x = i/mag*math.sin(half_t)
		self.y = j/mag*math.sin(half_t)
		self.z = k/mag*math.sin(half_t)
		self.printq()	
	
	def printq(self):
		print self.w		
		print self.x
		print self.y
		print self.z
		return self


	
