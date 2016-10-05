import math

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

	angle = 5

	q_shift_left = local_q(1,0,0,-angle)
	q_shift_right = local_q(1,0,0,angle) 
	q_forward = local_q(0,1,0,-angle)
	q_backward = local_q(0,1,0,angle)
	q_rotate_cw = local_q(0,0,1,90)
	q_rotate_cc = local_q(0,0,1,-90)

	def shift_left(self):
		self.multipy(self.q_shift_left)

	def shift_right(self):
		self.multipy(self.q_shift_right)

	def forward(self):
		self.multipy(self.q_forward)

	def backward(self):
		self.multipy(self.q_backward)

	def rotate_cw(self):
		self.multipy(self.q_rotate_cw)

	def rotate_cc(self):
		self.multipy(self.q_rotate_cc)

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


	
