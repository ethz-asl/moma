import numpy as np
from numpy import linalg as LA

EPS = 1e-6

#----- Description -----

# Definition of all utility functions used in the velocity planner and direction 
# estimation classes. 

#-----------------------

def OrthoProjection(direction):
	
	assert np.abs(np.linalg.norm(direction) - 1.0) < EPS
	projection = np.matmul(direction, direction.T)
	
	return np.eye(3) - projection
	
def NullProjection(A):
	
	return np.eye(A.shape[1])-np.matmul(LA.pinv(A), A) 
	
#----- Optimization objectives and constraints -----
	
def O1(x, *args):

	vdesEE = args[0]
	vmean = args[1]
	s = args[2]
	
	return LA.norm(vdesEE - s*x) + LA.norm(vmean - x)
	
def C1(x, *args):

	v = args[0]
	
	return v**2 - x[0]**2 - x[1]**2
