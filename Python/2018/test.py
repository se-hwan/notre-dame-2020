#Load Packages
import numpy as np
from numpy import linalg as LA
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers import Activation


X = np.zeros((100,5,2))
Y = np.zeros((100,2))
for i in range(100):
	for j in range(5):
		X[i,j,0] = np.sin(i+j)
		X[i,j,1] = np.cos(i+j)
	Y[i,0] = (i+5)
	Y[i,1] = (i+5)


	
z = np.zeros((2,2,2,2),float)
A = [[2,-12],[1,-5]]
x = [[1],[1]]
print(LA.matrix_power(A,6))
print(np.matmul(LA.matrix_power(A,6),x))
B = [[5,6],[7,8]]
C = [[9,10],[11,12]]
D = [[13,14],[15,16]]	
z[0,0,:,:] = A
z[0,1,:,:] = B
z[1,0,:,:] = C
z[1,1,:,:] = D


#print(z) 
#print(z[1,1,:,:])



