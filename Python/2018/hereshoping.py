import numpy as np
from numpy import linalg as LA
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers import Activation


np.random.random_sample()

############################################################################################################################################################################################################################
#creates diagonalizable array of size row x col, with random values between 0 and 1. returns final matrix of converged power iteration.

def createArray(row, col):

	#instantiates diagonalizable matrix
		
	P = np.matrix(np.random.rand(row,col))
	while(LA.det(P)==0):
		P = np.matrix(np.random.rand(row,col))
	vec = np.random.rand(row)	#diagonal matrix (eigenvalues)
	D=np.diag(vec)
	print(max(vec))
	A = np.matmul(np.matmul(P,D),LA.inv(P))
	return A

	
	

###################################################################################################################################################################################################################################################################################
#instantiate 4D array Z of size numEigen x convergeLength, that has power iterations of numEigen matrices

n = int(input("N for NxN matrix: "))
convergeLength = int(input("Converge length: "))
numEigen = int(input("Number of matrices to test: "))
tol = 1e-6

y = np.zeros((numEigen,convergeLength))
Z = np.zeros((numEigen,convergeLength,n+1),float)
#A = createArray(n,n)
#print("A: ", A)

#randomized initial guess for eigenvector
x = np.random.random([n,1])
print("x: ", x)


for i in range(numEigen):
	A = createArray(n,n)
	eigenvec = np.matmul(LA.matrix_power(A,0),x)	#eigenvector for each power iteration
	eigenval = np.dot(np.transpose(np.matmul(A,eigenvec)),eigenvec)/np.dot(np.transpose(eigenvec),eigenvec) #eigenval calculation
	eigenvec = np.r_[eigenvec,eigenval] # appending eigenval to eigenvec
	Z[i,0,:] = np.reshape(eigenvec,n+1)
	for j in range(1,convergeLength):
		#power iteration
		eigenvec = np.matmul(LA.matrix_power(A,j+1),x)	#eigenvector for each power iteration
		eigenval = np.dot(np.transpose(np.matmul(A,eigenvec)),eigenvec)/np.dot(np.transpose(eigenvec),eigenvec) #eigenval calculation
		eigenvec = np.r_[eigenvec,eigenval] # appending eigenval to eigenvec
		Z[i,j,:] = np.reshape(eigenvec,n+1)
		y[i,j-1] = eigenval #Rayleigh's Equation to calculate eigenvalue for next entry
	eigenvec = np.matmul(LA.matrix_power(A,j+1),x)	
	eigenval = np.dot(np.transpose(np.matmul(A,eigenvec)),eigenvec)/np.dot(np.transpose(eigenvec),eigenvec) 
	eigenvec = np.r_[eigenvec,eigenval] 
	y[i,j] = eigenval

print(Z[:,:,:])
print(y)

## ISSUES

#Testing with 3x3 matrices, can compare printed y values to initial eigenvalues in A - are generally pretty close
#value of convergelength and numEigen are totally arbitrary - not sure on what criterion to base these on.
#what is the relationship between number of iterations for convergence (w/specified tolerance) and size of matrix?
#is the y vector set up correctly with only the final eigenvalues, or should it be a matrix with the eigenvalue at each iteration?
#reshaping 4D Z array into 3D for LSTM to read in matrix iterations as input and accept eigenvalue for output?


###################################################################################################################################################################################################################################################################################
#LSTM 

model = Sequential()
model.add(LSTM(8,input_shape=(convergeLength,n+1),return_sequences=False))#True = many to many
model.add(Dense(convergeLength,kernel_initializer='normal',activation='linear'))
model.add(Dense(convergeLength,kernel_initializer='normal',activation='linear'))
model.compile(loss='mse',optimizer ='adam',metrics=['accuracy'])
model.fit(Z,y,epochs=50,batch_size=10,validation_split=0.05,verbose=1);
scores = model.evaluate(Z,y,verbose=1,batch_size=5)
print('Accurracy: {}'.format(scores[1])) 
import matplotlib.pyplot as plt
predict=model.predict(Z)
print(predict[-1,-1])








