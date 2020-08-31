import numpy as np
from numpy import linalg as LA

np.random.random_sample()


row=10
col = 10
#instantiates diagonalizable matrix
P = np.matrix(np.random.rand(row,col))
while(LA.det(P)==0):
	P = np.matrix(np.random.rand(row,col))
#vec=np.array([np.random.randint(0,high=4),np.random.randint(4,high=8),np.random.randint(8,high=11)])

vec = np.random.rand(row)
#np.random.uniform -> for diagonal vector
D=np.diag(vec)
A = np.matmul(np.matmul(P,D),LA.inv(P))

print("P:")
print(P)
print("D:")
print(D)
print("A:")
print(A)

#tolerance and iteration number
tol = 1e-6
n=0

#instantiates intitial eigenvector estimate
x = np.random.random([row,1])
print(x)


xnew = np.array([0,0,0])
xnew = xnew.transpose()

while(1):
	xnew = np.dot(A,x)
	xnew = (1/np.abs(xnew).max())*xnew
	print(xnew)
	#Raleigh quotient
	ev = np.vdot(np.dot(A,xnew),xnew)/np.vdot(xnew,xnew)
	n=n+1
	print("EV: ", ev, "N: ",n)
	if(LA.norm(np.abs(x)-np.abs(xnew))<tol):
		break
	x=xnew

