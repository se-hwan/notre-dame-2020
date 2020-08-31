#Load Packages
import numpy as np

from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers import Activation


X = np.zeros((100,5,1))
Y = np.zeros((100,2))
for i in range(100):
	for j in range(5):
		X[i,j,0] = np.sin(i+j)
		#X[i,j,1] = np.cos(i+j)
	Y[i,0] = np.sin(i+5)
	#Y[i,1] = np.cos(i+5)


print(X)
print(Y)


model = Sequential()
model.add(LSTM(8,input_shape=(5,1),return_sequences=False))#True = many to many
model.add(Dense(2,kernel_initializer='normal',activation='linear'))
model.add(Dense(2,kernel_initializer='normal',activation='linear'))
model.compile(loss='mse',optimizer ='adam',metrics=['accuracy'])
model.fit(X,Y,epochs=30,batch_size=5,validation_split=0.05,verbose=1);
scores = model.evaluate(X,Y,verbose=1,batch_size=5)
print('Accurracy: {}'.format(scores[1])) 
import matplotlib.pyplot as plt
predict=model.predict(X)


print(predict)


plt.plot(np.arange(5),X[0,:,0], 'o')
plt.plot([5],Y[0,0], 'o')
plt.plot([5],predict[0,0], '*')
Xpredict = np.zeros((1,3,2))
Xpredict = X[0,0:3,:]
print(predict)
#predict1 = model.predict(np.reshape(Xpredict,(1,3,2)))
#plt.plot([3],predict1[0,0],'*')
#print(predict1)

plt.show()