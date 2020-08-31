import numpy as np
import matplotlib.pyplot as plt
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.models import load_model
 
 
subInt = 25
seq_length = 10
dx = seq_length/subInt

# return training data
X = [[dx*(i+j) for j in range(subInt)] for i in range(subInt)]
X = np.array(X)
y = [[np.sin(dx*(i+j)) for i in range(subInt)] for j in range(subInt)]
y = np.array(y)

plt.plot(X[0],y[0])
plt.show()
X = X.reshape(subInt, len(X), 1)
y = y.reshape(subInt,len(y))


print(X)
print(y)


# define model
model = Sequential()
model.add(LSTM(1, input_shape=(subInt,1)))
model.add(Dense(subInt, activation='linear'))

# compile model
model.compile(loss='mse', optimizer='adam')

# fit model
model.fit(X, y, epochs=100, shuffle=False, validation_split=0.05,verbose=1)
scores = model.evaluate(X,y,verbose=0,batch_size=5)
print(scores)


yNew = model.predict(X)

print(yNew)
X=X.reshape(subInt,subInt)
plt.plot(X[0],y[0])
plt.plot(X[0],yNew[0])
plt.show()

# save model to single file
#model.save('lstm_model.h5')