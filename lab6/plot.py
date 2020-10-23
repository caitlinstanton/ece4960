import numpy as np  #used for generating arrays
import matplotlib.pyplot as plt  #used for graphs to visualize data
from scipy import pi
from scipy.fftpack import fft  #used for the fft algorithm
import pandas as pd
import csv

temp = pd.read_csv("tmp.csv")
x = []
y = []
z = []
dt = []

with open('tmp.csv', 'r') as csv_file:  #Opens the file in read mode
    next(csv_file)
    csv_reader = csv.reader(
        csv_file)  # Making use of reader method for reading the file
    for line in csv_reader:  #Iterate through the loop to read line by line
        line = line[len(line) - 1:]
        spl = line[0].split(",")
        dt.append(float(spl[0]))
        x.append(float(spl[1]))
        y.append(float(spl[2]))

plt.plot(dt, x, label="Motor Val")
plt.plot(dt, y, label="Yaw Gyro")
# plt.plot(dt, z, label="z")
plt.title('Motor Val & Yaw Gyro vs Time for kp = 5, ki = 0.5')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.legend(loc="upper left")
plt.show()