import numpy as np #used for generating arrays
import matplotlib.pyplot as plt #used for graphs to visualize data
from scipy import pi
from scipy.fftpack import fft #used for the fft algorithm
import pandas as pd 
import csv 

temp = pd.read_csv("tapping_imu.csv",usecols=['x','y','z','dt'])
x = []
y = []
z = []
dt = []

with open('tapping_imu.csv','r') as csv_file: #Opens the file in read mode
    next(csv_file)
    csv_reader = csv.reader(csv_file) # Making use of reader method for reading the file
    for line in csv_reader: #Iterate through the loop to read line by line
        line = line[len(line)-4:]
        x.append(float(line[0]))
        y.append(float(line[1]))
        z.append(float(line[2]))
        dt.append(float(line[3]))

T = np.mean(dt)
print(T)

# sample_rate = 1024 #number of samples taken per second
N = 1000 #size of the array where only 2 seconds of data is available

time = np.linspace(0,2,N) #returns an array of the size specified with evenly spaced samples, calculated from starting to ending value

plt.plot (x, dt)
plt.title ('Time Domain Signal')
plt.xlabel ('Time')
plt.ylabel ('Amplitude')
plt.show ()

frequency = np.linspace (0.0, 512, int (N/2)) #creates an array with frequencies to plot, ends at 512 due to Nyquist-Shannon sampling theorem

freq_data = fft(x) #moves data from time domain to frequency domain
y = 2/N * np.abs (x [0:np.int (N/2)]) #np.abs takes absolute value, only plotting positive frequencies

plt.plot(frequency, y)
plt.title('Frequency domain Signal')
plt.xlabel('Frequency in Hz')
plt.ylabel('Amplitude')
plt.show()
