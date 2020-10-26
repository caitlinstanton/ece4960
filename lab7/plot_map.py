import numpy as np  #used for generating arrays
import matplotlib.pyplot as plot  #used for graphs to visualize data
from scipy import pi
from scipy.fftpack import fft  #used for the fft algorithm
import pandas as pd
import csv
import math

r = []
theta = []

plot.axes(projection='polar')

with open('tmp.csv', 'r') as csv_file:
    next(csv_file)
    csv_reader = csv.reader(
        csv_file)  # Making use of reader method for reading the file
    for line in csv_reader:  #Iterate through the loop to read line by line
        r.append(float(line[9]))
        t = math.radians(float(line[13]))
        theta.append(t)

# Set the title of the polar plot
plot.title('Kitchen Mapping Using ToF and Gyroscope Data')

for i in range(0, len(theta)):
    plot.polar(theta[i], r[i], 'o')

# Display the Polar plot
plot.show()