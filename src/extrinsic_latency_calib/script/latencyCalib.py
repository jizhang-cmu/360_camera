#!/usr/bin/env python
import numpy as np
import os
import matplotlib.pyplot as plt

image_scale = 0.065;
image_latency = 0;

imu_array  = []
imu_time_array = []

image_array  = []
image_time_array = []

with open('./../data/imu_latency.txt') as imu_file:
  for raw_line in imu_file:
    line = raw_line.rstrip().split()
    
    imu, time = line
    
    imu = float(imu)
    time = float(time)

    imu_array.append(imu)
    imu_time_array.append(time)

with open('./../data/image_latency.txt') as image_file:
  for raw_line in image_file:
    line = raw_line.rstrip().split()
    
    image, time = line
    
    image = float(image)
    time = float(time)

    image_array.append(image_scale * image)
    image_time_array.append(time - image_latency)

plt.axhline(y = 0, color = 'black', linestyle = '-') 
plt.plot(imu_time_array, imu_array,'-', markersize=3, color="red", label="IMU")
plt.plot(image_time_array, image_array,'-', markersize=3, color="blue", label="Image")
plt.legend(loc="upper right")
plt.xlabel('Time (s)')
plt.ylabel('Yaw rate (rad/s)')

plt.show()
