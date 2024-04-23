#!/usr/bin/env python
"""Create flightroom map

Creates flightroom map file in format move_base accepts.

"""

__authors__ = "D. Knowles"
__date__ = "29 Nov 2022"

import os

import cv2
import rospkg
import numpy as np
import matplotlib.pyplot as plt

pkg_path = rospkg.RosPack().get_path("navlab_turtlebot_sim")

# # plot previous map
# img_path = os.path.join(pkg_path,"maps","turtlebot3_world.pgm")
# img = cv2.imread(img_path)
# plt.figure()
# plt.imshow(img)

resolution = 0.05 # map resolution
square = 50.0 # meters
img_size = int(square/resolution)
img_flightroom = 205*np.ones((img_size,img_size,3),dtype=np.uint8)

flightroom_x = [int(x*0.0254/resolution) for x in [-311,343]]
flightroom_y = [int(y*0.0254/resolution) for y in [-130,132]]

x_low = int(img_size/2) + flightroom_x[0]
x_hi = int(img_size/2) + flightroom_x[1]
y_low = int(img_size/2) + flightroom_y[0]
y_hi = int(img_size/2) + flightroom_y[1]
# create open space
img_flightroom[y_low:y_hi,x_low:x_hi,:] = 254
# create wall
wall_width = 10
img_flightroom[y_low-wall_width:y_low,x_low-wall_width:x_hi+wall_width] = 0
img_flightroom[y_hi:y_hi+wall_width,x_low-wall_width:x_hi+wall_width] = 0
img_flightroom[y_low-wall_width:y_hi+wall_width,x_low-wall_width:x_low] = 0
img_flightroom[y_low-wall_width:y_hi+wall_width,x_hi:x_hi+wall_width] = 0

img_flightroom_path = os.path.join(pkg_path,"maps","flightroom.pgm")
cv2.imwrite(img_flightroom_path, cv2.cvtColor(img_flightroom,cv2.COLOR_BGR2GRAY))

plt.figure()
plt.imshow(img_flightroom)
plt.show()
