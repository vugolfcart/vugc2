import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cosine
import math

image = cv.imread('steering.png')
image = image[300:430, 520:700]
image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
ret,thresh = cv.threshold(image,120,140,0)

edged = cv.Canny(image,270,300)

kernel = np.ones((5,5),np.uint8)
closed = cv.morphologyEx(edged, cv.MORPH_CLOSE, kernel)

#cnts, contours, hierarchy = cv.findContours(closed.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

#cv.drawContours(image, contours, -1, (0,255,0), 3)

cv.imwrite('img.png', closed)

mat = np.argwhere(closed != 0)
mat[:, [0,1]] = mat[:, [1, 0]]
mat = np.array(mat).astype(np.float32)

m, e = cv.PCACompute(mat, mean = np.array([]))

center = tuple(m[0])
top = np.array((m[0][0]+1, m[0][1])).astype(np.float32)
endpoint1 = tuple(m[0] + e[0]*10)
endpoint2 = tuple(m[0] + e[1]*50)

# center to endpoint1: 1st principal component

print(center)
print(top)
print(endpoint1)
base_vector = top - m[0]
pc1 = e[0]
pc1[1] = -pc1[1]

angle = (cosine(base_vector, pc1))

print(angle)

cv.circle(image, center, 5, 255)
cv.line(image, center, endpoint1, 255)
cv.line(image, center, tuple(top), 255)
#cv.line(image, center, endpoint2, 255)
cv.imwrite("out.bmp", image)




# m, e = cv.PCACompute(contours, mean = np.array([]))
# print(m)

