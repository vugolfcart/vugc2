#!/usr/bin/env python
# coding: utf-8

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vugc1_control.srv import VisualEncoder
import cv2 as cv
import numpy as np
import rospy
from scipy.spatial.distance import cosine
import math

bridge = CvBridge()


def callback(message):
    print("[vugc1_control_visual_encoder#callback]: received image", message.img.header.stamp)
    try:
        image = bridge.imgmsg_to_cv2(message.img)
        image = image[480:600, 590:720]
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        ret,thresh = cv.threshold(image,0,255,0)
        edged = cv.Canny(image,0,300)

        kernel = np.ones((5,5),np.uint8)
        closed = cv.morphologyEx(edged, cv.MORPH_CLOSE, kernel)

        #cnts, contours, hierarchy = cv.findContours(closed.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        #cv.drawContours(image, contours, -1, (0,255,0), 3)

        #cv.imwrite('img.png', closed)

        mat = np.argwhere(closed != 0)
        mat[:, [0,1]] = mat[:, [1, 0]]
        mat = np.array(mat).astype(np.float32)

        m, e = cv.PCACompute(mat, mean = np.array([]))

        center = tuple(m[0])
        top = np.array((m[0][0]+1, m[0][1])).astype(np.float32)

        # center to endpoint1: 1st principal component

        base_vector = top - m[0]
        pc1 = e[0]
        pc1[1] = -pc1[1]

        angle = (cosine(base_vector, pc1))

        return angle
    except CvBridgeError as e:
        print(e)


def main():
    rospy.init_node('vugc1_control_visual_encoder')
    service = rospy.Service('vugc1_control_visual_encoder', VisualEncoder, callback)
    print("[vugc1_control_visual_encoder] initialized")
    rospy.spin()

if __name__=='__main__':
    main()
