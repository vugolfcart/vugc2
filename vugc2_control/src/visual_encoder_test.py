#!/usr/bin/env python

from sensor_msgs.msg import Image
from vugc1_control.msg import Drive_param
from vugc1_control.srv import VisualEncoder
import argparse
import cv2
import numpy as np
import rospy


control_drive_parameters = rospy.Publisher('vugc1_control_drive_parameters', Drive_param, queue_size=10)


def get_steering_angle(message):
    rospy.wait_for_service('vugc1_control_visual_encoder')
    try:
        visual_encoder = rospy.ServiceProxy('vugc1_control_visual_encoder', VisualEncoder)
        angle = visual_encoder(message)

        return angle
    except rospy.ServiceException:
        print('[vugc1_control_steering_pid] service call failed')
        return None


def callback(message):
    print("[vugc1_control_steering_pid#callback]: received image")

    angle = get_steering_angle(message)
    print('[vugc1_control_steering_pid#callback] angle={}'.format(angle))


def main():
    rospy.init_node('vugc1_control_steering_pid', anonymous=True)
    rospy.Subscriber("zed/rgb/image_rect_color", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
