#!/usr/bin/env python

import rospy
from vugc1_control.msg import steering_values
from vugc1_control.msg import drive_param
from vugc1_control.srv import VisualEncoder
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from numpy import interp


control_torque_parameters = rospy.Publisher('vugc1_control_torque_parameters', steering_values, queue_size=10)

desired_angle = 0
perceived_angle = 0
kp = 0
ki = 0
kd = 0
cte = 0
prev_cte = 0
acc_cte = 0

voltage_maximum_difference = 1.5
voltage_center = 2.5
voltage_low = voltage_center - (voltage_maximum_difference / 2.0)
voltage_high = voltage_center + (voltage_maximum_difference / 2.0)
steering_low = -100
steering_high = 100
torque_low = 4095 / 5.0 * voltage_low
torque_high = 4095 / 5.0 * voltage_high


def get_steering_angle(message):
    rospy.wait_for_service('vugc1_control_visual_encoder')
    try:
        visual_encoder = rospy.ServiceProxy('vugc1_control_visual_encoder', VisualEncoder)
        angle = visual_encoder(message)

        return angle
    except rospy.ServiceException:
        print('[vugc1_control_steering_pid] service call failed')
        return None


def on_drive_parameters(data):
    global desired_angle
    desired_angle = data.angle


def on_image(image):
    global perceived_angle
    perceived_angle = get_steering_angle(image)

    global cte, prev_cte, acc_cte
    cte = perceived_angle - desired_angle
    diff_cte = cte - prev_cte
    prev_cte = cte
    acc_cte += cte

    voltage_difference = -kp * cte - ki * acc_cte - kd * diff_cte
    # clamp
    voltage_difference = interp(voltage_difference, [-1 * voltage_maximum_difference, voltage_maximum_difference], [-1 * voltage_maximum_difference, voltage_maximum_difference])
    voltage_1 = voltage_center + (voltage_difference / 2.0)
    voltage_2 = voltage_center - (voltage_difference / 2.0)

    # volts to torque
    torque_1 = int(interp(voltage_1, [voltage_low, voltage_high], [torque_low, torque_high]))
    torque_2 = int(interp(voltage_2, [voltage_low, voltage_high], [torque_low, torque_high]))

    print('(kp, ki, kd)={}, volts={}, torque={}'.format((kp, ki, kd), (voltage_1, voltage_2), (torque_1, torque_2)))

    parameters = steering_values()
    parameters.trq_1 = torque_1
    parameters.trq_2 = torque_2
    control_torque_parameters.publish(parameters)


def main():
    rospy.init_node('vugc1_control_steering_pid', anonymous=True)
    rospy.Subscriber('vugc1_control_drive_parameters', drive_param, on_drive_parameters)
    rospy.Subscriber('zed/rgb/image_rect_color', Image, on_image)
    rospy.spin()


if __name__ == '__main__':
    print('[vugc1_control_steering_pid] initialized')
    main()
