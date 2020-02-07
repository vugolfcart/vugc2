#!/usr/bin/env python

import rospy
from vugc2_control.msg import Drive_param, Torque_param, Angle_param
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from numpy import interp


control_torque_parameters = rospy.Publisher('vugc2_control_torque_parameters', Torque_param, queue_size=10)

desired_angle = 0
perceived_angle = 0
kp = .5
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


def on_drive_parameters(data):
    global desired_angle
    desired_angle = data.angle

def on_angle_parameters(data):
    print("ANGLE CALLBACK")
    print("Desired angle: ", desired_angle)
    print("Perceived angle: ", data.angle)

    global perceived_angle
    perceived_angle = data.angle

    global cte, prev_cte, acc_cte
    cte = perceived_angle - desired_angle
    diff_cte = cte - prev_cte
    prev_cte = cte
    acc_cte += cte

    voltage_difference = -(kp * cte + ki * acc_cte + kd * diff_cte)
    print("PID controlval: ", voltage_difference)
    # clamp
    voltage_difference = interp(voltage_difference, [-1 * voltage_maximum_difference, voltage_maximum_difference], [-1 * voltage_maximum_difference, voltage_maximum_difference])
    voltage_1 = voltage_center + (voltage_difference / 2.0)
    voltage_2 = voltage_center - (voltage_difference / 2.0)

    # volts to torque
    torque_1 = int(interp(voltage_1, [voltage_low, voltage_high], [torque_low, torque_high]))
    torque_2 = int(interp(voltage_2, [voltage_low, voltage_high], [torque_low, torque_high]))

    print('(kp, ki, kd)={}, volts={}, torque={}'.format((kp, ki, kd), (voltage_1, voltage_2), (torque_1, torque_2)))

    parameters = Torque_param()
    parameters.torque1 = torque_1
    parameters.torque2 = torque_2
    control_torque_parameters.publish(parameters)


def main():
    rospy.init_node('vugc2_control_steering_pid', anonymous=True)
    rospy.Subscriber('vugc2_control_drive_parameters', Drive_param, on_drive_parameters)
    rospy.Subscriber('vugc2_control_angle_parameters', Angle_param, on_angle_parameters)

    rospy.spin()


if __name__ == '__main__':
    print('[vugc2_control_steering_pid] initialized')
    main()
