#!/usr/bin/env python

import rospy
from vugc2_control.msg import Torque_param
from vugc2_control.msg import Drive_param
from std_msgs.msg import Bool
from numpy import interp


control_torque_parameters = rospy.Publisher('vugc2_control_torque_parameters', Torque_param, queue_size=10)

voltage_maximum_difference = 1.5
voltage_center = 2.5
voltage_low = voltage_center - (voltage_maximum_difference / 2.0)
voltage_high = voltage_center + (voltage_maximum_difference / 2.0)
steering_low = -100
steering_high = 100
torque_low = 4095 / 5.0 * voltage_low
torque_high = 4095 / 5.0 * voltage_high


def angle_to_volt(angle):
    difference = interp(angle, [steering_low, steering_high], [-1 * voltage_maximum_difference, voltage_maximum_difference])
    voltage1 = voltage_center + (difference / 2.0)
    voltage2 = voltage_center - (difference / 2.0)
    return voltage1, voltage2


def callback(data):
    angle = data.angle

    # angle to volts
    difference = interp(angle, [steering_low, steering_high], [-1 * voltage_maximum_difference, voltage_maximum_difference])
    voltage1 = voltage_center + (difference / 2.0)
    voltage2 = voltage_center - (difference / 2.0)

    # volts to torque
    torque1 = int(interp(voltage1, [voltage_low, voltage_high], [torque_low, torque_high]))
    torque2 = int(interp(voltage2, [voltage_low, voltage_high], [torque_low, torque_high]))

    print('angle={}, volts={}, torque={}'.format(angle, (voltage1, voltage2), (torque1, torque2)))

    parameters = Torque_param()
    parameters.torque1 = torque1
    parameters.torque2 = torque2
    control_torque_parameters.publish(parameters)


def main():
    rospy.init_node('vugc2_control_drive_to_torque', anonymous=True)
    rospy.Subscriber('vugc2_control_drive_parameters', Drive_param, callback)
    rospy.spin()


if __name__ == '__main__':
    print('[vugc2_control_drive_to_torque] initialized')
    main()
