#!/usr/bin/env python

import rospy
from vugc1_control.msg import steering_values
from vugc1_control.msg import drive_param
from std_msgs.msg import Bool
from numpy import interp


control_torque_parameters = rospy.Publisher('vugc1_control_torque_parameters', steering_values, queue_size=10)
control_emergency_stop = rospy.Publisher('vugc1_control_emergency_stop', Bool, queue_size=10)

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
    voltage_1 = voltage_center + (difference / 2.0)
    voltage_2 = voltage_center - (difference / 2.0)
    return voltage_1, voltage_2


def callback(data):
    angle = data.angle

    # angle to volts
    difference = interp(angle, [steering_low, steering_high], [-1 * voltage_maximum_difference, voltage_maximum_difference])
    voltage_1 = voltage_center + (difference / 2.0)
    voltage_2 = voltage_center - (difference / 2.0)

    # volts to torque
    torque_1 = int(interp(voltage_1, [voltage_low, voltage_high], [torque_low, torque_high]))
    torque_2 = int(interp(voltage_2, [voltage_low, voltage_high], [torque_low, torque_high]))

    print('angle={}, volts={}, torque={}'.format(angle, (voltage_1, voltage_2), (torque_1, torque_2)))

    parameters = steering_values()
    parameters.trq_1 = torque_1
    parameters.trq_2 = torque_2
    control_torque_parameters.publish(parameters)


def main():
    rospy.init_node('vugc1_control_torque_talker', anonymous=True)
    control_emergency_stop.publish(False)
    rospy.Subscriber('vugc1_control_drive_parameters', drive_param, callback)
    rospy.spin()


if __name__ == '__main__':
    print('[vugc1_control_torque_talker] initialized')
    main()
