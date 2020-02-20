#!/usr/bin/env python

import rospy
from vugc2_control.msg import Torque_param
from vugc2_control.msg import Drive_param
from vugc2_control.srv import DriveService
from std_msgs.msg import Bool
from numpy import interp

voltage_maximum_difference = 1.5
voltage_center = 2.5
voltage_low = voltage_center - (voltage_maximum_difference / 2.0)
voltage_high = voltage_center + (voltage_maximum_difference / 2.0)
steering_low = -100
steering_high = 100
torque_low = 4095 / 5.0 * voltage_low
torque_high = 4095 / 5.0 * voltage_high


def angleToVolts(angle):
    difference = interp(angle, [steering_low, steering_high], [-1 * voltage_maximum_difference, voltage_maximum_difference])
    voltage1 = voltage_center + (difference / 2.0)
    voltage2 = voltage_center - (difference / 2.0)
    return voltage1, voltage2

def voltsToTorque(voltage1, voltage2):
    torque1 = int(interp(voltage1, [voltage_low, voltage_high], [torque_low, torque_high]))
    torque2 = int(interp(voltage2, [voltage_low, voltage_high], [torque_low, torque_high]))
    return torque1, torque2

# def wrapTorqueParams(torque1, torque2):
#     parameters = Torque_param(torque1, torque2)
#     return parameters

def callback(data):
    angle, velocity = data.drive_params.angle, data.drive_params.velocity

    voltage1, voltage2 = angleToVolts(angle)

    torque1, torque2 = voltsToTorque(voltage1, voltage2)

    print('angle={}, volts={}, torque={}'.format(angle, (voltage1, voltage2), (torque1, torque2)))

    return Torque_param(torque1, torque2)


def main():
    rospy.init_node('vugc2_control_drive_service', anonymous=True)
    service = rospy.Service('vugc2_control_drive_service', DriveService, callback)
    rospy.spin()


if __name__ == '__main__':
    print('[vugc2_control_drive_service] initialized')
    main()
