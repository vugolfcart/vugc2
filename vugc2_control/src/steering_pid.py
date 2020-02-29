#!/usr/bin/env python

import rospy
from vugc2_control.msg import Drive_param, Torque_param, Angle_param
from vugc2_control.srv import DriveService, DriveServiceRequest, DriveServiceResponse
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

DESIRED_ANGLE = 0
PERCEIVED_ANGLE = 0

def on_angle_parameters(data):
    global PERCEIVED_ANGLE
    PERCEIVED_ANGLE = data.angle

def drive(driveParams):

    rospy.wait_for_service('vugc2_control_drive_service')
    try:
        driveService = rospy.ServiceProxy('vugc2_control_drive_service', DriveService)
        torqueParams = driveService(driveParams)

        return torqueParams.torque_params
    except rospy.ServiceException:
        print('[vugc2_control_drive_service] service call failed')
        return None

def on_drive_parameters(data):
    global DESIRED_ANGLE
    DESIRED_ANGLE = data.angle
    print("Desired angle: ", DESIRED_ANGLE)
    print("Perceived angle: ", data.angle)

    # TODO: Arduino doesnt seem to read back angle changes anymore

    global cte, prev_cte, acc_cte
    cte = PERCEIVED_ANGLE - DESIRED_ANGLE
    diff_cte = cte - prev_cte
    prev_cte = cte
    acc_cte += cte

    target = -(kp * cte + ki * acc_cte + kd * diff_cte)

    print("PID virtual target angle: ", PERCEIVED_ANGLE + target)

    print('(kp, ki, kd)={}'.format((kp, ki, kd)))

    driveParams = Drive_param(target, 0) # 0 velocity 

    torqueParams = drive(DriveServiceRequest(driveParams))

    control_torque_parameters.publish(torqueParams)


def main():
    rospy.init_node('vugc2_control_steering_pid', anonymous=True)
    rospy.Subscriber('vugc2_control_drive_parameters', Drive_param, on_drive_parameters)
    rospy.Subscriber('vugc2_control_angle_parameters', Angle_param, on_angle_parameters)

    rospy.spin()


if __name__ == '__main__':
    print('[vugc2_control_steering_pid] initialized')
    main()
