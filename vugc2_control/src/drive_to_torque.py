#!/usr/bin/env python

import rospy
from vugc2_control.msg import Torque_param
from vugc2_control.msg import Drive_param
from vugc2_control.srv import DriveService, DriveServiceRequest, DriveServiceResponse
from std_msgs.msg import Bool
from numpy import interp


control_torque_parameters = rospy.Publisher('vugc2_control_torque_parameters', Torque_param, queue_size=10)

def drive(driveParams):

    rospy.wait_for_service('vugc2_control_drive_service')
    try:
        driveService = rospy.ServiceProxy('vugc2_control_drive_service', DriveService)
        torqueParams = driveService(driveParams)

        return torqueParams.torque_params
    except rospy.ServiceException:
        print('[vugc2_control_drive_service] service call failed')
        return None

def callback(driveParams):
    print(driveParams)
    torqueParams = drive(DriveServiceRequest(driveParams))
    control_torque_parameters.publish(torqueParams)



def main():
    rospy.init_node('vugc2_control_driver', anonymous=True)
    rospy.Subscriber('vugc2_control_drive_parameters', Drive_param, callback)
    rospy.spin()


if __name__ == '__main__':
    print('[vugc2_control_driver] initialized')
    main()
