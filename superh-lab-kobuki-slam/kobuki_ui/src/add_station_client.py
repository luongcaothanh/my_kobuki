#!/usr/bin/env python

import rospy
from kobuki_ui.srv import AddStation
import sys

def station_client():
    rospy.wait_for_service('add_station')
    try:
        add_station = rospy.ServiceProxy('add_station', AddStation)
        name = sys.argv[1]
	positionX = float(sys.argv[2])
	positionY = float(sys.argv[3])
	positionZ = float(sys.argv[4])
	orientationX = float(sys.argv[5])
	orientationY = float(sys.argv[6])
	orientationZ = float(sys.argv[7])
	orientationW = float(sys.argv[8])
	id = sys.argv[9]
	response = add_station(name, positionX, positionY, positionZ, orientationX, orientationY, orientationZ, orientationW, id)
	print(response.success)
    except rospy.ServiceException as e:
        print("Service call failed: ", e)

if __name__ == '__main__':
    station_client()

