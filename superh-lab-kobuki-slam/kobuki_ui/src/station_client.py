#!/usr/bin/env python

import rospy
from kobuki_ui.srv import GetStationList

def station_client():
    rospy.wait_for_service('get_station_list')
    try:
        get_station_list = rospy.ServiceProxy('get_station_list', GetStationList)
        response = get_station_list()
	print(response.station_list)
    except rospy.ServiceException as e:
        print("Service call failed: ", e)

if __name__ == '__main__':
    station_client()

