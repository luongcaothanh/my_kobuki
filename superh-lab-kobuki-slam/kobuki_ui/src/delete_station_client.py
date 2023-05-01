#!/usr/bin/env python

import sys
import rospy
from kobuki_ui.srv import DeleteStation

def delete_station_client(station_id):
    rospy.wait_for_service('delete_station')
    try:
        delete_station = rospy.ServiceProxy('delete_station', DeleteStation)
        response = delete_station(station_id)
        if response.success:
            print("Station deleted successfully")
        else:
            print("Station not found")
    except rospy.ServiceException as e:
        print("Service call failed: ", e)

if __name__ == '__main__':
    if len(sys.argv) == 2:
        station_id = str(sys.argv[1])
    else:
        print("Usage: delete_station_client.py [station_id]")
        sys.exit(1)
    delete_station_client(station_id)

