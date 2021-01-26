#!/usr/bin/env python
import numpy as np

import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix

f = plt.figure()
filter_points = np.empty((0, 2), float)


def callback1()

def callback(nav_sat_fix):
    global filter_points
    lat = nav_sat_fix.latitude
    lon = nav_sat_fix.longitude
    filter_points = np.append(filter_points, np.array([[lat, lon]]), axis=0)


if __name__ == '__main__':
    rospy.init_node('imu_rotate', anonymous=False)

    rospy.Subscriber("gps/filtered", NavSatFix, callback)
    # rospy.Subscriber("gps/fix", NavSatFix, callback)
    try:
        rospy.spin()
        plt.scatter(filter_points[:, 0], filter_points[:, 1])
        plt.show()
    except rospy.ROSInterruptException:
        pass