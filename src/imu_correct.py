#!/usr/bin/env python
import numpy as np

import rospy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

extrinsicRot = np.array([[0, -1, 0],
                         [0, 0, 1],
                         [-1, 0, 0]])
extrinsicRPY = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0]
    ])
extrinsicRPY = R.from_matrix(extrinsicRPY)

print("Rotating imu/data...")
rospy.init_node('imu_rotate', anonymous=False)
pub = rospy.Publisher('imu_correct', Imu, queue_size=10)

def callback(imu):
    # Only calibrate accelerometer and gyroscope
    q = imu.orientation
    a = imu.linear_acceleration
    w = imu.angular_velocity
    qn = np.array([q.x, q.y, q.z, q.w])
    an = np.array([a.x, a.y, a.z])
    wn = np.array([w.x, w.y, w.z])
    q_new = (R.from_quat(qn) * extrinsicRPY).as_quat()
    a_new = np.dot(extrinsicRot, an)
    w_new = np.dot(extrinsicRot, wn)
    imu.orientation.x = q_new[0]
    imu.orientation.y = q_new[1]
    imu.orientation.z = q_new[2]
    imu.orientation.w = q_new[3]
    imu.linear_acceleration.x = a_new[0]
    imu.linear_acceleration.y = a_new[1]
    imu.linear_acceleration.z = a_new[2]
    imu.angular_velocity.x = w_new[0]
    imu.angular_velocity.y = w_new[1]
    imu.angular_velocity.z = w_new[2]
    pub.publish(imu)
    
def listener():
    rospy.spin()

if __name__ == '__main__':
    rospy.Subscriber("imu/data", Imu, callback)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass