#!/usr/bin/env python
from math import radians, sqrt, sin, cos
import rospy, numpy as np
from statistics import mean
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import std_msgs.msg
#from thanos.General_Functions import now, tic, toc

imu_msg = Imu()
odom_msg = Odometry()

def listener(hz=10):


    rate = rospy.Rate(hz) # 10hz
    rate.sleep()



if __name__ == '__main__':
#    start = tic()
    rospy.init_node('arduino_listener', anonymous=True)

    # rospy.Subscriber("odom", Odometry, get_accel)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    imu_msg.header.frame_id = 'imu_link'
    while not rospy.is_shutdown():
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z = np.random.randn(3)
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = np.random.uniform(size=4)
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z = np.random.randn(3)
        odom_msg.twist.twist.angular.x, odom_msg.twist.twist.angular.y, odom_msg.twist.twist.angular.z = np.random.randn(3)

        # listener()
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = np.random.uniform(size=4)
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = np.random.randn(3)
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = np.random.randn(3)
        odom_pub.publish(odom_pub)
        imu_pub.publish(imu_msg)
    rospy.spin()
