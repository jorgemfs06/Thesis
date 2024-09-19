#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from ackermann_msgs.msg import AckermannDriveStamped


def odom_callback(data):
    global last_time, current_time, x, y, th, vx, vy, vth, covariance_twist, covariance_pose
    global odom_pub, odom_broadcaster  # , twist_pub
    current_time = rospy.Time.now()

    speed = data.drive.speed  # / 1.03
    angle = data.drive.steering_angle  # *0.9
    wheelbase = 2.55  # * 1.1

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = vx * cos(th) * dt
    delta_y = vx * sin(th) * dt
    delta_th = vth * dt
    x += delta_x
    y += delta_y
    th += delta_th

    width = 1.475
    # method 3 from  https://answers.ros.org/question/296112/odometry-message-for-ackerman-car/
    vx = speed
    vy = 0.0
    vth = math.tan(angle)*(speed/wheelbase)
    # if angle == 0:
    #     vth = 0
    # else:
    #     vth = speed/math.sqrt(math.pow(wheelbase, 2)/math.pow(angle, 2) + math.pow(wheelbase, 2)/4)

    odomMsg = Odometry()
    odomMsg.header.stamp = rospy.Time.now()
    odomMsg.twist.twist.linear.x = vx
    odomMsg.twist.twist.linear.y = vy
    odomMsg.twist.twist.angular.z = vth
    odomMsg.twist.covariance = covariance_twist

    odomMsg.header.frame_id = 'odom'
    odomMsg.child_frame_id = 'base_link'

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # set the position
    odomMsg.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    odomMsg.pose.covariance = covariance_pose

    # publish the message
    odom_pub.publish(odomMsg)
    last_time = current_time


def main():
    global last_time, current_time, x, y, th, vx, vy, vth, covariance_twist, covariance_pose
    global odom_pub, odom_broadcaster, twist_pub

    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    # twist_pub = rospy.Publisher("/ackermann_steering_controller/cmd_vel", Twist, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    covariance_twist = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 1000.0]
    covariance_pose = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       1000.0]

    rospy.Subscriber('ackermann_steering_controller/ackermann_drive', AckermannDriveStamped, odom_callback, queue_size=10)
    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0
    vy = 0.0
    vth = 0.0

    rospy.spin()
