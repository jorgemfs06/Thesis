#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

class LaserToPointCloud:
    def __init__(self):
        rospy.init_node('laser_to_pointcloud')
        self.laser_projector = LaserProjection()
        self.pc_pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)
        rospy.spin()

    def laser_scan_callback(self, laser_scan):
        try:
            point_cloud = self.laser_projector.projectLaser(laser_scan)
            self.pc_pub.publish(point_cloud)
        except Exception as e:
            rospy.logerr("Failed to project laser scan to point cloud: %s", str(e))

if __name__ == '__main__':
    try:
        LaserToPointCloud()
    except rospy.ROSInterruptException:
        pass


