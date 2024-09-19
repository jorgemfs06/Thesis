#!/usr/bin/env python3

import rospy 
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

def assemble_and_publish():
    rospy.init_node("assemble_scans_to_cloud")
    rospy.wait_for_service("assemble_scans2")
    assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
    pub = rospy.Publisher ("/laser_pointcloud", PointCloud2, queue_size=1)

    rate = rospy.Rate(0.1)  # Adjust the loop rate as needed

    while not rospy.is_shutdown():
        try:
            resp = assemble_scans(rospy.Time(0), rospy.get_rostime())
            rospy.loginfo("Got cloud with %u points" % len(resp.cloud.data))
            pub.publish(resp.cloud)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        rate.sleep()

if __name__ == "__main__":
    try:
        assemble_and_publish()
    except rospy.ROSInterruptException:
        pass

