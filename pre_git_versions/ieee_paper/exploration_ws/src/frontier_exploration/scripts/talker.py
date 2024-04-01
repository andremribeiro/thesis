#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped

def talker():
    pub = rospy.Publisher('best_frontier', PointStamped, queue_size=10)
    rospy.init_node('point_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        point_stamped = PointStamped()
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.frame_id = "uav1/world_origin"
        point_stamped.point.x = 18
        point_stamped.point.y = 18
        point_stamped.point.z = 2
        rospy.loginfo(point_stamped)
        pub.publish(point_stamped)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
