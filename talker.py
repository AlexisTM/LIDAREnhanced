#!/usr/bin/env python
import rospy
from laserpack.msg import distance

def talker():
    pub = rospy.Publisher('/custom', distance, queue_size=10)
    msg = distance()
    msg.X1 = 13
    msg.X2 = 12
    msg.Y1 = 137
    msg.Y2 = 14
    msg.Z1 = 168
    msg.Z2 = 17

    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass