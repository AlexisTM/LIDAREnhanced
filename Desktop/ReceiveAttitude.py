#!/usr/bin/env python
# vim:set ts=4 sw=4 et:

 
import rospy
import mavros
 
from sensor_msgs.msg import Imu



def callback(Imu):
  
     rospy.loginfo(rospy.get_caller_id() + "X : %s", Imu.orientation.x)
     rospy.loginfo(rospy.get_caller_id() + "Y : %s", Imu.orientation.y)
     rospy.loginfo(rospy.get_caller_id() + "Z : %s", Imu.orientation.z)

def ReceiveAttitude():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ReceiveAttitude')
    #mavros.set_namespace()  # initialize mavros module with default namespace
    rate = rospy.Rate(10)
    rospy.loginfo("Init passe")


# Suscriber : 
# 1 : On va lire le topic chatter 
# 2 : On utilise le type de messahe String 
# 3 : Le noeuds appellera callback a chaque nouveau message 

  #   rospy.Subscriber(mavros.get_topic('imu_pub', 'data'),Imu, callback)
    rospy.Subscriber('/mavros/imu/data',Imu, callback)
    rospy.loginfo("Suscriber passe")
    rospy.loginfo("AvantSpin")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    rospy.loginfo("Spin passe")


# Programme principal qui tourne en boucle
if __name__ == '__main__':
    rospy.loginfo("Entre dans Main")
    ReceiveAttitude()
    rospy.loginfo("Listen passe")


 

        
 
       
