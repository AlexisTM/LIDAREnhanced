#!/usr/bin/env python
# vim:set ts=4 sw=4 et:

 
import rospy
import mavros
from getch import *
from setpoint import *
#from geometry_msgs.msg import  PoseStamped





def SendPose():
    pub = rospy.Publisher('/mavros/mocap/pose',PoseStamped, queue_size=100)
    rospy.init_node('SendPose')
    #mavros.set_namespace()  # initialize mavros module with default namespace
    rate = rospy.Rate(10)
    rospy.loginfo("Init passe")

    msg = PoseStamped(  
        header=Header(  
            frame_id="base_footprint",
            stamp=rospy.Time.now()),
    )
    z= input("Quel est ta hauteur actuelle ? ")
    while not rospy.is_shutdown():      
    #for i in range(0,11) :
        
            what = getch()
            print(z)
            if what == "8" :
                z = float(z) +0.1
                print("8 ok")
                #rospy.loginfo("Hauteur  : " + str(z))
		
            elif what == "2" :
                z = float(z) -0.1
                print("2 ok")
                #rospy.loginfo("Hauteur  : " + str(z))
		
            elif what == "p" : 
                exit()
        
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = z
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            pub.publish(msg)
            rate.sleep()

          #  i = i+1
        
 
    # On ecrit sur le topic  
        
# Programme principal qui tourne en boucle
if __name__ == '__main__':
    rospy.loginfo("Entre dans Main")
   
    SendPose()
    rospy.loginfo("Listen passe")

       
