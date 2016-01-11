#!/usr/bin/env python
# vim:set ts=4 sw=4 et:

 
import rospy
import thread
import threading
import time
import mavros
 
from math import *
from mavros.utils import *
#from mavros import setpoint as SP
from setpoint import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped,Vector3, Vector3Stamped, Point, Quaternion

 
 
class SetpointPosition:
    """
    Creation d'une classe SetPointPosition 
    On revoit les valeurs de x,y,z 
    This class sends position targets to FCU's position controller
    """

    def __init__(self):
     # Constructeur 
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
 
        # publisher for mavros/setpoint_position/local
        # Aller voir dans mavros/mavros/src/mavros/setpoint.py 
        self.pub = rospy.Publisher("/mavros/setpoint_position/pose", PoseStamped,queue_size=10)
        


        # subscriber for mavros/local_position/local
        # self.sub = rospy.Subscriber("/mavros/setpoint/local",PoseStamped, self.reached)
        self.sub = rospy.Subscriber("/mavros/local_position/pose",PoseStamped, self.reached)
        try:
            thread.start_new_thread(self.navigate, ())
        except:
            fault("Error: Unable to start thread")
 
        self.done = False
        self.done_evt = threading.Event()
 
    def navigate(self):
        rate = rospy.Rate(10)   # 10hz
 
        msg = PoseStamped(
            header=Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )
 
        while not rospy.is_shutdown():
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z
 
            # For demo purposes we will lock yaw/heading to north.
            yaw_degrees = 0  # North
            yaw = radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = Quaternion(*quaternion)
 
            self.pub.publish(msg)
            rate.sleep()
 
    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
 
        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()
                rospy.loginfo(self.done)
 
        time.sleep(delay)
 
    def reached(self, topic):
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) < 0.5
 
        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y) and \
           is_near('Z', topic.pose.position.z, self.z):
            self.done = True
            self.done_evt.set()
 
 
def setpoint():
    rospy.init_node('setpoint_position_demo')
    #mavros.set_namespace()  # initialize mavros module with default namespace
    rate = rospy.Rate(10)
    setpoint = SetpointPosition()
    while not rospy.is_shutdown():
        Sp = input("What's your setpoint ")
        rospy.loginfo("setpoint : " + str(Sp))
        setpoint.set(0.0, 0.0, Sp, 0)
 
    rospy.loginfo("Bye!")
 
 
if __name__ == '__main__':
    try:
        setpoint()
    except rospy.ROSInterruptException:
        pass