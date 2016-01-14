#!/usr/bin/env python
# vim:set ts=4 sw=4 et:

 
import rospy
import mavros
import time
from getch import *
from threading import Thread
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros.utils import *

# From http://wiki.ros.org/mavros : 
# Published topic = Mavros publish it
# Subscribed topic = Mavros subscribe it
# local : We write on local to send
# pose  : Mavros write on it, comming from PixHawk

# 'local_position', 'pose' = Position drone que lui pense
# 'local_position', 'local' = Position que l'on envoie
# Envoie la hauteur actuelle
def sendSetpoint():
    global zSetPoint
    global setPointsCount
    setPointsCount = 0
    local_setpoint_pub   = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():

            msg = PoseStamped()
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = zSetPoint
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            local_setpoint_pub.publish(msg)
            rate.sleep()
            setPointsCount = setPointsCount + 1

def sendPosition():
    global zPosition
    global PositionsCount
    local_pos_pub   = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
            msg = PoseStamped()
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = float(zPosition)
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            local_pos_pub.publish(msg)
            rate.sleep()
            PositionsCount = PositionsCount + 1

def State_Callback(data):
    global state
    state = data

def Pose_Callback(data):
    global pose
    pose = data

def IMU_Callback(data):
    global imu
    imu = data

def InterfaceKeyboard():
    global zSetPoint
    global zPosition
    global pose
    global imu
    global disarm
    what = getch()
    if what == "z":
        zPosition = float(float(zPosition) + 0.1)
    if what == "s": 
        zPosition = float(float(zPosition) - 0.1)
    if what == "o":
        zSetPoint = zSetPoint + 1
    if what == "l":
        zSetPoint = zSetPoint - 1
    if what == "q":
        disarm = True
    
    rospy.loginfo("Positions sent : %i, Setpoints sent : %i",PositionsCount, setPointsCount )
    rospy.loginfo("Position is now %s", zPosition)
    rospy.loginfo("Position with int is %s", pose.pose.position.z)
    rospy.loginfo("Setpoint is now %s", zSetPoint)
    rospy.loginfo("IMU :")
    rospy.loginfo("roll : %s", imu.orientation.x)
    rospy.loginfo("pitch : %s", imu.orientation.y)
    rospy.loginfo("yaw : %s", imu.orientation.z)


def init(): 
    global state
    global disarm
    global zSetPoint
    global zPosition
    global setPointsCount
    global PositionsCount
    setPointsCount = 0
    PositionsCount = 0
    zSetPoint = 0
    zPosition = float(0)
    state = State()
    disarm = False

    rospy.init_node('position')

    rate = rospy.Rate(20.0)

    pose_sub        = rospy.Subscriber('mavros/local_position/pose', PoseStamped, Pose_Callback)
    state_sub       = rospy.Subscriber('mavros/imu/data', Imu, IMU_Callback)
    state_sub       = rospy.Subscriber('mavros/state', State, State_Callback)
    rospy.wait_for_service('mavros/cmd/arming')
    arming_client   = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    tSetPoints = Thread(target=sendSetpoint).start()
    tPositions = Thread(target=sendPosition).start()

    time.sleep(5)
    rospy.loginfo("We should have 100 setpoints sent now")


    while(state.mode != "OFFBOARD"):
        time.sleep(1)
        try : 
            rospy.loginfo("Trying to set to OFFBOARD mode")
            set_mode_client(custom_mode = "OFFBOARD")
        except rospy.ServiceException as ex:
            rospy.loginfo("OFFBOARD FAILED")
            

    rospy.loginfo("OFFBOARD SUCCESS")
    time.sleep(1)

    count = 0
    while(state.armed != True):
        time.sleep(10)
        if count == 10 : 
            exit()
        try : 
            rospy.loginfo("Trying to ARM")
            arming_client(True)
            count = count + 1
        except rospy.ServiceException as ex:
            rospy.loginfo("ARM FAILED")


    rospy.loginfo("ARM SUCCESS")
    while not rospy.is_shutdown(): 
        
        if disarm:
            arming_client(False)
            exit()

        InterfaceKeyboard()


if __name__ == '__main__':
    rospy.loginfo("We are ready")
    try:
        init()
    except rospy.ROSInterruptException:
        rospy.loginfo("init failed")
        pass