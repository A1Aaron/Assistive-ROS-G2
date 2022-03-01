#!/usr/bin/env python

# Needed Packages
import rospy
from geometry_msgs.msg import PoseStamped
import time
import tf
import sys

def Position():

    pos = input("Enter the required posistion in the format 'x, y,theta(radians)'\n")
    pos = pos.split(',') # Splits the input into 3 varaibles 
    
    #Creating a PoseStamped message handler
    Pose = PoseStamped()
    
    
    # Assigning inputs to PoseStamped message type
    Pose = PoseStamped()
    Pose.header.seq = 1
    Pose.header.frame_id = "map"
    Pose.pose.position.x = float(pos[0]) 
    Pose.pose.position.y = float(pos[1])
    roll = 0
    pitch = 0
    yaw = float(pos[2])
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    Pose.pose.orientation.x = quaternion[0]
    Pose.pose.orientation.y = quaternion[1]
    Pose.pose.orientation.z = quaternion[2]
    Pose.pose.orientation.w = quaternion[3]

    #Creating a publisher object with topic '/move_base_simple/goal'
    pub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=10)
    # Initaing a node called User_Pos_Input
    rospy.init_node('User_Pos_Input',anonymous=False)
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
            for i in range (3):
                rospy.loginfo("Going to Posisition: (%s,%s, theta:%s " % (Pose.pose.position.x,Pose.pose.position.y,yaw)    )
                #rospy.loginfo("CHECKING\n: %s" % Pose )
                pub.publish(Pose)
                rate.sleep()
                
            sys.exit("DONE")
            

if __name__ == '__main__':
    try:
        Position()
    except rospy.ROSInterruptException:
        pass
    