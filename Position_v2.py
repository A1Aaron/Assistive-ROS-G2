#!/usr/bin/env python

# Needed Packages
import rospy
from move_base_msgs.msg import MoveBaseActionGoal
import time
import sys

def Position():

    pos = input("Enter the required posistion in the format 'x, y,'\n")
    pos = pos.split(',') # Splits the input into 6 varaibles 
    
    #Creating a Goal message handler
    Goal = MoveBaseActionGoal()
    
    # Assigning inputs to GoalTarget message type
    Goal.goal.target_pose.pose.position.x = float(pos[0]) 
    Goal.goal.target_pose.pose.position.y = float(pos[1])
    

    #Creating a publisher object with topic '/move_base_simple/goal'
    pub = rospy.Publisher('/move_base/goal',MoveBaseActionGoal, queue_size=10)
    # Initaing a node called User_Pos_Input
    rospy.init_node('User_Pos_Input', anonymous=False)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
            rospy.loginfo("Going to Position: (%s,%s)" % (Goal.goal.target_pose.pose.position.x ,Goal.goal.target_pose.pose.position.y )    )
            rospy.loginfo("CHECKING\n: %s" % MoveBaseActionGoal )
            pub.publish(Goal)
            rate.sleep()
            sys.exit("BYE BYE!!")

if __name__ == '__main__':
    try:
        Position()
    except rospy.ROSInterruptException:
        pass
    