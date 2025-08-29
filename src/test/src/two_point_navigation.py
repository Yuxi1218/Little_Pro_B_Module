#!/user/bin/env python3
# coding=utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal(ac,x,y,z,ox,oy,oz,ow):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.x = ox
    goal.target_pose.pose.orientation.y = oy
    goal.target_pose.pose.orientation.z = oz
    goal.target_pose.pose.orientation.w = ow
    ac.send_goal(goal)
    if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("nav success")
    else:
        rospy.loginfo("nav lose")
        
if __name__ == "__main__":
    rospy.init_node("movebase_client")
    
    ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)
    
    rospy.loginfo("waiting for connect MoveBase actionServer...")
    ac.wait_for_server()
    rospy.loginfo("MoveBase action server is connect")
    
    #send first goal
    rospy.loginfo("send first goal point...")
    send_goal(ac,2.0,1.0,0,0,0,0,1)
    
    #send second goal
    rospy.loginfo("send second goal point...")
    send_goal(ac,3.35,2.63,0,0,0,0,1)
    
