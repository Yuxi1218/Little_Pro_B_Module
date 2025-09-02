#!/user/bin/env python3
# coding=utf-8
import RPi.GPIO as gpio
import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import threading

btn = 17   # GPIO 0 (Emergency)
ledG = 12   # GPIO 27 (G)
ledY = 26   # GPIO 26 (Y)
ledR = 16  # GPIO 25 (R)

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
    rospy.loginfo("goal sent: x=%s, y=%s, z=%s, ox=%s, oy=%s, oz=%s, ow=%s", x, y, z, ox, oy, oz, ow)
    # then, time me the goal status,and print until the goal is reached or canceled
    while not ac.wait_for_result(rospy.Duration(5)):
        rospy.loginfo("Waiting for goal to be reached...")
    if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Navigation succeeded")
    else:
        rospy.loginfo("Navigation failed with state: %s", ac.get_state())
def ems(client):
    try:
        while True:
            get_btn = gpio.input(btn)
            if get_btn == gpio.LOW:
                gpio.output(ledY, gpio.LOW)
                gpio.output(ledR, gpio.HIGH)
                print('STOP!')
                client.cancel_all_goals()
		#launch.shutdown()
                #rospy.signal_shutdown("Emergency stop")
                #rospy.loginfo("Emergency stop")
                #break

            else:
                gpio.output(ledY, gpio.HIGH)
                gpio.output(ledR, gpio.LOW)
                #print('GO!')
            time.sleep(0.2)
    except Exception as e:
        print("Error:", e)

    except:
        return

if __name__ == "__main__":

    rospy.init_node("movebase_client")

    try:
        gpio.setmode(gpio.BCM)
        gpio.setup(btn, gpio.IN)
        gpio.setup(ledG, gpio.OUT)
        gpio.setup(ledR, gpio.OUT)
        gpio.setup(ledY, gpio.OUT)
        gpio.output(ledG,gpio.HIGH)

        ac = actionlib.SimpleActionClient("move_base",MoveBaseAction)
        ac.wait_for_server()
        th = threading.Thread(target=ems, args=(ac,))
        while True:
            get_btn = gpio.input(btn)
            if get_btn == gpio.LOW:
                gpio.output(ledY, gpio.LOW)
                gpio.output(ledR, gpio.HIGH)
                print('STOP!')
            else:
                gpio.output(ledY, gpio.HIGH)
                gpio.output(ledR, gpio.LOW)
                #print('GO!')
                break
            time.sleep(0.1)



        th.start()

        rospy.loginfo("waiting for connect MoveBase actionServer...")

        rospy.loginfo("MoveBase action server is connect")

        #send first goal
        rospy.loginfo("send first goal point...")
        send_goal(ac,2.483,-3.353,0,0,0,0,1)
#        send_goal(ac,1.6,-3.32,0,0,0,0,1)
        th.join()

    except Exception as e:
        print("Error:", e)
    finally:
        gpio.cleanup()


