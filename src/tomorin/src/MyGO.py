#!/usr/bin/env python3
# coding=utf-8
import rospy
import RPi.GPIO as gpio
import time
import threading
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
# 初始化
init_x = 0.3 - 0.3
init_y = 0.3 - 0.3
# 目標
goal_x = 2.55
goal_y = -3.23
class MoveBaseWithEmergency:
    def __init__(self):
        self.btn = 17     # GPIO 17 = pin 11
        self.ledG = 12    # GPIO 12 = pin 32
        self.ledY = 26    # GPIO 26 = pin 37
        self.ledR = 16    # GPIO 16 = pin 36

        self.goal_sent = False
        self.goal_cancelled = False
        self.trying_forward = False

        # 初始化 GPIO
        gpio.setmode(gpio.BCM)
        gpio.setup(self.btn, gpio.IN)
        gpio.setup(self.ledG, gpio.OUT)
        gpio.setup(self.ledY, gpio.OUT)
        gpio.setup(self.ledR, gpio.OUT)

        gpio.output(self.ledG, gpio.HIGH)  # 系統啟動中
        gpio.output(self.ledY, gpio.LOW)
        gpio.output(self.ledR, gpio.LOW)

        # 發佈 initial pose
        self.init_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

        # 等待 move_base action server
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待 move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("已連接 move_base")
        self.publish_initial_pose()

        # 監聽 move_base 狀態
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.move_base_status_callback)

        # 啟動監控執行緒
        self.thread = threading.Thread(target=self.monitor_button)
        self.thread.start()

    def publish_initial_pose(self):
        theta_deg = 0
        theta_rad = math.radians(theta_deg)
        qz = math.sin(theta_rad / 2.0)
        qw = math.cos(theta_rad / 2.0)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = init_x
        pose_msg.pose.pose.position.y = init_y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = 0.0685

        self.init_pose_pub.publish(pose_msg)
        rospy.loginfo("初始位置已發佈：x=%.2f, y=%.2f, angle=%d°", init_x, init_y, theta_deg)

    def send_goal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        rospy.loginfo("導航目標已送出")
        self.goal_sent = True
        self.goal_cancelled = False
        self.trying_forward = False

    def try_forward(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"  # 相對座標：前進 0.3 公尺
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 0.3
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        rospy.logwarn("已送出手動復原前進指令")
        self.trying_forward = True
        self.send_goal

    def move_base_status_callback(self, msg):
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1].status
        # status code 4 表示 ABORTED
        if latest_status == 4 and self.goal_sent and not self.goal_cancelled and not self.trying_forward:
            rospy.logwarn("導航失敗，嘗試前進修復")
            self.try_forward()

    def monitor_button(self):
        prev_state = gpio.input(self.btn)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            current_state = gpio.input(self.btn)

            if not self.goal_sent:
                if current_state == gpio.HIGH:
                    gpio.output(self.ledY, gpio.HIGH)
                    gpio.output(self.ledR, gpio.LOW)
                    rospy.loginfo("按鈕放開，啟動導航")
                    self.send_goal()

            elif not self.goal_cancelled:
                if prev_state == gpio.HIGH and current_state == gpio.LOW:
                    rospy.logwarn("再次按下急停，取消導航")
                    self.client.cancel_all_goals()
                    self.goal_cancelled = True
                    gpio.output(self.ledY, gpio.LOW)
                    gpio.output(self.ledR, gpio.HIGH)

            prev_state = current_state
            rate.sleep()

    def cleanup(self):
        gpio.cleanup()


def main():
    rospy.init_node("movebase_emergency_button")
    node = MoveBaseWithEmergency()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()


if __name__ == "__main__":
    main()
