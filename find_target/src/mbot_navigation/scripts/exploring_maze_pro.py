#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
import random
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from std_msgs.msg import Int8

STATUS_EXPLORING    = 0
STATUS_CLOSE_TARGET = 1
STATUS_GO_HOME      = 2

class ExploringMaze():
    def __init__(self):  
        rospy.init_node('exploring_maze_pro', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 0.5)  

        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5) 
 
        # 创建一个Subscriber，订阅/exploring_cmd
        rospy.Subscriber("/exploring_cmd", Int8, self.cmdCallback)

        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  
        
        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
 
        # 保存运行时间的变量   
        start_time = rospy.Time.now()  
        running_time = 0  

        rospy.loginfo("Starting exploring maze")  
        
        # 初始位置
        start_location = Pose(Point(0, 0, 0), Quaternion(0.000, 0.000, 0.709016873598, 0.705191515089))  
 
        locations = []  

        locations.append(Pose(Point(-0.322, 5.857, 0.000),  Quaternion(0.000, 0.000, 0.709016873598, 0.995191515089)))  
######################################请补充更多位置（开始）######################################################
	locations.append(Pose(Point(2.447, 4.191, 0.000),  Quaternion(0.000, 0.000, 0.0123661873459, 0.999923535782)))  
	locations.append(Pose(Point(7.411, -0.387, 0.000),  Quaternion(0.000, 0.000, -0.163554119075, 0.986534363382)))  
	locations.append(Pose(Point(4.588, 0.004, 0.000),  Quaternion(0.000, 0.000, -0.709519194356, 0.704686109442)))  
	locations.append(Pose(Point(6.421, 5.896, 0.000),  Quaternion(0.000, 0.000, -0.636628380104, 0.771170737026)))  
	locations.append(Pose(Point(6.875, 2.367, 0.000),  Quaternion(0.000, 0.000,  0.424508317953, 0.905424037669)))  


######################################请补充更多位置（结束）######################################################

        # 命令初值
        self.exploring_cmd = 0
        
        location = 0
       
        # 开始主循环，随机导航  
        while not rospy.is_shutdown():
######################################请补充主循环中的代码（开始）######################################################
			# 设定下一个随机目标点
			self.goal = MoveBaseGoal()
			self.goal.target_pose.pose = start_location	# 先回到初始点
			self.goal.target_pose.header.frame_id = 'map'
			self.goal.target_pose.header.stamp = rospy.Time.now()	# 把当前时间发布
			if self.exploring_cmd is STATUS_EXPLORING:
				self.goal.target_pose.pose = locations[location]
				location = location + 1
				if location is 6:
					location = 0
			elif self.exploring_cmd is STATUS_CLOSE_TARGET:	# 视觉伺服
				rospy.sleep(0.1)
			elif self.exploring_cmd is STATUS_GO_HOME:	# 返航
				self.goal.target_pose.pose.position.x = 0
				self.goal.target_pose.pose.position.y = 0
		
			# 让用户知道下一个位置
			rospy.loginfo("Going to:" + str(self.goal.target_pose.pose))

			# 向下一个位置进发
			self.move_base.send_goal(self.goal)
			
			# 五分钟时间限制
			finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

			# 查看是否成功到达
			if not finished_within_time:
				self.move_base.cancel_goal()
				rospy.loginfo("Timed out achieving goal")
			else:
				state = self.move_base.get_state()
				if state == GoalStatus.SUCCEEDED:
					rospy.loginfo("Goal succeeded!")
				else:
					rospy.loginfo("Goal failed!")
			# 运行所用时间
			running_time = rospy.Time.now() - start_time
			running_time = running_time.secs / 60.0

			# 输出本次导航的所有信息
			rospy.loginfo("current time:" + str(trunc(running_time,1))+"min")

######################################请补充主循环中的代码（结束）######################################################
        self.shutdown()

    def cmdCallback(self, msg):
        rospy.loginfo("Receive exploring cmd : %d", msg.data)
        self.exploring_cmd = msg.data

        if self.exploring_cmd is STATUS_CLOSE_TARGET:
            rospy.loginfo("Stopping the robot...")  
            self.move_base.cancel_goal() 

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)  

def trunc(f, n):  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try:  
        ExploringMaze()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring maze finished.")
