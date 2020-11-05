#! /usr/bin/python

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from numpy import pi, hypot,arctan2 
import os


class Main():
	def __init__(self):
		self.rPose = Pose()
		self.fPose = Pose()
		self.rabb_sub = rospy.Subscriber('/michelangelo/pose', Pose, self.setRabbit)
		self.runner_sub = rospy.Subscriber('/raphael/pose', Pose, self.setFox)
		self.runner_pub = rospy.Publisher('/raphael/cmd_vel', Twist, queue_size = 1)

			
	def setRabbit(self, pose):
		self.rPose = pose


	def setFox(self, pose):
		self.fPose = pose


	def run(self):
		while not rospy.is_shutdown(): 
			if self.distance() < 0.1 and self.distance() != 0:
				rospy.logerr('Pereehal cherepahu ta za sho')
				for node in nodes:
    					os.system(node)	
			else:
				angle_to_rabb = self.getAngleRabb()
				msg = Twist()
				msg.angular.z = angle_to_rabb
				msg.linear.x = self.get_speed(angle_to_rabb)
				self.runner_pub.publish(msg)


	def get_speed(self, angle_to_rabb):
		return (pi - abs(angle_to_rabb)) / pi

		
	def distance(self):
		return hypot(self.rPose.x - self.fPose.x, self.rPose.y - self.fPose.y)


	def getAngleRabb(self):
		location = self.fPose.theta
		rabb = arctan2(self.rPose.y - self.fPose.y, self.rPose.x - self.fPose.x)
		if location >= 0:
			if rabb >= location:
				return rabb - location
			else:
				if location - pi <= rabb:
					return rabb - location
				else:
					return rabb - location + 2 * pi
		else:
			if rabb <= location:
				return rabb - location
			else:
				if location + pi >= rabb:
					return rabb - location
				else:
					return rabb - location - 2 * pi
rospy.init_node('runner')
rospy.wait_for_service('/spawn')
spawn_func = rospy.ServiceProxy('/spawn', Spawn)
res = spawn_func(0.0, 5.0, 3.0, 'raphael')
m = Main()
m.run()
