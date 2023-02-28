#!/usr/bin/env python
import rospy as ros
import numpy as np
import json
from os import environ
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from kinematics import *
from numpy import cos, sin, pi
from numpy.linalg import inv



class JointStatePublisher():
	def __init__(self):
		self.q_des = np.zeros(6)
		self.qd_des = np.zeros(6)
		self.tau_ffwd = np.zeros(6)
		self.filter_1 = np.zeros(6)
		self.filter_2 = np.zeros(6)

		self.q = np.zeros(6)
		self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
		                    'wrist_2_joint', 'wrist_3_joint']

	def set_des(self, q):
		self.q_des = q.reshape((1, 6)).tolist()[0]

	def send_des_jstate(self):
		msg = Float64MultiArray()
		# print(self.q_des)
		msg.data = self.q_des
		self.pub_des_jstate.publish(msg)

	def receive_jstate(self, msg):
		for msg_idx in range(len(msg.name)):
			for joint_idx in range(len(self.joint_names)):
				if self.joint_names[joint_idx] == msg.name[msg_idx]:
					self.q[joint_idx] = msg.position[msg_idx]

	def initFilter(self, q):
		self.filter_1 = np.copy(q)
		self.filter_2 = np.copy(q)

	def secondOrderFilter(self, input, rate, settling_time):
		dt = 1 / rate
		gain = dt / (0.1 * settling_time + dt)
		self.filter_1 = (1 - gain) * self.filter_1 + gain * input
		self.filter_2 = (1 - gain) * self.filter_2 + gain * self.filter_1
		return self.filter_2


def talker(p):
	ros.init_node('custom_joint_pub_node', anonymous=True)
	p.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=10)
	p.sub_jstate = ros.Subscriber("/ur5/joint_states", JointState, callback=p.receive_jstate, queue_size=1)

	loop_frequency = 100.
	loop_rate = ros.Rate(loop_frequency)  # 1000hz

	# init variables
	time = 0
	q_des0 = np.array(
		[-0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085,
		 -1.0017417112933558])
	p.initFilter(q_des0)

	while not ros.is_shutdown():
		roll = -pi
		pitch = cos(time*3)*0.4
		yaw = 0
		rotm = eul2rotm([roll, pitch, yaw])
		pos = np.array([0, 0.4, -0.8])
		
		jstate = inverse_kin(pos, rotm)

		p.set_des(jstate)
		p.qd_des = np.zeros(6)
		p.tau_ffwd = np.zeros(6)
		p.send_des_jstate()
		
		time = np.round(time + np.array([1 / loop_frequency]), 3)
		loop_rate.sleep()


if __name__ == '__main__':
	# environ["ROS_MASTER_URI"] = "http://10.218.202.205:11311"
	myPub = JointStatePublisher()

	try:
		talker(myPub)
	except ros.ROSInterruptException:
		pass