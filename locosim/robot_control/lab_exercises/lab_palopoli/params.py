# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np


robot_params = {'ur5': {'dt': 0.001,
                        'kp': np.array([300, 300, 300, 30, 30, 1]),
                        'kd': np.array([20, 20, 20, 5, 5, 0.5]),
                        # 'q_0': np.array([-0.32, -0.2, -2.8, -1.63, 0, -1.0]),
                        'q_0': np.array([-0.32, -0.2, -2.56, -1.63, -1.57, -1.0]),
                        # 'q_0': np.array([2.4168, -2.5879, 0., 1.0171, 1.5709, -0.846]),
                        # limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                        'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                        'soft_gripper_joint_names': ['hand_1_joint', 'hand_2_joint'],
                        'gripper_joint_names': ['hand_1_joint', 'hand_2_joint', 'hand_3_joint'],
                        'ee_frame': 'tool0',
                        'control_mode': 'trajectory',  # 'trajectory','point'
                        'real_robot': False,
                        'control_type': 'position',  # 'position', 'torque'
                        'gripper_sim': True,  # False: the gripper is treated as a Rigid Body, True: you can move the finger joints
                        'soft_gripper': True,
                        'vision': False,
                        'vel_limit': 3.14,
                        'spawn_x': 0.5,
                        'spawn_y': 0.35,
                        'spawn_z': 1.8,
                        'world_name': 'tavolo_brick.world' #'empty.world'
                        }}

plotting = False


