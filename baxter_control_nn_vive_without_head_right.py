#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Feb  12 13:32:24 2018

@author: Sanket
"""
import rospy
import tf 
import baxter_interface
from baxter_pykdl import baxter_kinematics
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from multiprocessing import Process, Array
from std_msgs.msg import String
import time 
import sys
import os
import zipfile
import pickle
import numpy as np
#import tf.transformations
#from geometry_msgs.msg import Twist, TwistStamped
import re
import scipy.io
import h5py
import csv
import threading
from keras.models import model_from_json

#Moveit
#from moveit_commander import MoveGroupCommander


baxterData = 0
skeleton_joint_names = ['skeleton_head_1', 'skeleton_neck_1', 'skeleton_torso_1', 'skeleton_left_shoulder_1', 'skeleton_left_elbow_1', 'skeleton_left_hand_1', 'skeleton_right_shoulder_1', 'skeleton_right_elbow_1', 'skeleton_right_hand_1', 'skeleton_left_hip_1', 'skeleton_left_knee_1' ,'skeleton_left_foot_1', 'skeleton_right_hip_1', 'skeleton_right_knee_1', 'skeleton_right_foot_1']
trans = 0
rot = 0
SLEEP_TIME = 1.0
vive = []


def vive_callback(data):
	global vive
	vive = re.findall(r"[-+]?\d*\.*\d+", data.data)
	#print vive
    
def main():

	rospy.init_node('kinect_test')
	vive_data = []
	
	rate = rospy.Rate(30)
  
	rospy.Subscriber("unity_locations", String, vive_callback)
  	
	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')

	left_kin = baxter_kinematics('left')
        right_kin = baxter_kinematics('right')
  
  	print('Program Starting! 10 Seconds to move to initial pose')
	rospy.sleep(6.0)

	print('Pose Acquired, Demonstrator must hold this position for now!!!!')
	
	for i in vive:
		vive_data.append(float(i))
  
	human_data = np.array(vive_data)
	print "human_data"
	print human_data

	c1 = human_data[0:7]
	c2 = human_data[7:14]
        head = human_data[14:21]
	
	c1[0] = c1[0] - head[0]
	c1[1] = c1[1] - head[1]
        c1[2] = c1[2] - head[2]
        c2[0] = c2[0] - head[0]
        c2[1] = c2[1] - head[1]
        c2[2] = c2[2] - head[2]
	
	human_data = np.concatenate((c1, c2))
	human_data = np.reshape(human_data, (1,14))
	print human_data
	
	right_json_file = open('right_nn_without_head.json', 'r')
	right_loaded_model_json = right_json_file.read()
	right_json_file.close()
	right_joint_nn_model = model_from_json(right_loaded_model_json)
	# load weights into new model
	right_joint_nn_model.load_weights("right_nn_without_head_wt.h5")
	right_joint_nn_model.compile(loss = 'mean_squared_error', optimizer='sgd')
	right_predicted_joints = right_joint_nn_model.predict(human_data)[0]

	print right_predicted_joints
	left_neutral_position = {'left_s0':0.948767, 'left_s1':0.901981, 'left_e0':-1.456131, 'left_e1':0.541495, 'left_w0':0.887408, 'left_w1':0.488189, 'left_w2':-2.937190}
	right_neutral_position = {'right_s0':-1.039272, 'right_s1':0.868233, 'right_e0':1.078005, 'right_e1':0.537660, 'right_w0':-3.045335, 'right_w1':-0.272282, 'right_w2':-1.091811}
	left.move_to_joint_positions(left_neutral_position)
	print "left completed"
	right.move_to_joint_positions(right_neutral_position)
	right.move_to_joint_positions({'right_s0': right_predicted_joints[0],'right_s1': right_predicted_joints[1],'right_e0': right_predicted_joints[2],'right_e1': right_predicted_joints[3],'right_w0': right_predicted_joints[4],'right_w1':right_predicted_joints[5],'right_w2': right_predicted_joints[6]})
	#print 'Want Robot Ready to Move?[y/n]', 
	#userInput = sys.stdin.readline().strip()
	#if(userInput == 'y'):
	    #left.move_to_joint_positions(left_neutral_position)
	    #print "left completed"
	    #right.move_to_joint_positions(right_neutral_position)
	    #right.move_to_joint_positions({'right_s0': right_predicted_joints[0],'right_s1': right_predicted_joints[1],'right_e0': right_predicted_joints[2],'right_e1': right_predicted_joints[3],'right_w0': right_predicted_joints[4],'right_w1':right_predicted_joints[5],'right_w2': right_predicted_joints[6]})
	#else:
	    #print("Program exiting...")
    	    #return

	#print('Attempt real time tracking?[y/n]')
	#if(userInput != 'y'):
	#	print("Program exiting...")
	#	return

	pub_right = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)
	pub_left = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=10)
	print('Realtime Tracking Starting! Demonstrator can move very slowly now')

	while not rospy.is_shutdown():
	    vive_data = []
	    
	    for i in vive:
	    	vive_data.append(float(i))

  	    human_data = np.array(vive_data)
	    c1 = human_data[0:7]
	    c2 = human_data[7:14]
            head = human_data[14:21]

	    c1[0] = c1[0] - head[0]
	    c1[1] = c1[1] - head[1]
            c1[2] = c1[2] - head[2]
            c2[0] = c2[0] - head[0]
            c2[1] = c2[1] - head[1]
            c2[2] = c2[2] - head[2]
 	    
	    human_data = np.concatenate((c1, c2))
  	    human_data = np.reshape(human_data, (1,14))
	    print human_data
	   
	    right_predicted_joints = right_joint_nn_model.predict(human_data)[0]
  	    #right_end_effector = right_kin.forward_position_kinematics({'right_s0': right_predicted_joints[0],'right_s1': right_predicted_joints[1],'right_e0': right_predicted_joints[2],'right_e1': right_predicted_joints[3],'right_w0': right_predicted_joints[4],'right_w1':right_predicted_joints[5],'right_w2': right_predicted_joints[6]})
	    #print "end effector:"
	    #print right_end_effector
	    #with open("test_end_effector_vive.csv", 'a') as myfile:
	    #	wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
	    #	wr.writerow(np.concatenate((np.array(right_end_effector), human_data[0])))
            print "predicted joints:"
	    print right_predicted_joints
	    #right.move_to_joint_positions({'right_s0': right_predicted_joints[0],'right_s1': right_predicted_joints[1],'right_e0': right_predicted_joints[2],'right_e1': right_predicted_joints[3],'right_w0': right_predicted_joints[4],'right_w1':right_predicted_joints[5],'right_w2': right_predicted_joints[6]})
	    
	    command_msg = JointCommand()
	    command_msg.mode = JointCommand.POSITION_MODE
	    command_msg.command = [right_predicted_joints[i] for i in xrange(0, 6)]
	    command_msg.names = ['right_s0', 'right_s1','right_e0','right_e1','right_w0','right_w1', 'right_w2']
	    pub_right.publish(command_msg)

    	    rate.sleep()  
    	    #rospy.sleep(0.1) 


if __name__ == '__main__':
  main()
