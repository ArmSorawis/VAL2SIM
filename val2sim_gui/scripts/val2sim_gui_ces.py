#!/usr/bin/env python

# Import necessary package 
import sys
import rospy
import roslaunch
from functools import partial
from std_msgs.msg import String
from PyQt5 import QtWidgets, uic, QtCore, QtGui
import os

# Initialize home directory
home = os.path.expanduser("~")

# Class for operating the ces show graphic user interface
class MainWindow(QtWidgets.QMainWindow):

	# Initial state
	def __init__(self):
		super(MainWindow,self).__init__()
		# Load ui
		uic.loadUi("{}/val2sim_ws/src/val2sim_gui/ui/ces.ui".format(home),self)
		self.setWindowTitle('VAL2')
		# Load val2 image
		pixmap_val2 = QtGui.QPixmap('{}/val2sim_ws/src/val2sim_gui/picture/val2sim.png'.format(home))
		self.label_val2_pic.setPixmap(pixmap_val2)
		# Define back end for each buttons
		self.startButton.clicked.connect(self.buttonStart_clicked)
		self.pauseButton.clicked.connect(self.buttonPause_clicked)
		self.continueButton.clicked.connect(self.buttonContinue_clicked)
		self.endButton.clicked.connect(self.buttonEnd_clicked)
		self.pauseButton.setEnabled(False)
		self.continueButton.setEnabled(False)
		self.endButton.setEnabled(False)
		# Define message which will be appear on gui 
		self.label_goal1.setText("Station 1")
		self.label_goal2.setText("Base Station")
		# Define node name
		rospy.init_node('val2sim_gui_ces_node', anonymous=True)
		# Initial publisher node 
		self.gui_command = rospy.Publisher('gui_cmd', String, queue_size = 10)  

	# Backend for start button
	def buttonStart_clicked(self):
		if self.lineEdit_rotate.text() != '' and self.lineEdit_translate.text() != '':
			print("start sent")
			self.label_processing.setText("Start")
			# Storage rotate angle and translate distance from front end
			rospy.set_param('~rotate_degree', int(self.lineEdit_rotate.text()))
			rospy.set_param('~translate_meter', int(self.lineEdit_translate.text()))
			self.startButton.setEnabled(False)
			self.startButton.setStyleSheet("background-color: gray")
			self.pauseButton.setEnabled(True)
			self.continueButton.setEnabled(False)
			self.endButton.setEnabled(True)
			self.pauseButton.setStyleSheet("background-color: None")
			self.continueButton.setStyleSheet("background-color: None")
			self.endButton.setStyleSheet("background-color: None")
			self.gui_command.publish("start")
		elif self.lineEdit_rotate.text() == '' or self.lineEdit_translate.text() == '':
			self.label_processing.setText("Empty")

	# Backend for pause button
	def buttonPause_clicked(self):
		print("pause sent")
		rospy.set_param('~silent_volume', 0.0)
		self.label_processing.setText("Pause")
		self.pauseButton.setEnabled(False)
		self.pauseButton.setStyleSheet("background-color: gray")
		self.continueButton.setEnabled(True)
		self.endButton.setEnabled(True)
		self.startButton.setStyleSheet("background-color: None")
		self.continueButton.setStyleSheet("background-color: None")
		self.endButton.setStyleSheet("background-color: None")
		self.gui_command.publish("pause")

	# Backend for continue button
	def buttonContinue_clicked(self):
		print("continue sent")
		self.label_processing.setText("Continue")
		self.continueButton.setEnabled(False)
		self.continueButton.setStyleSheet("background-color: gray")
		self.pauseButton.setEnabled(True)
		self.endButton.setEnabled(True)
		self.startButton.setStyleSheet("background-color: None")
		self.pauseButton.setStyleSheet("background-color: None")
		self.endButton.setStyleSheet("background-color: None")
		nodes = os.popen("rosnode list").read().splitlines()
		interest_node = '/ces_pause_subscriber_node'
		if interest_node in nodes:
			os.system("rosnode kill {}".format(interest_node))
		self.gui_command.publish("continue")
		rospy.set_param('~silent_volume', 1.0)
		
	# Backend for end button
	def buttonEnd_clicked(self):
		print("end sent")
		self.label_processing.setText("End")
		self.endButton.setEnabled(False)
		self.pauseButton.setEnabled(False)
		self.continueButton.setEnabled(False)
		self.endButton.setStyleSheet("background-color: gray")
		self.pauseButton.setStyleSheet("background-color: None")
		self.continueButton.setStyleSheet("background-color: None")

		## KILL SPECIFIC NODE
		# nodes = os.popen("rosnode list").read().splitlines()
		# interest_node_1 = '/sim_rotateBy_odom_node'
		# interest_node_2 = '/sim_translateBy_odom_node'
		# interest_node_3 = '/val2sim_soundplay_node'
		# interest_node_4 = '/val2sim_fsm_ces_node'
		# if interest_node_4 in nodes:
		# 	os.system("rosnode kill {}".format(interest_node_4))
		# if interest_node_1 in nodes:
		# 	os.system("rosnode kill {}".format(interest_node_1))
		# elif interest_node_2 in nodes:
		# 	os.system("rosnode kill {}".format(interest_node_2))
		# elif interest_node_3 in nodes:
		# 	os.system("rosnode kill {}".format(interest_node_3))
		# self.startButton.setEnabled(True)
		# self.startButton.setStyleSheet("background-color: None")
		# self.endButton.setStyleSheet("background-color: None")

		## KILL ALL NODE
		nodes = os.popen("rosnode list").readlines()
		for i in range(len(nodes)):
			nodes[i] = nodes[i].replace("\n","")
		for node in nodes:
			os.system("rosnode kill "+ node)


if __name__ == "__main__":
	app = QtWidgets.QApplication(sys.argv)
	Window = MainWindow()
	Window.show()
	sys.exit(app.exec_())
