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

# Class for operating the surveillance graphic user interface
class MainWindow(QtWidgets.QMainWindow):

	# Initial state
	def __init__(self):
		super(MainWindow,self).__init__()
		# Load ui
		uic.loadUi("{}/val2sim_ws/src/val2sim_gui/ui/surveillance.ui".format(home),self)
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
		# Define node name
		rospy.init_node('val2sim_gui_surveillance_node', anonymous=True)
		# Initial publisher node 
		self.gui_command = rospy.Publisher('gui_cmd', String, queue_size = 10)  

	# Backend for start button
	def buttonStart_clicked(self):
		print("start sent")
		self.label_processing.setText("Start")
		self.startButton.setEnabled(False)
		self.startButton.setStyleSheet("background-color: gray")
		self.pauseButton.setEnabled(True)
		self.continueButton.setEnabled(False)
		self.endButton.setEnabled(True)
		self.pauseButton.setStyleSheet("background-color: None")
		self.continueButton.setStyleSheet("background-color: None")
		self.endButton.setStyleSheet("background-color: None")
		self.gui_command.publish("start")

	# Backend for pause button
	def buttonPause_clicked(self):
		print("pause sent")
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
		interest_node = '/surveillance_pause_subscriber_node'
		if interest_node in nodes:
			os.system("rosnode kill {}".format(interest_node))
		self.gui_command.publish("continue")

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
		# interest_node_2 = '/val2sim_fsm_surveillance_node'
		# if interest_node_2 in nodes:
		# 	os.system("rosnode kill {}".format(interest_node_2))
		# if interest_node_1 in nodes:
		# 	os.system("rosnode kill {}".format(interest_node_1))
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
