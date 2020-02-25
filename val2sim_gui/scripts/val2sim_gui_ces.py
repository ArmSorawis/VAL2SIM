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
		# Define message which will be appear on gui 
		self.label_goal1.setText("Station 1")
		self.label_goal2.setText("Base Station")
		# Define node name
		rospy.init_node('val2sim_gui_ces_node', anonymous=True)
		# Initial publisher node 
		self.gui_command = rospy.Publisher('gui_cmd', String, queue_size = 10)  

	# Backend for start button
	def buttonStart_clicked(self):
		print("start sent")
		self.label_processing.setText("Start")
		# Storage rotate angle and translate distance from front end
		rospy.set_param('~rotate_degree', int(self.lineEdit_rotate.text()))
		rospy.set_param('~translate_meter', int(self.lineEdit_translate.text()))
		self.gui_command.publish("start")
		
	# Backend for pause button
	def buttonPause_clicked(self):
		print("pause sent")
		self.label_processing.setText("Pause")
		self.gui_command.publish("pause")

	# Backend for continue button
	def buttonContinue_clicked(self):
		print("continue sent")
		self.label_processing.setText("Continue")
		self.gui_command.publish("continue")
		nodes = os.popen("rosnode list").read().splitlines()
		interest_node = '/val2sim_fsm_ces_pause_node'
		if interest_node in nodes:
			os.system("rosnode kill {}".format(interest_node))

	# Backend for end button
	def buttonEnd_clicked(self):
		print("end sent")
		self.label_processing.setText("End")
		self.gui_command.publish("end")
		# nodes = os.popen("rosnode list").read().splitlines()
		# interest_node = '/val2sim_fsm_ces_node'
		# if interest_node in nodes:
		# 	os.system("rosnode kill {}".format(interest_node))

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
