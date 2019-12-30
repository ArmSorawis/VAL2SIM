#!/usr/bin/env python

# Import necessary package 
import sys
import rospy
import roslaunch
from functools import partial
from std_msgs.msg import String
from PyQt5 import QtWidgets, uic, QtCore, QtGui
from os.path import expanduser

# Initialize home directory
home = expanduser("~")

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
		# Define message which will be appear on gui 
		self.label_goal1.setText("Station 1")
		self.label_goal2.setText("Base Station")
		# Define node name
		rospy.init_node('val2sim_gui_ces_node', anonymous=True)

	# Backend for start button
	def buttonStart_clicked(self):
		# Storage rotate angle and translate distance from front end
		rospy.set_param('~rotate_degree', int(self.lineEdit_rotate.text()))
		rospy.set_param('~translate_meter', int(self.lineEdit_translate.text()))
		# Robot moving in term of ces state machine process
		fsm_node = roslaunch.core.Node(	package='val2sim_fsm', 
										node_type='val2sim_fsm_ces.py', 
										name='val2sim_fsm_ces_node',
										output='screen')
		fsm_launch = roslaunch.scriptapi.ROSLaunch()
		fsm_launch.start()
		fsm_process = fsm_launch.launch(fsm_node)
		while fsm_process.is_alive():
			if fsm_process.is_alive() == False:
				break


if __name__ == "__main__":
	app = QtWidgets.QApplication(sys.argv)
	Window = MainWindow()
	Window.show()
	sys.exit(app.exec_())
