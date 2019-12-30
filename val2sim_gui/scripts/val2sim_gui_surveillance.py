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
		# Define node name
		rospy.init_node('val2sim_gui_surveillance_node', anonymous=True)

	# Backend for start button
	def buttonStart_clicked(self): 
		# Robot moving in term of surveillance state machine process
		fsm_node = roslaunch.core.Node(	package='val2sim_fsm', 
										node_type='val2sim_fsm_surveillance.py', 
										name='val2sim_fsm_surveillance_node',
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
