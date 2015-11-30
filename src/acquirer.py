#!/usr/bin/env python
import time, os
from time import gmtime, strftime

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

#####################################################################################
###    NOTE    																	  ###
#####################################################################################
#																					#
#	It is advisable to set the smoothing to 100 in Motive (as high as possible)		#
#	before acquiring the markerset-robot transform.									#
#	The appropriate smoothing for each configuration/markerset is also described	#
#	in the documentation "The RoCKIn Benchmarking System".							#
#																					#
#####################################################################################


if __name__ == '__main__':
	rospy.init_node('acquirer', anonymous=True)
	
	# init tf and subscribers
	tf_listener = tf.TransformListener()
	
	# Acquire the markerset-robot transform
	transform_captured = False
	while not transform_captured:
		try:
			time.sleep(1.0)
			
			print "acquiring transform from mocap"
			
			marker_to_robot_transform = tf_listener.lookupTransform("/robot_markerset", "/world", rospy.Time(0))
			
			robot_to_marker_transform = tf_listener.lookupTransform("/world", "/robot_markerset", rospy.Time(0))
			
			print "/robot_markerset to /world", marker_to_robot_transform
			
			print "/world to /robot_markerset", robot_to_marker_transform
			
			transform_captured = True
			
			robot_info = raw_input("insert the name of the team (wothout spaces): ")
			
			filename = "~/logs/transform-" + robot_info.replace(" ", "") + "-" + strftime("%Y-%m-%d_%H:%M:%S", gmtime()) + ".yaml"
			
			rospy.set_param('robot_info', robot_info)
			rospy.set_param('transform_timestamp', strftime("%Y-%m-%d_%H:%M:%S", gmtime()) )
			rospy.set_param('robot_to_marker_transform', robot_to_marker_transform)
			rospy.set_param('marker_to_robot_transform', marker_to_robot_transform)
			#TODO: change from robot_to_marker_transform to robot_to_markerset_transform (also in fbm2h, and everywhere else the yaml is used as input)
			
			os.system("rosparam dump " + filename + " " + rospy.get_namespace())
			
			print "result dumped in " + filename
			
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), Argument:
			rospy.loginfo("marker-robot transform not acquired")
			rospy.loginfo(Argument)

