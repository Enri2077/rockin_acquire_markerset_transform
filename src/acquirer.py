#!/usr/bin/env python
import time, os
from time import gmtime, strftime

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

if __name__ == '__main__':
	rospy.init_node('acquirer', anonymous=True)
	
	# init tf and subscribers
	tf_listener = tf.TransformListener()
	tf_broadcaster = tf.TransformBroadcaster()
	
	# Robot position and yaw
#	robot_pose = rospy.get_param('robot_pose')
	
	# Acquire the marker-robot transform
	transform_captured = False
	while not transform_captured:
		try:
			time.sleep(1.0)
			
			print "acquiring transform from mocap"
			
		#	now = rospy.Time.now()
			
			# Robot pose
		#	(x_r0, y_r0, theta_r0) = (robot_pose['x'], robot_pose['y'], robot_pose['theta'])
		#	robot_rotation = tf.transformations.quaternion_from_euler(0, 0, theta_r0)
			
			# broadcast the robot frame relative to the marker
		#	tf_broadcaster.sendTransform((x_r0, y_r0, 0.0), robot_rotation, now, "/actual_robot", "/world")
			
			# receive the robot transform and compute the robot pose
		#	tf_listener.waitForTransform("/world", "/actual_robot", now, rospy.Duration(5.0))
			
			
			marker_to_robot_transform = tf_listener.lookupTransform("/robot_markerset", "/world", rospy.Time(0))
			
			robot_to_marker_transform = tf_listener.lookupTransform("/world", "/robot_markerset", rospy.Time(0))
			
			print "/robot_markerset to /world", marker_to_robot_transform
			
			print "/world to /robot_markerset", robot_to_marker_transform
			
			transform_captured = True
			
		#	( (mrt_x, mrt_y, _), mrt_rotation ) = marker_to_robot_transform
		#	(_, _, mrt_theta) = tf.transformations.euler_from_quaternion(mrt_rotation)
			
		#	mrt_pose = (mrt_x, mrt_y, mrt_theta)
			
		#	rospy.loginfo("marker-robot transform acquired")
		#	rospy.loginfo("transform: \t"+str(marker_to_robot_transform))
		#	rospy.loginfo("to 2D: \t"+str(mrt_pose))
			
			robot_info = raw_input("insert the name of the team (wothout spaces): ")
			
		#	filename = "~/logs/" + rospy.get_namespace().strip("/") + "-" + robot_info.replace(" ", "") + "-" + strftime("%Y-%m-%d_%H:%M:%S", gmtime()) + ".yaml"
			filename = "~/logs/transform-" + robot_info.replace(" ", "") + "-" + strftime("%Y-%m-%d_%H:%M:%S", gmtime()) + ".yaml"
			
			rospy.set_param('robot_info', robot_info)
			
		#	rospy.set_param('marker_to_robot_transform_2D', mrt_pose)
			rospy.set_param('robot_to_marker_transform', robot_to_marker_transform)
			rospy.set_param('marker_to_robot_transform', marker_to_robot_transform)
				
#			rospy.set_param('info', "marker_to_robot_transform is the tf transform from the markerset frame to the robot frame. The robot frame is computed from robot_pose")
			
			os.system("rosparam dump " + filename + " " + rospy.get_namespace())
			
			print "result dumped in " + filename
			
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), Argument:
			rospy.loginfo("marker-robot transform not acquired")
			rospy.loginfo(Argument)

