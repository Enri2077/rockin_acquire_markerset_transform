#!/usr/bin/env python
import time
from time import gmtime, strftime

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class MyNode():
	def __init__(self):
		rospy.init_node('acquirer', anonymous=True)
		
		# init tf and subscribers
		tf_listener = tf.TransformListener()
		tf_broadcaster = tf.TransformBroadcaster()
		
		# Robot position and yaw
		robot_pose = rospy.get_param('robot_pose')
		
		# Aquire the marker-robot transform
		transform_captured = False
		while not transform_captured:
			try:
				time.sleep(1.0)
				
				rospy.loginfo("acquiring transform from mocap")
				
				now = rospy.Time.now()
				
				# Robot pose
				(x_r0, y_r0, theta_r0) = (robot_pose['x'], robot_pose['y'], robot_pose['theta'])
				robot_rotation = tf.transformations.quaternion_from_euler(0, 0, theta_r0)
				
				# broadcast the robot frame relative to the marker
				tf_broadcaster.sendTransform((x_r0, y_r0, 0.0), robot_rotation, now, "/actual_robot", "/world")
				
				# receive the robot transform and compute the robot pose
				tf_listener.waitForTransform("/world", "/actual_robot", now, rospy.Duration(5.0))
				marker_to_robot_transform = tf_listener.lookupTransform("/robot_at_home", "/actual_robot", now)
				
				transform_captured = True
				
				rospy.loginfo("marker-robot transform aquired: ")
				rospy.loginfo(str(marker_to_robot_transform))
				
				rospy.set_param('marker_to_robot_transform', marker_to_robot_transform)			
				os.system("rosparam dump ~/" + rospy.get_namespace().strip("/") + "-" + strftime("%Y-%m-%d_%H:%M:%S", gmtime()) + ".yaml " + rospy.get_namespace())

		
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), Argument:
				rospy.loginfo("marker-robot transform not aquired") #TODO test
				rospy.loginfo(Argument)

if __name__ == '__main__':
	try:
		MyNode()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("ROSInterrupt")

