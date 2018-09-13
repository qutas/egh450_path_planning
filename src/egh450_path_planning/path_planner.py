#!/usr/bin/env python

from math import *

import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from breadcrumb.srv import RequestPath
from breadcrumb.srv import RequestPathRequest

class PathPlanner():
	def __init__(self):
		self.start_x = float(rospy.get_param("~goal_start_x"))
		self.start_y = float(rospy.get_param("~goal_start_y"))
		self.end_x = float(rospy.get_param("~goal_end_x"))
		self.end_y = float(rospy.get_param("~goal_end_y"))
		self.height = float(rospy.get_param("~goal_height"))

		# Needs to be connected to contrail
		self.pub_path = rospy.Publisher('~path', Path, queue_size=10, latch=True)

		# Wait for the breadcrumb interface to start up
		# then prepare a Service Client
		rospy.loginfo("[NAV] Waiting to connect with Breadcrumb")
		rospy.wait_for_service('~request_path')
		self.srvc_bc = rospy.ServiceProxy('~request_path', RequestPath)

		self.request_path()

	def shutdown(self):
		# Unregister anything that needs it here
		pass

	def request_path(self):
		#Request a path from breadcrumb
		req = RequestPathRequest()

		req.start.x = self.start_x
		req.start.y = self.start_y
		req.start.z = self.height
		req.end.x = self.end_x
		req.end.y = self.end_y
		req.end.z = self.height

		res = self.srvc_bc(req)

		if len(res.path.poses) > 0:
			rospy.loginfo("[NAV] Path planned, preparing to transmit")

			msg_out = Path()
			msg_out.header = res.path.header

			#Insert the start pose
			ps = PoseStamped()
			ps.header = res.path.header
			ps.pose.position = req.start
			ps.pose.orientation.w = 1.0
			ps.pose.orientation.x = 0.0
			ps.pose.orientation.y = 0.0
			ps.pose.orientation.z = 0.0
			msg_out.poses.append(ps)

			# Instert the path recieved from breadcrumb
			for sp in res.path.poses:
				p = PoseStamped()
				p.header = res.path.header
				p.pose.position = sp.position
				p.pose.orientation.w = 1.0
				p.pose.orientation.x = 0.0
				p.pose.orientation.y = 0.0
				p.pose.orientation.z = 0.0
				msg_out.poses.append(p)

			#Insert the end pose
			pe = PoseStamped()
			pe.header = res.path.header
			pe.pose.position = req.end
			pe.pose.orientation.w = 1.0
			pe.pose.orientation.x = 0.0
			pe.pose.orientation.y = 0.0
			pe.pose.orientation.z = 0.0
			msg_out.poses.append(pe)

			self.pub_path.publish(msg_out)
		else:
			rospy.logerr("[NAV] No path received, abandoning planning")
			return;
