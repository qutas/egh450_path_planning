#!/usr/bin/env python

from math import *

import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from breadcrumb.srv import RequestPath
from breadcrumb.srv import RequestPathRequest

import actionlib
from actionlib_msgs.msg import GoalStatus

from contrail.msg import TrajectoryAction, TrajectoryGoal
from geometry_msgs.msg import Vector3

class PathPlanner():
	def __init__(self):
		self.start_x = float(rospy.get_param("~goal_start_x"))
		self.start_y = float(rospy.get_param("~goal_start_y"))
		self.end_x = float(rospy.get_param("~goal_end_x"))
		self.end_y = float(rospy.get_param("~goal_end_y"))
		self.height = float(rospy.get_param("~goal_height"))

		# Wait for the breadcrumb interface to start up
		# then prepare a Service Client
		rospy.loginfo("[NAV] Waiting to connect with Breadcrumb")
		rospy.wait_for_service('~request_path')
		self.srvc_bc = rospy.ServiceProxy('~request_path', RequestPath)

		rospy.loginfo("Waiting for contrail to connect...")
		self.client_base = actionlib.SimpleActionClient(rospy.get_param("~contrail"), TrajectoryAction)
		self.client_base.wait_for_server()

		# Needs to be connected to contrail
		self.pub_path = rospy.Publisher('~planned_path', Path, queue_size=10, latch=True)

		self.request_path()

	def shutdown(self):
		# Unregister anything that needs it here
		pass

	def path_display(self, path, start, end):
		msg_out = Path()
		msg_out.header = path.header

		#Insert the start pose
		ps = PoseStamped()
		ps.header = path.header
		ps.pose.position = start
		ps.pose.orientation.w = 1.0
		ps.pose.orientation.x = 0.0
		ps.pose.orientation.y = 0.0
		ps.pose.orientation.z = 0.0
		msg_out.poses.append(ps)

		# Instert the path recieved from breadcrumb
		for sp in path.poses:
			p = PoseStamped()
			p.header = path.header
			p.pose.position = sp.position
			p.pose.orientation.w = 1.0
			p.pose.orientation.x = 0.0
			p.pose.orientation.y = 0.0
			p.pose.orientation.z = 0.0
			msg_out.poses.append(p)

		#Insert the end pose
		pe = PoseStamped()
		pe.header = path.header
		pe.pose.position = end
		pe.pose.orientation.w = 1.0
		pe.pose.orientation.x = 0.0
		pe.pose.orientation.y = 0.0
		pe.pose.orientation.z = 0.0
		msg_out.poses.append(pe)

		self.pub_path.publish(msg_out)

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
			self.path_display(res.path, req.start, req.end)

			for i in xrange(len(res.path.poses)):
				# Build new goal message
				# https://github.com/qutas/contrail/blob/master/contrail/action/Trajectory.action
				goal_base = TrajectoryGoal()

				# Start point
				goal_base.positions.append(res.path.poses[i].position)
				goal_base.yaws.append(0.0)
				# End point
				goal_base.positions.append(res.path.poses[i+1].position)
				goal_base.yaws.append(0.0)

				goal_base.duration = rospy.Duration.from_sec(10)

				# Set a start time to be "start imidiately"
				goal_base.start = rospy.Time(0)

				# Transmit the goal to contrail
				self.client_base.send_goal(goal_base)
				self.client_base.wait_for_result()

		else:
			rospy.logerr("[NAV] No path received, abandoning planning")
			return;
