#!/usr/bin/env python
import rospy
import cv2 as cv
import time
from rospy import ServiceException, ROSException, ROSInterruptException
from butler_erl.msg import PickNPlacePoseAction, PickNPlacePoseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from actionlib import SimpleActionClient
from smach import State

# Srv
from lasr_pcl.srv import TransformCloudRequest, TransformCloud
from lasr_moveit.srv import ManipulationRequest, Manipulation
from std_srvs.srv import Empty, EmptyRequest

import numpy as np
from cv_bridge import CvBridge
from moveit_msgs.msg import MoveItErrorCodes
from utilities import Util

# inverting dict, going from name->code to code->name
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name


class Manipulate(State):
	def __init__(self, tiago, ogm, search):
		State.__init__(self, outcomes=['manipulate_done'])
		self.tiago = tiago
		self.ogm = ogm
		self.search = search

		self.manipulation_srv = rospy.ServiceProxy('manipulation', Manipulation)

		rospy.loginfo('connecting to clear octomap service')
		self.clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)
		self.clear_octomap_srv.wait_for_service()
		rospy.loginfo('successfully connected to clear octomap service!')
		# self.bridge = CvBridge()
		# self.tfBuffer = tf2_ros.Buffer()
		# self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)

		rospy.loginfo('connecting for pickup_pose action server')
		self.pick_as = SimpleActionClient('/pickup_pose', PickNPlacePoseAction)
		time.sleep(1.0)
		if not self.pick_as.wait_for_server(rospy.Duration(20)):
			rospy.logerr('Could not connect to pickup_pose action server!')
			exit()

		rospy.loginfo('connecting to place_pose action server')
		self.place_as = SimpleActionClient('/place_pose', PickNPlacePoseAction)
		self.place_as.wait_for_server()

		self.object_pose_pub = rospy.Publisher('/detected_object_pose', PoseStamped, queue_size=1, latch=True)


	def pick(self, object, action='pickup', planning_attempts=100):
		# send a pickup request with object pcl data to manipulation service
		try:
			rospy.loginfo('waiting for {}'.format('/manipulation'))
			rospy.wait_for_service('manipulation')
			rospy.loginfo('connected to {}'.format('/manipulation'))

			manipulation_request = ManipulationRequest(obj=object, command=action, planning_attempts=planning_attempts)
			manipulation_response = self.manipulation_srv(manipulation_request)
			rospy.loginfo('manipulation response is : ')
			rospy.loginfo(manipulation_request)
		except ServiceException as ex:
			print(ex)

		return manipulation_response.success


	def pick_n_place(self, object):
		# pick n place using action server
		pick_goal = PickNPlacePoseGoal()
		rospy.loginfo('Setting cube pose based on pcl detection')
		pick_goal.object_pose.pose.position = object.center.pose.position

		pick_goal.object_height = object.height
		pick_goal.object_width = object.width
		pick_goal.object_depth = object.depth
		rospy.loginfo('object pose in base_footprint: ' + str(pick_goal))

		pick_goal.object_pose.header.frame_id = 'base_footprint'

		# publish object pose for action server
		self.object_pose_pub.publish(pick_goal.object_pose)

		rospy.loginfo('Gonna pick: ' + str(pick_goal))
		self.pick_as.send_goal_and_wait(pick_goal)
		rospy.loginfo('Done picking object!')

		result = self.pick_as.get_result()
		print('moveit pick result : ', result)
		if result.error_code != 1:
			rospy.logerr('Failed to pick, not trying further')
			return False

		return True


	def approach_object(self, current_furniture, object):
		# calculate object approach point and navigate to it
		approach_object_point = self.ogm.approach_object(current_furniture, object)
		self.tiago.goto_location(Pose(approach_object_point, Util.quaternion_at_point(approach_object_point)))

		# orient to face object
		self.tiago.goto_location(Util.turn_towards_point(object.centroid))


	def dimensions_within_bounds(self, object, filtered_object, threshold=0.2):
        # check if object dimensions in new frame is within bounds of previously recorded dimensions in a diff frame
		print('checking for dimensions : ')
		if Util.within_threshold(object.width, filtered_object.width, threshold=threshold) and Util.within_threshold(object.height, filtered_object.height, threshold=threshold) and Util.within_threshold(object.depth, filtered_object.depth, threshold=threshold):
			print(object)
			print(filtered_object)
			return True

		return False


	def check_object(self, object):
		self.tiago.play('lower_head')
		pick_object_found = False

		while not pick_object_found:
			self.search.set_image_and_cloud(frame='base_footprint')
			yolo_detected_objects = self.search.yolo_detect_objects(self.search.image, confidence=0.5, visualize=True)
        	# if no yolo detections, then loop
			print('no of yolo : ', len(yolo_detected_objects))
			if not len(yolo_detected_objects):
				continue

			bounding_boxes, segmented_objects, whole_cloud = self.search.pcl_segment_objects(self.search.cloud, 0.5, 0.02, 200, 9500, visualize=True)
        	# filter pcl segmented object-clusters with yolo detected objects
			filtered_bounding_boxes, filtered_objects = self.search.filter_segmented_objects(yolo_detected_objects, bounding_boxes, segmented_objects, visualize=True)

			print('no of filtered : ', len(filtered_objects))
			# if no filtered objects, then loop
			if not len(filtered_objects):
				continue

			for filtered_object in filtered_objects:
				if self.dimensions_within_bounds(object, filtered_object):
					self.tiago.pick_object = filtered_object
					pick_object_found = True
					break


	def execute(self, userdata):
		fetch_reqs = filter(lambda q: q['intent'] == 'fetch', self.tiago.snf_requests)
		# sort to nearest fetch reqs first and pick last one
		fetch_reqs = sorted(fetch_reqs, key=lambda x: x['furniture']['distance_from_user'])

		for current_fetch_req in fetch_reqs:
			# move and orient to face object on furniture
			self.approach_object(current_fetch_req['furniture'], current_fetch_req['object'])

			# obtain object point cloud again, but in base_footprint
			self.check_object(current_fetch_req['object'])

			rospy.loginfo('clearing octomap')
			self.clear_octomap_srv.call(EmptyRequest())
			rospy.sleep(2.0)

			self.tiago.talk('I am looking around this furniture to build an octomap')
			# play motion to set up robot for pick/place
			self.tiago.play('head_look_around')
			# self.tiago.play('pregrasp')

			self.tiago.talk('I will now attempt to pick this object')
			# continue with other objects if current pick fails, else deposit
			if not self.pick_n_place(self.tiago.pick_object):
				current_fetch_req['status'] = 'pick_failed'
				self.tiago.play('home')
			else:
				current_fetch_req['status'] = 'pick_success'

		return 'manipulate_done'
