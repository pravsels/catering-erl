#!/usr/bin/env python
import rospy
from exceptions import IndexError

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, Quaternion
from visualization_msgs.msg import Marker

from tf.transformations import quaternion_from_euler
from itertools import permutations
from util import Util

class OccupancyGridMap:
	def __init__(self):
		ogm_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.load_map, queue_size=1)

		self.marker_pub = rospy.Publisher('/nav_marker', Marker, queue_size=10)

	def load_map(self, ogm):

		self.ogm = ogm
		self.width = ogm.info.width
		self.height = ogm.info.height
		self.resolution = ogm.info.resolution
		self.free_thresh = 0.196
		self.occupied_thresh = 0.65

        # set the origin
		self.origin = Point(ogm.info.origin.position.x, ogm.info.origin.position.y, 0.0)

        # the ogm is loaded as a single-dimensional array
		self.data = ogm.data


	def are_indices_within_map(self, i, j):

		return 0 <= i < self.height and 0 <= j < self.width


	def get_by_index(self, i, j):

		if not self.are_indices_within_map(i, j):
			raise IndexError()

		return self.data[i*self.width + j]


	def get_occupancy_value_by_location(self, x, y):

		i, j = self.get_index_from_location(x, y)

		return self.get_by_index(i, j)


	def get_occupancy_value_by_index(self, x, y):

		return self.get_by_index(x, y)


	def get_index_from_location(self, x, y):

		i = int((y - self.origin.y) / self.resolution)
		j = int((x - self.origin.x) / self.resolution)

		return i, j


	def get_location_from_index(self, x_index, y_index):

		y = x_index*self.resolution + self.origin.y
		x = y_index*self.resolution + self.origin.x

		return x, y


	def approach_user(self, user_point):
		# generate points around user and choose one to approach
		sampled_points = []
		# the 8 directions
		directions = list(permutations([-1, 0, 1], 2))
		directions.append((1, 1))
		directions.append((-1,-1))

		for direction in directions:
			# unit distance in all 8 directions
			x_loc =  user_point.x + direction[0]
			y_loc =  user_point.y + direction[1]
			occupancy_value = self.get_occupancy_value_by_location(x_loc, y_loc)

			if occupancy_value >= 0 and occupancy_value <= self.free_thresh:
				# add point if it is not occupied
				point = Point(x_loc, y_loc, 0.0)
				sampled_points.append(
					{
						'point': (x_loc, y_loc),
						'dist_from_user': Util.point_distance(user_point, point)
					}
				)
				marker = Util.visualize_marker(point, frame='map', scale=0.2)
				self.marker_pub.publish(marker)

		# sort in ascending order and choose the nearest one
		sampled_points = sorted(sampled_points, key = lambda x: x['dist_from_user'])
		nearest_point = sampled_points[0]['point']
		# marker = Util.visualize_marker(Point(nearest_point[0], nearest_point[1], 0.0), frame='map', color='green', scale=0.2)
		# self.marker_pub.publish(marker)

		return Point(nearest_point[0], nearest_point[1], 0.0)


	def approach_furniture(self, furniture, step_size=0.2, offset=2):
		# find point to approach furniture
		bottom_left = furniture['bottom_left']
		bottom_right = furniture['bottom_right']
		top_right = furniture['top_right']

		center = Point((bottom_left.x + bottom_right.x)/2, bottom_left.y, 0.0)

		occupancy_value = self.get_occupancy_value_by_location(center.x, center.y)

		if top_right.y > bottom_left.y:
			step_size *= -1

		# TODO: fill up table as occupied
		while occupancy_value > self.free_thresh:
			center.y += step_size
			occupancy_value = self.get_occupancy_value_by_location(center.x, center.y)

		center.y += step_size*offset
		print('furniture approach point : ')
		print(center)
		# marker = Util.visualize_marker(center, frame='map', scale=0.2)
		# self.marker_pub.publish(marker)

		return center


	def approach_object(self, furniture, object, step_size=0.1, offset=2):
		# find point to approach object on furniture
		# bottom_left = furniture['bottom_left']
		# top_right = furniture['top_right']
		#
		# approach_point = Point(object.centroid.x, bottom_left.y, 0.0)
		#
		# occupancy_value = self.get_occupancy_value_by_location(approach_point.x, approach_point.y)
		#
		# if top_right.y > bottom_left.y:
		# 	step_size *= -1
		#
		# # TODO: fill up table as occupied
		# while occupancy_value > self.free_thresh:
		# 	approach_point.y += step_size
		# 	occupancy_value = self.get_occupancy_value_by_location(approach_point.x, approach_point.y)
		#
		# approach_point.y += step_size*offset
		# print('object approach point : ')
		# print(approach_point)
		# marker = Util.visualize_marker(approach_point, frame='map', scale=0.2)
		# self.marker_pub.publish(marker)
		#
		# return approach_point

		bottom_left = { 'name': 'bottom_left', 'point': furniture['bottom_left'] }
		bottom_right = { 'name': 'bottom_right', 'point': furniture['bottom_right'] }
		top_right = { 'name': 'top_right', 'point': furniture['top_right'] }
		top_left = { 'name': 'top_left', 'point': furniture['top_left'] }

		corners = [bottom_left, bottom_right, top_right, top_left]

		# store distance from object center
		for corner in corners:
			corner['distance'] = Util.point_distance(corner['point'], object.centroid)
		# sort corners by distance from object
		corners = sorted(corners, key=lambda x: x['distance'])
		approach_corner = corners[0] # choose closest corner
		print('corner closest to : ')
		print(approach_corner)

		# project centroid onto adjacent table sides of nearest corner
		approach_point1 = {'point': Point(approach_corner['point'].x, object.centroid.y, 0.0) }
		approach_point2 = { 'point': Point(object.centroid.x, approach_corner['point'].y, 0.0) }
		approach_points = [approach_point1, approach_point2]
		for ap in approach_points:
			ap['distance'] = Util.point_distance(ap['point'], object.centroid)
		# sort approach points by distance to object
		approach_points = sorted(approach_points, key=lambda x: x['distance'])
		approach_point = approach_points[0] # choose closest point
		marker = Util.visualize_marker(approach_point['point'], frame='map', scale=0.2)
		self.marker_pub.publish(marker)

		# the 4 directions N, S, E, W
		directions = list(permutations([-1, 0, 1], 2))
		directions.append((1, 1))
		directions.append((-1,-1))
		sampled_points = []

		for direction in directions:
			# unit distance in all 4 directions
			x_loc =  approach_point['point'].x + offset*direction[0]*step_size
			y_loc =  approach_point['point'].y + offset*direction[1]*step_size
			occupancy_value = self.get_occupancy_value_by_location(x_loc, y_loc)

			if occupancy_value >= 0 and occupancy_value <= self.free_thresh:
				# choose point if not occupied
				point = { 'point': Point(x_loc, y_loc, 0.0) }
				point['distance'] = Util.point_distance(approach_point['point'], point['point'])
				sampled_points.append(point)

				marker = Util.visualize_marker(point['point'], frame='map', scale=0.2)
				self.marker_pub.publish(marker)

		sampled_points = sorted(sampled_points, key=lambda x: x['distance'])

		return sampled_points[0]['point']
