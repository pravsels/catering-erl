#!/usr/bin/env python
import rospy
from rospy import ROSException, ROSInterruptException, ServiceException

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from math import sin, cos, atan2, sqrt
from math import pi as PI
import random

class Util:
    def __init__(self):
        rospy.logwarn('Util class should not be instanced')
        pass

    @staticmethod
    def point_distance(p1, p2):
        # calculates the distance between two points
        if isinstance(p1, Point) and isinstance(p2, Point):
            x = (p1.x - p2.x)
            y = (p1.y - p2.y)
            z = (p1.z - p2.z)
            return sqrt(x*x + y*y + z*z)

        else:
            rospy.logfatal('Arguments ' + str(type(p1)) + ' ,' + str(type(p2)) + ' to point_distance are not points!')
            quit()

    @staticmethod
    def to_degrees(rad):
        # converts to degrees
        return rad / PI * 180.0

    @staticmethod
    def to_radians(deg):
        # converts to radians
        return deg / 180.0 * PI

    @staticmethod
    def quaternion_to_rad(quaternion):

        return euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]

    @staticmethod
    def quaternion_at_point(point, robot_pose=None):

        if not robot_pose:
            robot_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose

        if isinstance(point, Point):
            rad = atan2((point.y - robot_pose.position.y), (point.x - robot_pose.position.x))
            arr = quaternion_from_euler(0.0, 0.0, rad)

            return Quaternion(arr[0], arr[1], arr[2], arr[3])
        else:
            rospy.logfatal("quaternion_at_point expected 'Point' but got " + str(type(point)))

    @staticmethod
    def turn_towards_point(point, robot_pose=None):
        if not robot_pose:
            robot_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose

        if isinstance(point, Point):
            quat = Util.quaternion_at_point(point, robot_pose)
            robot_pose.orientation = quat

            return robot_pose
        else:
            rospy.logfatal("turn_towards_point expected 'Point' but got " + str(type(point)))

    @staticmethod
    def map_to_base(point, robot_pose=None):
        # convert point from map frame to base_footprint frame
        if not robot_pose:
            robot_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose

        angle = Util.quaternion_to_rad(robot_pose.orientation)
        sin_a = sin(angle)
        cos_a = cos(angle)

        if isinstance(point, Point):
            base_x = ((point.x - robot_pose.position.x) * cos_a) + ((point.y - robot_pose.position.y) * sin_a)
            base_y = ((point.y - robot_pose.position.y) * cos_a) - ((point.x - robot_pose.position.x) * sin_a)

            return Point(base_x, base_y, point.z)
        else:
            rospy.logfatal("map_to_base expected 'Point' but got " + str(type(point)))

    @staticmethod
    def base_to_map(point, robot_pose=None):
        # convert point from base_footprint frame to map frame
        if not robot_pose:
            robot_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped).pose.pose

        angle = Util.quaternion_to_rad(robot_pose.orientation)
        sin_a = sin(angle)
        cos_a = cos(angle)

        if isinstance(point, Point):
            map_x = (point.x * cos_a) - (point.y * sin_a) + robot_pose.position.x
            map_y = (point.x * sin_a) + (point.y * cos_a) + robot_pose.position.y

            return Point(map_x, map_y, point.z)
        else:
            rospy.logfatal("base_to_map expected 'Point' but got " + str(type(point)))


    @staticmethod
    def bb_iou(bounding_box, yolo_detected_objects, match_threshold=0.6):
        # find IOU between pcl segmented bounding box and yolo detected objects
        for yolo_object in yolo_detected_objects:
            # get the x,y coordinates of the intersection box
            x1 = max(bounding_box.x_offset, yolo_object.xywh[0])
            x2 = min(bounding_box.x_offset + bounding_box.width, yolo_object.xywh[0] + yolo_object.xywh[2])
            y1 = max(bounding_box.y_offset, yolo_object.xywh[1])
            y2 = min(bounding_box.y_offset + bounding_box.height, yolo_object.xywh[1] + yolo_object.xywh[3])

            # area of the intersection box
            area_of_intersection = max(0, x2 - x1 + 1) * max(0, y2 - y1 + 1)

            # area of the two bounding boxes
            box1_area = (bounding_box.width + 1) * (bounding_box.height + 1)
            box2_area = (yolo_object.xywh[2] + 1) * (yolo_object.xywh[3] + 1)

            iou = area_of_intersection / float(box1_area + box2_area - area_of_intersection)

            if iou >= match_threshold:
                return True, yolo_object.name
            else:
                return False, None

    @staticmethod
    def visualize_marker(marker_point, frame='base_footprint', scale=1.0, color='red', type='point'):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'basic_shapes'
        marker.id = random.randint(0, 100)

        marker.action = marker.ADD
        marker.lifetime.secs = 40

        if type == 'point':
            marker.pose.position = marker_point
            marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
            marker.type = marker.SPHERE
        else:
            # for pose
            marker.pose = marker_point
            marker.type = marker.CYLINDER

        marker.color.a = 1.0
        if (color == 'red'):
            marker.color.r = 1.0
        if (color == 'green'):
            marker.color.g = 1.0
        if (color == 'blue'):
            marker.color.b = 1.0

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        return marker

    @staticmethod
    def create_snf_requests(objects, queries, current_furniture, intent_confidence_thresh=0.55):
        # remove objects that we don't focus on
        requests = []
        for object in objects:
            for query in queries:
                entity = query['entities'][0]
                if object.name == entity and query['intent_confidence'] >= intent_confidence_thresh:
                    r = {'furniture': current_furniture, 'intent': query['intent'], 'object': object, 'status': 'found'}
                    requests.append(r)
                    queries.remove(query)

        return queries, requests

    @staticmethod
    def within_threshold(value_a, value_b, threshold=0.3):
        diff = abs(value_a - value_b)
        percent_diff = diff / value_a
        print('percent diff : ', percent_diff)

        return True if percent_diff <= threshold else False
