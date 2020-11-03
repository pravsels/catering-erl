#!/usr/bin/env python
import rospy
from smach import State

from geometry_msgs.msg import Point, Pose

from utilities import Util
from collections import namedtuple


class Explore(State):
    def __init__(self, tiago, ogm, search):
        State.__init__(self, outcomes=['manipulate_object', 'report_back'])

        self.tiago = tiago
        self.ogm = ogm
        self.search = search

        self.search_n_fetch_requests = []


    def get_centroid(self, filtered_object):
        center_x_base = filtered_object.center.pose.position.x
        center_y_base = filtered_object.center.pose.position.y

        return Point(center_x_base, center_y_base, 0.0)


    def centroid_within_bounds(self, centroid, objects, thresh=0.2):
        # check if new centroid is within bounds of previously noted ones
        for object in objects:
            if  Util.within_threshold(centroid.x, object.centroid.x) and Util.within_threshold(centroid.y, object.centroid.y):
                    return True

        return False


    def look_for_objects(self, current_furniture):
        self.search.set_image_and_cloud()
        yolo_detected_objects = self.search.yolo_detect_objects(self.search.image, confidence=0.5, visualize=True)
        # if no yolo detections, then exit
        if not len(yolo_detected_objects):
            return current_furniture

        bounding_boxes, segmented_objects, whole_cloud = self.search.pcl_segment_objects(self.search.cloud, 0.5, 0.02, 200, 9500, visualize=True)
        # filter pcl segmented object-clusters with yolo detected objects
        filtered_bounding_boxes, filtered_objects = self.search.filter_segmented_objects(yolo_detected_objects,
                                                                                bounding_boxes, segmented_objects, visualize=True)
        # if nothing left after filtering
        if not len(filtered_objects):
            return current_furniture

        # check if object already found, if not then add to found list
        if not len(current_furniture['objects']):
            # store initial set of objects
            for filtered_object in filtered_objects:
                centroid = self.get_centroid(filtered_object)

                object = namedtuple('item', ('name', 'centroid', 'center', 'height', 'width', 'depth'))(filtered_object.name, centroid, filtered_object.center, filtered_object.height, filtered_object.width, filtered_object.depth)
                current_furniture['objects'].append(object)
        else:
            # check if detected yolo objects are redundant or not
            for filtered_object in filtered_objects:
                centroid = self.get_centroid(filtered_object)

                if not self.centroid_within_bounds(centroid, current_furniture['objects']):
                    object = namedtuple('item', ('name', 'centroid', 'center', 'height', 'width', 'depth'))(filtered_object.name, centroid, filtered_object.center, filtered_object.height, filtered_object.width, filtered_object.depth)
                    current_furniture['objects'].append(object)

        return current_furniture


    def execute(self, userdata):
        # get search and fetch params
        search_n_fetch_queries = rospy.get_param('/search_n_fetch_queries')

        for index in range(0, len(self.tiago.furniture)):
            current_furniture = self.tiago.furniture[index]
            current_furniture['objects'] = []
            rospy.loginfo('Going to furniture %s' % current_furniture['name'])

            # initial approach to nearest free spot. orientation will be wrong
            approach_point = self.ogm.approach_furniture(current_furniture)
            self.tiago.goto_location(Pose(approach_point, Util.quaternion_at_point(approach_point)))

            # orient to face table center
            self.tiago.goto_location(Util.turn_towards_point(Point(approach_point.x, current_furniture['top_right'].y, 0.0)))

            self.tiago.talk('I am about to scan this furniture for objects.')

            self.tiago.play('lower_head')
            self.look_for_objects(current_furniture)

            center_y = (current_furniture['bottom_left'].y + current_furniture['top_right'].y)/2
            # turn head to cover left and right areas of the furniture
            current_pose = self.tiago.get_robot_pose()
            self.tiago.look_at(Util.map_to_base(Point(current_furniture['top_left'].x, center_y, 0.0), current_pose))
            self.look_for_objects(current_furniture)

            self.tiago.look_at(Util.map_to_base(Point(current_furniture['top_right'].x, center_y, 0.0), current_pose))
            self.look_for_objects(current_furniture)
            # reset head back to normal
            self.tiago.play('lower_head')

            if len(current_furniture['objects']):
                search_n_fetch_queries, requests = Util.create_snf_requests(current_furniture['objects'], search_n_fetch_queries, current_furniture)
                self.search_n_fetch_requests.extend(requests)

            # exit explore loop when queries are over
            if not len(search_n_fetch_queries):
                break

        rospy.set_param('/search_n_fetch_queries', search_n_fetch_queries)
        self.tiago.snf_requests = self.search_n_fetch_requests
        # transition to pick object or go back to user directly
        if len(filter(lambda q: q['intent'] == 'fetch', self.tiago.snf_requests)):    # if there are fetch queries
            return 'manipulate_object'
        else:
            return 'report_back'    # only search queries to report about
