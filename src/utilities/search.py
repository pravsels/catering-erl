#!/usr/bin/env python
import rospy
import cv2 as cv
import message_filters
from rospy import ServiceException, ROSException, ROSInterruptException

# Srv
from lasr_object_detection_yolo.srv import YoloDetection, YoloDetectionRequest
from jeff_segment_objects.srv import SegmentObjects, SegmentObjectsRequest
from lasr_pcl.srv import TransformCloud, TransformCloudRequest

# Msg
from sensor_msgs.msg import Image, PointCloud2, RegionOfInterest
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose, Quaternion

from threading import Event, Lock
from cv_bridge import CvBridge, CvBridgeError

from util import Util
from collections import namedtuple


class Search:
    def __init__(self, dataset='coco'):

        self.yolo_dataset = dataset
        self.yolo_detection = rospy.ServiceProxy('/yolo_detection', YoloDetection)
        self.segment_objects_srv = rospy.ServiceProxy('/segment_objects', SegmentObjects)
        self.transform_cloud_srv = rospy.ServiceProxy('transform_cloud', TransformCloud)

        self.bridge = CvBridge()
        self.image = Image()
        self.cloud = PointCloud2()

        self.mutex = Lock()
        self.cb_event = Event()


    def transform_cloud(self, frame='base_footprint'):
        rospy.loginfo('waiting for {}'.format('/transform_cloud'))
        rospy.wait_for_service('transform_cloud')
        rospy.loginfo('connected to {}'.format('/transform_cloud'))

        transform_req = TransformCloudRequest(self.cloud, frame)
        transform_res = self.transform_cloud_srv(transform_req)
        self.cloud = transform_res.transformed_cloud
        rospy.loginfo('the cloud is now in ' + frame + ' frame!')


    def set_image_and_cloud(self, frame='map'):
        # fetches and syncs RGB image and cloud
        self.cb_event = Event()
        print('inside setting image and cloud')
        image_sub = message_filters.Subscriber('/xtion/rgb/image_rect_color', Image)
        cloud_sub = message_filters.Subscriber('/xtion/depth_registered/points', PointCloud2)

        sync = message_filters.ApproximateTimeSynchronizer([image_sub, cloud_sub], 1, 0.1)
        sync.registerCallback(self.single_message_callback)
        rospy.loginfo('registered callback finished..')

        # wait until event is set and lock freed
        while not rospy.is_shutdown() and not self.cb_event.is_set():
            rospy.loginfo('sleeping..')
            rospy.sleep(0.1)
        rospy.loginfo('after event..')

        image_sub.unregister()
        cloud_sub.unregister()

        # transform the cloud to be in base_footprint
        self.transform_cloud(frame=frame)


    def single_message_callback(self, image, cloud):
        # syncs RGB image with PointCloud
        self.mutex.acquire()
        try:
            if not self.cb_event.is_set() and (rospy.Time.now() - cloud.header.stamp) < rospy.Duration(0.5):
                rospy.loginfo('setting image and cloud..')
                self.image = image
                self.cloud = cloud
                rospy.loginfo('setting event..')
                self.cb_event.set()

                # convert to cv greyscale image
                try:
                    self.bgr_image = self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
                    self.rgb_image = cv.cvtColor(self.bgr_image, cv.COLOR_BGR2RGB)
                    self.gray = cv.cvtColor(self.rgb_image, cv.COLOR_RGB2GRAY)
                except CvBridgeError as ex:
                    rospy.logwarn(ex)
                    return
        finally:
            self.mutex.release()


    def yolo_detect_objects(self, image, confidence=0.5, nms=0.3, visualize=False):
        try:
            rospy.loginfo('waiting for {}'.format('/yolo_detection'))
            rospy.wait_for_service('/yolo_detection')
            rospy.loginfo('connected to {}'.format('/yolo_detection'))
            detection_req = YoloDetectionRequest(image_raw=image, dataset=self.yolo_dataset, confidence=confidence, nms=nms)
            detection_res = self.yolo_detection(detection_req)

            print('YOLO detection results : ', detection_res.detected_objects)

            if visualize:
                try:
                    frame = self.bridge.imgmsg_to_cv2(detection_res.image_bb, 'bgr8')
                except CvBridgeError as ex:
                    rospy.logwarn(ex)
                    return
                cv.imshow('Detected Objects YOLO', frame)
                cv.waitKey(5000)


            return detection_res.detected_objects
        except ROSInterruptException:
            pass
        except ServiceException as ex:
            rospy.logwarn('service call failed to {}'.format('/yolo_detection'))
            rospy.logwarn(ex)
        except ROSException as ex:
            rospy.logwarn('timed out waiting for {}'.format('/yolo_detection'))
            rospy.logwarn(ex)


    def visualize_bounding_boxes(self, bounding_boxes, window_title):
        frame = self.bgr_image.copy()
        for bb in bounding_boxes:
            cv.rectangle(frame, (bb.x_offset, bb.y_offset),
                (bb.x_offset + bb.width, bb.y_offset + bb.height), (255, 0, 255), 1)

        cv.imshow(window_title, frame)
        cv.waitKey(5000)


    def dimension_check(self, object, huge=0.4, tiny=0.015):
        # check if object dimensions are within reasonable range

        # check if object too big
        if object.height > huge or object.width > huge or object.depth > huge:
            return False

        # check if object too small
        if object.height < tiny or object.width < tiny or object.depth < tiny:
            return False

        return True


    def pcl_segment_objects(self, cloud, non_planar_ratio, cluster_tol, min_size, max_size, visualize=False):
        # segments objects in pcl by clustering
        # npr is about how much of the cloud to filter out
        # cluster_tol distance between each point to be considered in 1 cluster. 0.02 is 2cm
        # last two are min and max number of points in a cluster

        try:
            rospy.loginfo('waiting for {}'.format('/segment_objects'))
            rospy.wait_for_service('/segment_objects')
            rospy.loginfo('connected to {}'.format('/segment_objects'))
            segmentation_req = SegmentObjectsRequest(cloud, non_planar_ratio, cluster_tol, min_size, max_size)
            segmentation_res = self.segment_objects_srv(segmentation_req)

            width = cloud.width

            rospy.loginfo('No of clusters found from segment objects : ')
            rospy.loginfo(len(segmentation_res.clusters))

            bounding_boxes = []
            for cluster in segmentation_res.clusters:
                left = cluster.indices[0]%width
                right = cluster.indices[0]%width
                top = int(cluster.indices[0]/width)
                bottom = int(cluster.indices[0]/width)

                for index in cluster.indices:
                    row = int(index/width)
                    column = index%width

                    left = min(left, column)
                    right = max(right, column)
                    top = min(top, row)
                    bottom = max(bottom, row)

                bounding_box = RegionOfInterest()
                bounding_box.x_offset = left
                bounding_box.y_offset = top
                bounding_box.width = right - left
                bounding_box.height = bottom - top
                bounding_boxes.append(bounding_box)

            if visualize:
                self.visualize_bounding_boxes(bounding_boxes, 'PCL segmented objects')

            return bounding_boxes, segmentation_res.objects, segmentation_res.whole
        except ServiceException as ex:
            rospy.logwarn(ex)


    def filter_segmented_objects(self, yolo_detected_objects, bounding_boxes, objects, visualize=False):
        # filter out objects that fail dimension check
        n = len(bounding_boxes)
        index = 0
        while (index < n):
            if not self.dimension_check(objects[index]):
                bounding_boxes.pop(index)
                objects.pop(index)
                n = n - 1
            else:
                index += 1

        # check each pcl segmented object with the list of yolo detection object
        # with a bounding box IOU score
        object_tuples = []
        n = len(bounding_boxes)
        print('No of objects after dimension filter : ', n)
        index = 0
        while (index < n):
            high_score, matched_object_name = Util.bb_iou(bounding_boxes[index], yolo_detected_objects, match_threshold=0.7)
            if not high_score:
                bounding_boxes.pop(index)
                objects.pop(index)
                n = n - 1
            else:
                object_tuple = namedtuple('pcl_filtered_object', ('name', 'center', 'height', 'width', 'depth'))(matched_object_name, objects[index].center, objects[index].height, objects[index].width, objects[index].depth)
                object_tuples.append(object_tuple)
                index += 1

        if visualize:
            self.visualize_bounding_boxes(bounding_boxes, 'IOU filtered objects')

        rospy.loginfo('no of objects left after filtering : ')
        rospy.loginfo(len(bounding_boxes))
        return bounding_boxes, object_tuples
