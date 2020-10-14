#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes, GripperTranslation
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3, Quaternion
from butler_erl.msg import PickNPlacePoseAction, PickNPlacePoseGoal, PickNPlacePoseResult
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
from math import pi as PI
from math import floor
import copy

from dynamic_reconfigure.server import Server
from butler_erl.cfg import GraspConfig

from tf.transformations import quaternion_from_euler
from utilities import Util

# inverting dict, going from name->code to code->name
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def create_pickup_goal(group='arm_torso', target='part',
    grasp_pose=PoseStamped(), possible_grasps=[], links_to_allow_contact=None):
    # create MoveIt pickup goal with the provided data
    pickup_goal = PickupGoal()
    pickup_goal.target_name = target
    pickup_goal.group_name = group
    pickup_goal.possible_grasps.extend(possible_grasps)
    pickup_goal.allowed_planning_time = 35.0
    pickup_goal.planning_options.planning_scene_diff.is_diff = True
    pickup_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    pickup_goal.planning_options.plan_only = False
    pickup_goal.planning_options.replan = True
    pickup_goal.planning_options.replan_attempts = 50
    pickup_goal.allowed_touch_objects = [target]                # name of the collision object we intend to pick
    pickup_goal.attached_object_touch_links = []
    pickup_goal.attached_object_touch_links.extend(links_to_allow_contact)

    return pickup_goal


def create_place_goal(place_pose, place_locations,
    group='arm_torso', target='part', links_to_allow_contact=None):
    # create MoveIt place goal with the provided data
    place_goal = PlaceGoal()
    place_goal.group_name = group
    place_goal.attached_object_name = target
    place_goal.place_locations = place_locations
    place_goal.allowed_planning_time = 15.0
    place_goal.planning_options.planning_scene_diff.is_diff = True
    place_goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    place_goal.planning_options.plan_only = False
    place_goal.planning_options.replan = True
    place_goal.planning_options.replan_attempts = 50
    place_goal.allowed_touch_objects = ['<octomap>']
    place_goal.allowed_touch_objects.extend(links_to_allow_contact)

    return place_goal



class PickNPlaceServer(object):
    def __init__(self):
        rospy.loginfo('initializing PickNPlaceServer')

        # loading config variables
        self.dynamic_reconfigure_srv = Server(GraspConfig, self.dynamic_reconfigure_cb)

        # grasp library

        rospy.loginfo('connecting to pickup action server')
        self.pickup_ac = SimpleActionClient('/pickup', PickupAction)
        self.pickup_ac.wait_for_server()
        rospy.loginfo('successfully connected to pickup action server!')

        rospy.loginfo('connecting to place action server')
        self.place_ac = SimpleActionClient('/place', PlaceAction)
        self.place_ac.wait_for_server()
        rospy.loginfo('successfully connected to place action server!')

        self.scene = PlanningSceneInterface()
        rospy.loginfo('connecting to /get_planning_scene service')
        self.scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo('successfully connected to get_planning_scene service!')

        rospy.loginfo('connecting to clear octomap service')
        self.clear_octomap_srv = rospy.ServiceProxy('/clear_octomap', Empty)
        self.clear_octomap_srv.wait_for_service()
        rospy.loginfo('successfully connected to clear octomap service!')

        self.links_to_allow_contact = rospy.get_param('/links_to_allow_contact')
        if not self.links_to_allow_contact:
            rospy.logwarn("Didn't find any links to allow contacts.. at param ~links_to_allow_contact")
        else:
            rospy.loginfo('Found links to allow contacts : ' + str(self.links_to_allow_contact))

        self.pick_as = SimpleActionServer('/pickup_pose', PickNPlacePoseAction,
        execute_cb=self.pick_cb, auto_start=False)
        self.pick_as.start()

        self.place_as = SimpleActionServer('/place_pose', PickNPlacePoseAction,
        execute_cb=self.place_cb, auto_start=False)
        self.place_as.start()

        self.grasp_marker_pub = rospy.Publisher('/grasp_marker', Marker, queue_size=10)


    def dynamic_reconfigure_cb(self, config, level):
        # load config values for grasping
        self._time_pre_grasp_posture = config["time_pre_grasp_posture"]
        self._time_grasp_posture = config["time_grasp_posture"]
        self._time_grasp_posture_final = config["time_grasp_posture_final"]

        self._max_contact_force = config["max_contact_force"]

        print(config)

        return config


    def pick_cb(self, goal):
        # check result from grasp_object and set it
        error_code = self.grasp_object(goal)
        pick_result = PickNPlacePoseResult()
        pick_result.error_code = error_code

        if error_code != 1:
            self.pick_as.set_aborted(pick_result)
        else:
            self.pick_as.set_succeeded(pick_result)


    def place_cb(self, goal):
        # check result from place_object and set it
        error_code = self.place_object(goal.object_pose)
        place_result = PickNPlacePoseResult()
        place_result.error_code = error_code

        if error_code != 1:
            self.place_as.set_aborted(place_result)
        else:
            self.place_as.set_succeeded(place_result)


    def wait_for_planning_scene_object(self, object_name='part'):
        rospy.loginfo('waiting for object ' + object_name + ' to appear in planning scene...')
        get_planning_scene_request = GetPlanningSceneRequest()
        get_planning_scene_request.components.components = get_planning_scene_request.components.WORLD_OBJECT_NAMES

        # loop until our object appears on scene
        part_in_scene = False
        while not rospy.is_shutdown() and not part_in_scene:
            get_planning_scene_response = self.scene_srv.call(get_planning_scene_request)

            for collision_object in get_planning_scene_response.scene.world.collision_objects:
                if collision_object.id == object_name:
                    part_in_scene = True
                    break
                else:
                    rospy.sleep(1.0)

        rospy.loginfo(object_name + ' is in the scene!')


    def open_and_close_gripper(self, grasp_width):
        # open or close grippers
        pre_grasp_posture = JointTrajectory()
        pre_grasp_posture.header.frame_id = 'arm_tool_link'
        pre_grasp_posture.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        jt_point1 = JointTrajectoryPoint()
        jt_point1.positions = [0.05, 0.05]
        jt_point1.time_from_start = rospy.Duration(self._time_pre_grasp_posture)
        pre_grasp_posture.points.append(jt_point1)

        grasp_posture = deepcopy(pre_grasp_posture)
        grasp_posture.points[0].time_from_start = rospy.Duration(self._time_pre_grasp_posture + self._time_grasp_posture)
        jt_point2 = JointTrajectoryPoint()
        jt_point2.positions = [grasp_width, grasp_width]
        jt_point2.time_from_start = rospy.Duration(self._time_pre_grasp_posture + self._time_grasp_posture
                                                                                + self._time_grasp_posture_final)
        grasp_posture.points.append(jt_point2)

        return {'pre_grasp_posture': pre_grasp_posture, 'grasp_posture': grasp_posture}


    def create_gripper_translation(self, direction_vector, min_distance, desired_distance, frame_id='base_footprint'):
        # used to generate pre-grasp approach and post-grasp retreat for grasps
        g_trans = GripperTranslation()
        g_trans.direction.header.frame_id = frame_id
        g_trans.direction.vector = direction_vector
        g_trans.min_distance = min_distance
        g_trans.desired_distance = desired_distance

        return g_trans


    def create_grasp(self, pose, approach, retreat, grasp_width, grasp_id=''):

        g = Grasp()
        g.id = grasp_id
        g.grasp_pose.header.frame_id = 'base_footprint'
        g.grasp_pose.pose = pose

        # set direction of approach and retreat pre and post grasping, respectively
        g.pre_grasp_approach = self.create_gripper_translation(approach, 0.095, 0.9)
        g.post_grasp_retreat = self.create_gripper_translation(retreat, 0.1, 0.2)

        print('grasp width : ', grasp_width)
        gripper_postures = self.open_and_close_gripper(grasp_width)
        g.pre_grasp_posture = gripper_postures['pre_grasp_posture'] # open gripper before grasp
        g.grasp_posture = gripper_postures['grasp_posture']         # close gripper during grasp

        g.max_contact_force = self._max_contact_force       # don't knock the object down whilst grasping

        return g


    def generate_grasps(self, pnp_goal):
        # decide between front or top grasp
        eef_gripper_dist = 0.11
        gripper_height = 0.11     # objects must be equal or above this for front-grasp
        max_graspable = 0.10      # max obj dimension that can fit between the grippers
        grasp_depth = 0.06        # max depth between gripper tip to gripper palm
        grasps = []

        if (pnp_goal.object_width < max_graspable and pnp_goal.object_height >= gripper_height):
            front_grasp_pose = Pose()
            front_grasp_pose.position = copy.deepcopy(pnp_goal.object_pose.pose.position)
            front_grasp_pose.orientation = Quaternion(*quaternion_from_euler(PI/2, 0.0, 0.0))       # horizontal gripper
            front_grasp_pose.position.x = front_grasp_pose.position.x - (eef_gripper_dist + (pnp_goal.object_depth/2))

            front_approach = Vector3(1.0, 0.0, 0.0)   # approach 'in' for front-grasp
            front_retreat = Vector3(0.0, 0.0, 1.0)    # retreat 'upwards' post-grasp

            front_grasp_width = pnp_goal.object_width/2
            front_grasp_width = floor(front_grasp_width*100)/100
            front_grasp = self.create_grasp(front_grasp_pose, front_approach, front_retreat, front_grasp_width, grasp_id='front_grasp')
            grasps.append(front_grasp)

        if (pnp_goal.object_width < max_graspable):
            top_grasp_width_pose = Pose()
            top_grasp_width_pose.position = copy.deepcopy(pnp_goal.object_pose.pose.position)
            top_grasp_width_pose.orientation = Quaternion(*quaternion_from_euler(PI/2, PI/2, 0.0))
            top_grasp_width_pose.position.z = top_grasp_width_pose.position.z + (grasp_depth + eef_gripper_dist + (pnp_goal.object_height/2))

            top_grasp_width_approach = Vector3(0.0, 0.0, -1.0)   # approach 'down' for top-grasp
            top_grasp_width_retreat = Vector3(0.0, 0.0, 1.0)    # retreat 'upwards' post-grasp

            top_grasp_width_width = pnp_goal.object_width/2
            top_grasp_width_width = floor(top_grasp_width_width*100)/100
            top_grasp_width = self.create_grasp(top_grasp_width_pose, top_grasp_width_approach, top_grasp_width_retreat, top_grasp_width_width, grasp_id='top_grasp_width')
            grasps.append(top_grasp_width)

        if (pnp_goal.object_depth < max_graspable):
            top_grasp_depth_pose = Pose()
            top_grasp_depth_pose.position = copy.deepcopy(pnp_goal.object_pose.pose.position)
            top_grasp_depth_pose.orientation = Quaternion(*quaternion_from_euler(0.0, PI/2, 0.0))
            top_grasp_depth_pose.position.z = top_grasp_depth_pose.position.z + (grasp_depth + eef_gripper_dist + (pnp_goal.object_height/2))

            top_grasp_depth_approach = Vector3(0.0, 0.0, -1.0)   # approach 'down' for top-grasp
            top_grasp_depth_retreat = Vector3(0.0, 0.0, 1.0)    # retreat 'upwards' post-grasp

            top_grasp_depth_width = pnp_goal.object_depth/2
            top_grasp_depth_width = floor(top_grasp_depth_width*100)/100
            top_grasp_depth = self.create_grasp(top_grasp_depth_pose, top_grasp_depth_approach, top_grasp_depth_retreat, top_grasp_depth_width, grasp_id='top_grasp_depth')
            grasps.append(top_grasp_depth)

        return grasps


    def clear_world_objects(self):
        rospy.loginfo("removing any previous 'part' object")
        # self.scene.remove_attached_object('arm_tool_link')
        self.scene.remove_world_object('part')


    def grasp_object(self, pnp_goal):
        self.clear_world_objects()
        rospy.loginfo("adding new 'part' object")
        self.scene.add_box('part', pnp_goal.object_pose, (pnp_goal.object_depth, pnp_goal.object_width, pnp_goal.object_height))

        # wait for objects part to appear
        self.wait_for_planning_scene_object('part')

        # compute grasp
        grasps = self.generate_grasps(pnp_goal)
        rospy.loginfo('computed grasps :: ')
        rospy.loginfo(grasps)
        error_code = -1

        for grasp in grasps:
            # publish grasp marker
            print('publishing grasp marker! ' + grasp.id)
            if grasp.id is not 'front_grasp':
                grasp_marker = Util.visualize_marker(grasp.grasp_pose.pose, scale=0.02, type='pose', color='blue')
            else:
                grasp_marker = Util.visualize_marker(grasp.grasp_pose.pose, scale=0.02, type='pose')

            self.grasp_marker_pub.publish(grasp_marker)

            goal = create_pickup_goal('arm_torso', 'part', pnp_goal.object_pose, [grasp], self.links_to_allow_contact)
            rospy.loginfo('sending pick goal')
            self.pickup_ac.send_goal(goal)
            rospy.loginfo('waiting for result')
            self.pickup_ac.wait_for_result()

            result = self.pickup_ac.get_result()
            error_code = result.error_code.val
            rospy.logdebug('using torso result: ' + str(result))
            rospy.loginfo('pick result: ' + str(moveit_error_dict[error_code]))

            if error_code == 1:
                break

        return error_code


    # def place_object(self, object_pose):
    #     rospy.loginfo('clearing octomap')
    #     self.clear_octomap_srv.call(EmptyRequest())
    #     # get possible placings
    #     possible_placings
    #     # try with only arm
    #     rospy.loginfo('trying to place using only arm')
    #     goal = create_place_goal(object_pose, possible_placings, 'arm', 'part', self.links_to_allow_contact)
    #     rospy.loginfo('sending place goal')
    #     self.place_ac.send_goal(goal)
    #     rospy.loginfo('waiting for result')
    #     self.place_ac.wait_for_result()
    #
    #     result = self.place_ac.get_result()
    #     rospy.loginfo('place result for arm only : ' + str(moveit_error_dict[result.error_code.val]))
    #
    #     if str(moveit_error_dict[result.error_code.val]) != 'SUCCESS':
    #         rospy.loginfo('trying to place with arm and torso')
    #         goal = create_place_goal(object_pose, possible_placings, 'arm_torso', 'part', self.links_to_allow_contact)
    #         rospy.loginfo('sending place goal for arm_torso')
    #         self.place_ac.send_goal(goal)
    #         rospy.loginfo('waiting for result')
    #         self.place_ac.wait_for_result()
    #
    #         result = self.place_ac.get_result()
    #         rospy.logerr(str(moveit_error_dict[result.error_code.val]))
    #
    #     # print result
    #     rospy.loginfo('result : ' + str(moveit_error_dict[result.error_code.val]))
    #     rospy.loginfo("removing previous 'part' object")
    #     self.scene.remove_world_object('part')
    #
    #     return result.error_code.val


if __name__ == '__main__':
    rospy.init_node('pick_n_place_server')
    pnp_server = PickNPlaceServer()
    pnp_server.clear_world_objects()
    rospy.spin()
