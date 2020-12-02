#!/usr/bin/env python
import rospy
import actionlib
from rospy import ROSException, ROSInterruptException
import rosnode

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, PoseWithCovarianceStamped, Vector3
from nav_msgs.msg import OccupancyGrid

import speech_recognition as sr
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import pyttsx
from util import Util


class Tiago:
    def __init__(self):

        self.move_base_goal_sent = False	# if goal is sent, we need to cancel before shutting down
        self.play_motion_goal_sent = False

        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(10)

        self.robot_pose = None
        self.snf_requests = []

		# set linear and angular velocities
        self.velocity = Twist()
		# publish and subcribe to relevant topics
        self.velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The move_base action server is up")

        self.play_motion = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
        self.play_motion.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The play_motion action server is up")

        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        self.tts_client.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The tts action server is up")

        self.point_head = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        self.point_head.wait_for_server(rospy.Duration(5))
        rospy.loginfo("The point_head action server is up")

        self.load_items()


    def load_items(self):
        furniture_dict = rospy.get_param('/furniture')
        user_pos = rospy.get_param('/user_position')
        # load user position
        self.user_point = Point(**user_pos)

        # load furniture points
        self.furniture = [
            {
                'name':         f['name'],
                'bottom_left':  Point(**f['bottom_left']),
                'top_right':    Point(**f['top_right']),
                'bottom_right': Point(f['top_right']['x'], f['bottom_left']['y'], 0.0),
                'top_left':     Point(f['bottom_left']['x'], f['top_right']['y'], 0.0),
                'objects':      None,
                'distance_from_user': Util.point_distance(Point(**f['bottom_left']), self.user_point)
            }
            for f in furniture_dict
        ]
        # sort in ascending order
        self.furniture = sorted(self.furniture, key=lambda x: x['distance_from_user'])


    def move(self, linear=(0,0,0), angular=(0,0,0)):
        self.velocity.linear.x = linear[0] 	# Forward or Backward with in m/sec.
        self.velocity.linear.y = linear[1]
        self.velocity.linear.z = linear[2]

        self.velocity.angular.x = angular[0]
        self.velocity.angular.y = angular[1]
        self.velocity.angular.z = angular[2] 	# Anti-clockwise/clockwise in radians per sec

        self.velocity_publisher.publish(self.velocity)


    def goto_location(self, location):
        # use pose to send a move base goal
        self.move_base_goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = location

        self.move_base.send_goal_and_wait(goal)
        rospy.loginfo('Sent move_base goal and waiting for robot to carry it out...')


    def accept_speech(self, mode='audio'):
        # accept speech through audio or text
        return self.recognize_speech() if mode == 'audio' else self.speak_with_text()


    def speak_with_text(self):
        # set up the response object
        response = {
            "success": True,
            "error": None,
            "transcription": None
        }

        text = raw_input('Type out what you want to say : ')
        response['transcription'] = text

        return response


    def recognize_speech(self, duration=None):
        # transcribe speech from recorded from microphone
        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        # adjust the recognizer sensitivity to ambient noise and record audio
        # from the microphone
        with microphone as source:
            recognizer.adjust_for_ambient_noise(source)
            audio = recognizer.listen(source)

        # set up the response object
        response = {
            "success": True,
            "error": None,
            "transcription": None
        }

        try:
            response["transcription"] = recognizer.recognize_google(audio).encode('ascii', 'ignore')
        except sr.RequestError:
            # API was unreachable or unresponsive
            response["success"] = False
            response["error"] = "API unavailable"
        except sr.UnknownValueError:
            # speech was unintelligible
            response["success"] = False
            response["error"] = "Unable to recognize speech"

        return response


    def play(self, motion_name, skip_planning=True):
        # send prescribed play motion goals
        self.play_motion_goal_sent = True
        play_goal = PlayMotionGoal()
        play_goal.motion_name = motion_name
        play_goal.skip_planning = skip_planning

        self.play_motion.send_goal_and_wait(play_goal)
        rospy.loginfo('Sent play_motion goal and waiting for robot to carry it out... ')


    def look_at(self, target_point, frame='base_footprint'):
        # make tiago look at a point
        point_head_goal = PointHeadGoal()
        point_head_goal.target.header.frame_id = frame
        point_head_goal.max_velocity = 1
        point_head_goal.min_duration = rospy.Duration(2.0)
        point_head_goal.target.header.stamp = rospy.Time(0)
        point_head_goal.target.point = target_point
        point_head_goal.pointing_frame = 'head_2_link'
        point_head_goal.pointing_axis = Vector3(1, 0, 0)

        self.point_head.send_goal_and_wait(point_head_goal)
        rospy.loginfo('Sent point_head goal and waiting for robot to carry it out... ')


    def get_robot_pose(self):
        # return the robot's current pose
        try:
            if '/amcl' in rosnode.get_node_names():
                self.robot_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5).pose.pose
                return self.robot_pose
            else:
                rospy.logfatal('/amcl is inactive')
        except ROSInterruptException:
            pass
        except ROSException:
            raise


    def talk(self, speech_in):
        # Create the TTS goal and send it
        print('\033[1;36mTIAGO: ' + speech_in + '\033[0m')
        # set speech engine
        speech_engine = pyttsx.init()
        speech_engine.say(speech_in)
        speech_engine.runAndWait()

        tts_goal = TtsGoal()
        tts_goal.rawtext.lang_id = 'en_GB'
        tts_goal.rawtext.text = speech_in
        self.tts_client.send_goal(tts_goal)


    def shutdown(self):
        if self.move_base_goal_sent:
            self.move_base.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)

        if self.play_motion_goal_sent:
            self.play_motion.cancel_goal()
            rospy.loginfo("Stop Robot")
            rospy.sleep(1)
