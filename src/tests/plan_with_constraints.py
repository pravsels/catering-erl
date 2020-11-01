#!/usr/bin/env python
import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Twist
from tf.transformations import quaternion_from_euler
from math import pi as PI

class Tiago():

    def __init__(self, group_name='arm_torso'):

        self.rate = rospy.Rate(10)

        self.robot = RobotCommander()
        rospy.sleep(1)

        self.move_group = MoveGroupCommander(group_name)

        self.velocity = Twist()
        self.velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)


    def move(self, linear=(0,0,0), angular=(0,0,0)):
        self.velocity.linear.x = linear[0] 	# Forward or Backward with in m/sec.
        self.velocity.linear.y = linear[1]
        self.velocity.linear.z = linear[2]

        self.velocity.angular.x = angular[0]
        self.velocity.angular.y = angular[1]
        self.velocity.angular.z = angular[2] 	# Anti-clockwise/clockwise in radians per sec

        self.velocity_publisher.publish(self.velocity)


    def send_arm_goal(self, frame_id='base_footprint'):
        # SET EEF GOAL
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_footprint'
        goal_pose.pose.position.x = 0.55
        goal_pose.pose.position.y = -0.30
        goal_pose.pose.position.z = 0.76
        goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(PI/2, 0.0, 0.0))

        # SEND GOAL
        self.move_group.set_pose_reference_frame(frame_id)
        self.move_group.set_pose_target(goal_pose)

        # EXECUTE EEF GOAL
        # print(dir(self.move_group))
        self.move_group.go()


    # def add_gripper_constraint(self):
        # ADD GRIPPER CONSTRAINT
        # gripper_constraint = Constraints()
        # gripper_constraint.name = 'gripper constraint'
        #
        # gripper_orientation_constraint = OrientationConstraint()
        # gripper_orientation_constraint.header.frame_id = 'map'
        # gripper_orientation_constraint.link_name = 'arm_tool_link'
        # gripper_orientation_constraint.orientation = move_group.get_current_pose().pose.orientation
        # gripper_orientation_constraint.absolute_x_axis_tolerance = 0.50  # allow max rotation of 1 degree
        # gripper_orientation_constraint.absolute_y_axis_tolerance = 0.50
        # gripper_orientation_constraint.absolute_z_axis_tolerance = 0.50
        # gripper_orientation_constraint.weight = 0.1
        # gripper_constraint.orientation_constraints = [gripper_orientation_constraint]
        #
        # move_group.set_path_constraints(gripper_constraint)


if __name__ == '__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    tiago = Tiago()

    for i in range(85):
        tiago.move((0,0,0), (0,0,0.4))
        tiago.rate.sleep()

    # print(dir(robot))
    # cs = move_group.get_current_pose()
    # print('current pose : ', cs.pose.orientation)
