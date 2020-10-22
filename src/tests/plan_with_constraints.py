#!/usr/bin/env python
import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler
from math import pi as PI

if __name__ == '__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    robot = RobotCommander()
    rospy.sleep(1)

    # LOAD GROUP COMMANDER
    group_name = 'arm_torso'
    move_group = MoveGroupCommander(group_name)

    # SET EEF GOAL
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'base_footprint'
    goal_pose.pose.position.x = 0.40
    goal_pose.pose.position.y = 0.00
    goal_pose.pose.position.z = 0.76
    goal_pose.pose.orientation = Quaternion(*quaternion_from_euler(PI/2, 0.0, 0.0))

    # SEND GOAL 
    move_group.set_pose_reference_frame('base_footprint')
    move_group.set_pose_target(goal_pose)


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


    # EXECUTE EEF GOAL
    # print(dir(move_group))
    move_group.go()
    print('goal pose : ', goal_pose)

    # print(dir(robot))
    # cs = move_group.get_current_pose()
    # print('current pose : ', cs.pose.orientation)
