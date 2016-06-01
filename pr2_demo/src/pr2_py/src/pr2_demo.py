#!/usr/bin/env python

import sys
import copy
import rospy
import tf
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
#import geometry_msgs.PoseStamped


#from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import VisibilityConstraint, Constraints, OrientationConstraint, PositionConstraint

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+ " Pose x : %s", data.pose.position.x)
    #rospy.sleep(1)


def get_pose():
    raw_input("Press Enter to continue...")
    rospy.init_node('get_pose',anonymous=True)
    #rospy.Subscriber('/simtrack/ros_hydro_throttled', PoseStamped, callback, queue_size=1)
    rospy.spin()

def pick_n_place():
    ## Inititialise moveit_commander and rospy
    print "========Inititialise moveit_commander and rospy"
    #raw_input("Press Enter to continue...")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_n_place', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    left_arm = moveit_commander.MoveGroupCommander("left_arm");
    right_arm = moveit_commander.MoveGroupCommander("right_arm");

    right_arm.set_planner_id('RRTConnectkConfigDefault')

    right_arm.set_planning_time(10)

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    # Wait for RVIZ
    #rospy.sleep(10)

    print "============ Generating plan"
    raw_input("Press Enter to continue if Rviz visable...")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        try:
            # trans = tfBuffer.lookup_transform('odom_combined', 'ros_hydro',
            #                                   rospy.Time(0)) 
            # rospy.loginfo(trans)
            pose_target = geometry_msgs.msg.Pose()
            orient_target = tf.transformations.quaternion_from_euler(0,0,0)
            pose_target.orientation.x = orient_target[0]
            pose_target.orientation.y = orient_target[1]
            pose_target.orientation.z = orient_target[2]
            pose_target.orientation.w = orient_target[3]
            pose_target.position.x = 0.5
            pose_target.position.y = 0.2 #-0.6
            pose_target.position.z = 0.6

            constraints = Constraints()
            constraints.name = "Gripper Control"

            # Create an orientation constraint for the right gripper 
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = "r_wrist_roll_link"
            orientation_constraint.link_name = right_arm.get_end_effector_link()
            orientation_constraint.orientation = pose_target.orientation
            orientation_constraint.absolute_x_axis_tolerance = 10
            orientation_constraint.absolute_y_axis_tolerance = 10
            orientation_constraint.absolute_z_axis_tolerance = 10
            orientation_constraint.weight = 0.001

            # Append the constraint to the list of contraints
            constraints.orientation_constraints.append(orientation_constraint)

            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = "r_wrist_roll_link"
            position_constraint.link_name = right_arm.get_end_effector_link()
            position_constraint.target_point_offset = 0;
            position_constraint.constraint_region.primitive_poses = None 
            position_constraint.weight = 0.01

            rospy.loginfo(constraints)

            # Set the path constraints on the right_arm
            right_arm.set_path_constraints(constraints)


            # orient_target = geometry_msgs.msg.Pose()
            # quat = (
            #     trans.transform.rotation.x,
            #     trans.transform.rotation.y,
            #     trans.transform.rotation.z,
            #     trans.transform.rotation.w)
            # eular = tf.transformations.euler_from_quaternion(quat)
            # rospy.loginfo('Checkin quar now')
            # rospy.loginfo(quat)
            rospy.loginfo('Checkin Euler now')
            rospy.loginfo(orient_target)

            right_arm.set_start_state_to_current_state()
            right_arm.set_pose_target(pose_target)

            # right_arm.set_position_target([0.7,-0.6,1])
            # right_arm.set_rpy_target([0,-1,0])
            # right_arm.set_goal_orientation_tolerance(0.1)
            # right_arm.set_rpy_target(eular, 'r_wrist_roll_link')
            right_arm.allow_replanning(True)
            rospy.loginfo(pose_target)

            # right_arm.set_goal_tolerance(0.1)

            plan1 = right_arm.plan()
            rospy.sleep(2)

            # right_arm.go(wait=True)
            # rospy.sleep(2)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo('Error')
            # rospy.sleep(2)
            continue


    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)

    # Uncomment to execute on robot
    #raw_input("Executed on real robot..")
    #right_arm.go(wait=True)
    #rospy.sleep(10)

    raw_input("Motion Completed\nPress Enter to Shutdown..")




if __name__=='__main__':
    try:
        pick_n_place()
        #get_pose()
    except rospy.ROSInterruptException:
        pass

