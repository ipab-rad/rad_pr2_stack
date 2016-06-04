#!/usr/bin/env python

import sys
import copy
import rospy
import tf
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs

#from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from moveit_msgs.msg import VisibilityConstraint, Constraints, OrientationConstraint, PositionConstraint
from shape_msgs.msg import SolidPrimitive

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
            trans = tfBuffer.lookup_transform('odom_combined', 'ros_hydro',
                                              rospy.Time(0))
            rospy.loginfo(trans)

            look_at = tfBuffer.lookup_transform(
                'r_wrist_roll_link',
                'ros_hydro', 
                rospy.Time(0))
            look_at = trans
            # orient_target = geometry_msgs.msg.Pose()
            quat = (
                look_at.transform.rotation.x,
                look_at.transform.rotation.y,
                look_at.transform.rotation.z,
                look_at.transform.rotation.w)
            eular = tf.transformations.euler_from_quaternion(quat)
            rospy.loginfo(eular)

            pose_target = geometry_msgs.msg.Pose()
            orient_target = tf.transformations.quaternion_from_euler(
                eular[0],
                eular[1],
                eular[2]
                )
            pose_target.orientation.x = orient_target[0]
            pose_target.orientation.y = orient_target[1]
            pose_target.orientation.z = orient_target[2]
            pose_target.orientation.w = orient_target[3]
            pose_target.position.x = 0.5 # trans.transform.translation.x - 0.21
            pose_target.position.y = 0.0 # trans.transform.translation.y
            pose_target.position.z = 0.9 #trans.transform.translation.z


            constraints = Constraints()
            constraints.name = "Gripper Control"

            # Create an orientation constraint for the right gripper
            # orientation_constraint = OrientationConstraint()
            # orientation_constraint.header.frame_id = "r_wrist_roll_link"
            # orientation_constraint.link_name = right_arm.get_end_effector_link()
            # orientation_constraint.orientation = pose_target.orientation
            # orientation_constraint.absolute_x_axis_tolerance = 10
            # orientation_constraint.absolute_y_axis_tolerance = 10
            # orientation_constraint.absolute_z_axis_tolerance = 10
            # orientation_constraint.weight = 0.1
            #
            # # Append the constraint to the list of contraints
            # constraints.orientation_constraints.append(orientation_constraint)



            trans1 = tfBuffer.lookup_transform('odom_combined',
                                               'r_wrist_roll_link',
                                              rospy.Time(0))
            tar = geometry_msgs.msg.PoseStamped()
            tar_header = std_msgs.msg.Header()
            tar_header.stamp = rospy.Time.now()
            tar_header.seq = 0
            tar_header.frame_id = 'ros_hydro'
            tar.header = tar_header
            p1 = geometry_msgs.msg.Pose()
            p1.position.x = trans.transform.translation.x
            p1.position.y = trans.transform.translation.y
            p1.position.z = trans.transform.translation.z
            p1.orientation.x = trans.transform.rotation.x
            p1.orientation.y = trans.transform.rotation.y
            p1.orientation.z = trans.transform.rotation.z
            p1.orientation.w = 1
            tar.pose = p1

            tar1 = geometry_msgs.msg.PoseStamped()
            tar_header.stamp = rospy.Time.now()
            tar_header.seq = 0
            tar_header.frame_id = 'r_wrist_roll_link'
            tar1.header = tar_header
            p1.position.x = trans1.transform.translation.x
            p1.position.y = trans1.transform.translation.y
            p1.position.z = trans1.transform.translation.z
            p1.orientation.w = 1
            tar1.pose = p1

            vis_constraint = VisibilityConstraint()

            vis_constraint.target_radius = 0.1
            vis_constraint.target_pose = tar
            vis_constraint.sensor_pose = tar1
            vis_constraint.cone_sides = 4
            vis_constraint.max_view_angle = 1.7
            vis_constraint.weight = 1
            constraints.visibility_constraints.append(vis_constraint)

            # position_constraint = PositionConstraint()
            # position_constraint.header.frame_id = "r_wrist_roll_link"
            # position_constraint.link_name = right_arm.get_end_effector_link()
            # position_constraint.weight = 0.001
            # position_constraint.target_point_offset = Vector3(0.2,0,0)
            # # position_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[10]))
            # position_constraint.constraint_region.primitive_poses.append(pose_target)
            # constraints.position_constraints.append(position_constraint)

            rospy.loginfo(constraints)

            # Set the path constraints on the right_arm
            right_arm.set_path_constraints(constraints)

            rospy.loginfo('Checkin Euler now')
            # rospy.loginfo(orient_target)

            # right_arm.set_start_state_to_current_state()
            right_arm.set_pose_target(pose_target)
            # right_arm.allow_looking(True)

            # right_arm.set_position_target([0.7,-0.6,1])
            # right_arm.set_rpy_target([0,-1,0])
            # right_arm.set_goal_orientation_tolerance(0.1)
            # right_arm.set_rpy_target(eular, 'r_wrist_roll_link')
            # right_arm.allow_replanning(True)
            # rospy.loginfo(pose_target)

            right_arm.set_goal_tolerance(0.0001)

            plan1 = right_arm.plan()
            # rospy.sleep(2)

            right_arm.go(wait=True)
            rospy.sleep(2)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo('Error')
            rospy.sleep(1)
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

