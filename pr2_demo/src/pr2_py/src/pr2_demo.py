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
# from scipy import linalg
from numpy import *
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from moveit_msgs.msg import (VisibilityConstraint, Constraints,
                             OrientationConstraint, PositionConstraint,
                             Grasp, GripperTranslation)

from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker

def qv_mult(q1, v1):
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2),
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

def pose_from_vector3D(waypoint):
    #http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
    pose= Pose()
    pose.position.x = waypoint[0]
    pose.position.y = waypoint[1]
    pose.position.z = waypoint[2]
    #calculating the half-way vector.
    u = [1,0,0]
    norm = linalg.norm(waypoint[3:])
    v = asarray(waypoint[3:])/norm
    if (array_equal(u, v)):
        pose.orientation.w = 1
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
    elif (array_equal(u, negative(v))):
        pose.orientation.w = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 1
    else:
        half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
        pose.orientation.w = dot(u, half)
        temp = cross(u, half)
        pose.orientation.x = temp[0]
        pose.orientation.y = temp[1]
        pose.orientation.z = temp[2]
    norm = math.sqrt(pose.orientation.x*pose.orientation.x +
                     pose.orientation.y*pose.orientation.y +
                     pose.orientation.z*pose.orientation.z +
                     pose.orientation.w*pose.orientation.w)
    if norm == 0:
        norm = 1
    pose.orientation.x /= norm
    pose.orientation.y /= norm
    pose.orientation.z /= norm
    pose.orientation.w /= norm
    return pose

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+ " Pose x : %s", data.pose.position.x)
    #rospy.sleep(1)


def get_pose():
    raw_input("Press Enter to continue...")
    rospy.init_node('get_pose',anonymous=True)
    #rospy.Subscriber('/simtrack/ros_hydro_throttled', PoseStamped, callback, queue_size=1)
    rospy.spin()


##############################################################
## https://github.com/ipa320/cob_object_manipulation/blob/master/object_manipulator/src/object_manipulator/draw_functions.py
def create_marker(type, dims, frame, ns = 'points', id = 0., duration = 1., color = [1,0,0], opaque = 0.5, pos = [0.,0.,0.], quat = [0.,0.,0.,1.]):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.type = type
    marker.action = Marker.ADD
    marker.scale.x = dims[0]
    marker.scale.y = dims[1]
    marker.scale.z = dims[2]
    marker.color.a = opaque
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.lifetime = rospy.Duration(duration)
    marker.id = id
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    return marker

###################################################################



def pick_n_place():
    ## Inititialise moveit_commander and rospy
    print "========Inititialise moveit_commander and rospy"
    #raw_input("Press Enter to continue...")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_n_place', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    right_arm = moveit_commander.MoveGroupCommander("right_arm");

    right_arm.set_planner_id('RRTConnectkConfigDefault')

    right_arm.set_planning_time(1)

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    print "============ Generating plan"
    raw_input("Press Enter to continue if Rviz visable...")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size = 1)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('odom_combined', 'ros_hydro',
                                              rospy.Time(0))

            pose_target = geometry_msgs.msg.PoseStamped()
            pose_target.header.frame_id = 'odom_combined'
            pose_target.header.stamp = rospy.Time.now()
            pose_target.pose.position.x = trans.transform.translation.x
            pose_target.pose.position.y = trans.transform.translation.y
            pose_target.pose.position.z = trans.transform.translation.z
            pose_target.pose.orientation.x = trans.transform.rotation.x
            pose_target.pose.orientation.y = trans.transform.rotation.y
            pose_target.pose.orientation.z = trans.transform.rotation.z
            pose_target.pose.orientation.w = trans.transform.rotation.w

            rospy.loginfo(pose_target)

            scene.add_box('ros_hydro', pose_target, (0.2,0.28,0.01) )
            box_marker = create_marker(Marker.ARROW,[.5,.04,.04],
                                       frame = 'odom_combined', ns = 'arrow',
                                      pos = (pose_target.pose.position.x + 0.1,
                                             pose_target.pose.position.y,
                                             pose_target.pose.position.z),
                                      color = (0,0.9, 1.0),
                                      quat = (pose_target.pose.orientation.x,
                                              pose_target.pose.orientation.y,
                                              pose_target.pose.orientation.z,
                                              pose_target.pose.orientation.w))
            box_marker.color.a = 1

            marker_pub.publish(box_marker)


            right_arm.set_goal_tolerance(0.1)
            right_arm.set_pose_target(pose_target)
            right_arm.plan()
            # rospy.sleep(2)
            # right_arm.go(wait=True)
            # rospy.sleep(2)

            # look_at = tfBuffer.lookup_transform(
            #     'r_wrist_roll_link',
            #     'ros_hydro', 
            #     rospy.Time(0))
            # look_at = trans
            # # orient_target = geometry_msgs.msg.Pose()
            # quat = (
            #     look_at.transform.rotation.x,
            #     look_at.transform.rotation.y,
            #     look_at.transform.rotation.z,
            #     look_at.transform.rotation.w)
            # eular = tf.transformations.euler_from_quaternion(quat)
            # rospy.loginfo(eular)
            #
            # pose_target = geometry_msgs.msg.Pose()
            # orient_target = tf.transformations.quaternion_from_euler(
            #     eular[0],
            #     eular[1],
            #     eular[2]
            #     )
            # pose_target.orientation.x = orient_target[0]
            # pose_target.orientation.y = orient_target[1]
            # pose_target.orientation.z = orient_target[2]
            # pose_target.orientation.w = orient_target[3]
            # pose_target.position.x = 0.5 # trans.transform.translation.x - 0.21
            # pose_target.position.y = 0.0 # trans.transform.translation.y
            # pose_target.position.z = 0.9 #trans.transform.translation.z
            #
            # lookat = (trans.transform.translation.x - 0.5,
            #           trans.transform.translation.y - 0.0,
            #           trans.transform.translation.z - 0.9)
            # rospy.loginfo("New directions to look at")
            # rospy.loginfo(lookat)
            #
            #
            # rot_vec = qv_mult(orient_target,(1,0,0)) # Rotate vector by quaternion
            # rospy.loginfo(rot_vec)
            # pose_target = pose_from_vector3D((0.5,0.0,0.9,
            #                                   lookat[0],lookat[1],lookat[2]))
            #
            #
            # constraints = Constraints()
            # constraints.name = "Gripper Control"
            #
            # # Create an orientation constraint for the right gripper
            # # orientation_constraint = OrientationConstraint()
            # # orientation_constraint.header.frame_id = "r_wrist_roll_link"
            # # orientation_constraint.link_name = right_arm.get_end_effector_link()
            # # orientation_constraint.orientation = pose_target.orientation
            # # orientation_constraint.absolute_x_axis_tolerance = 10
            # # orientation_constraint.absolute_y_axis_tolerance = 10
            # # orientation_constraint.absolute_z_axis_tolerance = 10
            # # orientation_constraint.weight = 0.1
            # #
            # # # Append the constraint to the list of contraints
            # # constraints.orientation_constraints.append(orientation_constraint)
            #
            #
            # # position_constraint = PositionConstraint()
            # # position_constraint.header.frame_id = "r_wrist_roll_link"
            # # position_constraint.link_name = right_arm.get_end_effector_link()
            # # position_constraint.weight = 0.001
            # # position_constraint.target_point_offset = Vector3(0.2,0,0)
            # # # position_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[10]))
            # # position_constraint.constraint_region.primitive_poses.append(pose_target)
            # # constraints.position_constraints.append(position_constraint)
            #
            # rospy.loginfo(constraints)
            #
            # # Set the path constraints on the right_arm
            # right_arm.set_path_constraints(constraints)

            # rospy.loginfo('Checkin Euler now')
            # rospy.loginfo(orient_target)

            # right_arm.set_start_state_to_current_state()
            # right_arm.set_pose_target(pose_target)
            # right_arm.allow_looking(True)

            # right_arm.set_position_target([0.7,-0.6,1])
            # right_arm.set_rpy_target([0,-1,0])
            # right_arm.set_goal_orientation_tolerance(0.1)
            # right_arm.set_rpy_target(eular, 'r_wrist_roll_link')
            # right_arm.allow_replanning(True)
            # rospy.loginfo(pose_target)

            # right_arm.set_goal_tolerance(0.1)
            #
            # plan1 = right_arm.plan()
            # # rospy.sleep(2)
            #
            # right_arm.go(wait=True)
            # # rospy.sleep(2)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.loginfo('Error')
            rospy.sleep(1)
            continue


    raw_input("Motion Completed\nPress Enter to Shutdown..")




if __name__=='__main__':
    try:
        pick_n_place()
        #get_pose()
    except rospy.ROSInterruptException:
        pass

