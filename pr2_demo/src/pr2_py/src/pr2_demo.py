#!/usr/bin/env python

import sys
import copy
import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs

#from std_msgs.msg import String
# from scipy import linalg
from numpy import *
from geometry_msgs.msg import (PoseStamped, Pose, PoseArray,
                               Vector3, Quaternion, Point)
from moveit_msgs.msg import (VisibilityConstraint, Constraints,
                             OrientationConstraint, PositionConstraint,
                             Grasp, GripperTranslation)

from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray

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
def create_marker(type, dims, frame, ns = 'points', id = 0., duration = 9.,
                  color = [1,0,0], opaque = 0.7, p=Pose()):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time(0)
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
    marker.pose = p
    return marker

###################################################################

#TODO : Add all other object types and mesh support
def add_scene_object(name = 'ros_hydro', obj_type = 'box',
                     ref_frame = 'odom_combined', size = (0.2,0.28,0.03),
                     sc = 0):
    pose =  PoseStamped()
    pose.header.frame_id = ref_frame
    pose.header.stamp = rospy.Time(0)
    pose.pose.position = Vector3(0,0,0)
    pose.pose.orientation = Quaternion(0,0,0,1)

    rospy.loginfo(pose)

    if obj_type == 'sphere':
        return 0
    else:
        sc.add_box(name,pose,size)


def get_manip_pose(obj, pub, dim, grip_len = 0.21):
    marker_array = MarkerArray()
    poses = PoseArray()
    poses.header.frame_id = obj
    poses.header.stamp = rospy.Time(0)

    # grip_len = 0.21
    col = (0.3, 0.1, 0.0)

    # Pointing towards the origin
    Right = (-1.,0.,0.)
    Left = (1.,0.,0.)
    Top = (0.,-1.,0.)
    Bottom = (0.,1.,0.)
    Top_Right = (-1.,-1.,0.)
    Bottom_Left =(1.,1.,0.)
    Top_Left = (1.,-1.,0.)
    Bottom_Right =(-1.,1.,0.)
    Z_C_P = (0.,0.,-1.)
    Z_C_N = (0.,0.,1.)

    # TODO: Add option for when gripper does need to face object (i.e. grip_len = 0)
    C_R_Pos = ((dim[0]/2) + grip_len, 0, 0)
    C_L_Pos = (-((dim[0]/2) + grip_len), 0, 0)
    C_T_Pos = (0, (dim[1]/2) + grip_len, 0)
    C_B_Pos = (0, -((dim[1]/2) + grip_len), 0)
    C_Z_F_Pos = (0,0,((dim[2]/2) + grip_len))
    C_Z_B_Pos = (0,0,-((dim[2]/2) + grip_len))


    # rospy.loginfo(C_R_Pos + Right)
    # rospy.loginfo(C_L_Pos + Left)

    for i in range(1,3):
        if i == 1:
            pose_target = pose_from_vector3D((C_R_Pos + Right))
            # rospy.loginfo("Right-X")
        elif i == 2:
            pose_target = pose_from_vector3D((C_L_Pos + Left))
            # rospy.loginfo("Left-X")
        else:
            rospy.loginfo("Error in manip loop")

        # if i == 1:
        #     pose_target = pose_from_vector3D ((0.31,0,-0.07,-1,0,0))
        #     rospy.loginfo("Right-X")
        # elif i == 2:
        #     pose_target = pose_from_vector3D((-0.32,0,-0.07,1,0,0))
        #     rospy.loginfo("Left-X")
        # elif i == 3:
        #     pose_target = pose_from_vector3D ((0,0.36,-0.07,0,-1,0))
        #     rospy.loginfo("Right-Y")
        # elif i == 4:
        #     pose_target = pose_from_vector3D((0,-0.36,-0.07,0,1,0))
        #     rospy.loginfo("Left-Y")
        # elif i == 5:
        #     pose_target = pose_from_vector3D ((0.31,0.34,-0.07,-1,-1,0))
        #     rospy.loginfo("Top-Right-Diag")
        # elif i == 6:
        #     pose_target = pose_from_vector3D((-0.32,-0.34,-0.07,1,1,0))
        #     rospy.loginfo("Bottom-Left-Diag")
        # elif i == 7:
        #     pose_target = pose_from_vector3D ((-0.32,0.34,-0.07,1,-1,0))
        #     rospy.loginfo("Top-Left-Diag")
        # elif i == 8:
        #     pose_target = pose_from_vector3D((0.31,-0.34,-0.07,-1,1,0))
        #     rospy.loginfo("Bottom-Right-Diag")
        # elif i == 9:
        #     pose_target = pose_from_vector3D ((0,0,0.2,0,0,-1))
        #     rospy.loginfo("Z-Center-Pos")
        # elif i == 10:
        #     pose_target = pose_from_vector3D((0,0,-0.2,0,0,1))
        #     rospy.loginfo("Z-Center-Neg")

        if i % 2 == 0:
            col = [x + 0.2 for x in col]

        poses.poses.append(pose_target)

        marker_array.markers.append(create_marker(Marker.ARROW, [.2,.01,.01],
                                       frame = obj, ns = obj, id = i,
                                       p = pose_target,
                                       color = col, opaque = 1))

    pub.publish(marker_array)

    # rospy.loginfo(len(marker_array.markers))


    return poses





def pick_n_place():
    ## Inititialise moveit_commander and rospy
    print "========Inititialise moveit_commander and rospy"
    #raw_input("Press Enter to continue...")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_n_place', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    dual_arm = moveit_commander.MoveGroupCommander("arms");

    r_gripper = moveit_commander.MoveGroupCommander('right_gripper')
    l_gripper = moveit_commander.MoveGroupCommander('left_gripper')

    dual_arm.set_planner_id('RRTConnectkConfigDefault')

    dual_arm.set_planning_time(0.5)

    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    print "============ Generating plan"
    raw_input("Press Enter to continue if Rviz visable...")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size = 1)
    markerArray_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 1)

    # add_scene_object(sc = scene)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('odom_combined', 'ros_hydro',
                                              rospy.Time(0))

            pose_target = geometry_msgs.msg.PoseStamped()
            pose_target.header.frame_id = 'odom_combined'
            pose_target.header.stamp = rospy.Time(0)
            pose_target.pose.position.x = trans.transform.translation.x
            pose_target.pose.position.y = trans.transform.translation.y
            pose_target.pose.position.z = trans.transform.translation.z
            pose_target.pose.orientation.x = trans.transform.rotation.x
            pose_target.pose.orientation.y = trans.transform.rotation.y
            pose_target.pose.orientation.z = trans.transform.rotation.z
            pose_target.pose.orientation.w = trans.transform.rotation.w

            # rospy.loginfo(pose_target)

            # scene.add_box('ros_hydro', pose_target, (0.2,0.28,0.01) )
            p = PoseStamped()
            p.header.frame_id = 'odom_combined'
            p.pose.position = Point(0.13,1.9, 0.44)
            p.pose.orientation.w = 1.0
            scene.add_box('Table', p, (1.2,1.2,0.7))

            # rospy.loginfo("test print")
            pose_array = get_manip_pose('ros_hydro', markerArray_pub, (0.23, 0.28,
                                                                  0.03))

            p = PoseStamped()
            p.header = pose_array.header

            p.pose = pose_array.poses[0]
            p.pose.position.z -= 0.018
            p.pose.position.x -= 0.03
            r_goal = tf2_geometry_msgs.do_transform_pose(p,trans)


            p.pose = pose_array.poses[1]
            p.pose.position.z -= 0.025
            p.pose.position.x += 0.005
            l_goal = tf2_geometry_msgs.do_transform_pose(p,trans)

            dual_arm.set_goal_tolerance(0.01)
            dual_arm.allow_replanning(True)
            dual_arm.set_pose_target(r_goal, 'r_wrist_roll_link')
            dual_arm.set_pose_target(l_goal, 'l_wrist_roll_link')
            dual_arm.set_start_state_to_current_state()
            # dual_arm.plan()
            l_gripper.set_start_state_to_current_state()
            l_gripper.set_joint_value_target((0.03))
            l_gripper.plan()
            rospy.sleep(5)
            l_gripper.go(wait=True)
            # dual_arm.go(wait=True)
            # rospy.sleep(8)

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

