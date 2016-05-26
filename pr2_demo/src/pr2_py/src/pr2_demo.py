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
from moveit_msgs.msg import VisibilityConstraint

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
            pose_target = geometry_msgs.msg.Pose()
            pose_target.orientation.w = trans.transform.rotation.w
            pose_target.orientation.x = trans.transform.rotation.x
            pose_target.orientation.y = trans.transform.rotation.y
            pose_target.orientation.z = trans.transform.rotation.z
            pose_target.position.x = trans.transform.translation.x
            pose_target.position.y = trans.transform.translation.y
            # pose_target.position.y = -0.4
            pose_target.position.z = trans.transform.translation.z


            orient_target = geometry_msgs.msg.Pose()
            quat = (
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w)
            # orient_target.w = trans.transform.rotation.w
            # orient_target.x = trans.transform.rotation.x
            # orient_target.y = trans.transform.rotation.y
            # orient_target.z = trans.transform.rotation.z
            eular = tf.transformations.euler_from_quaternion(quat)
            rospy.loginfo('Checkin quar now')
            rospy.loginfo(quat)
            rospy.loginfo('Checkin Euler now')
            rospy.loginfo(eular)
            # right_arm.set_orientation_target(orient_targetnux ,
            #                                  'r_gripper_r_finger_link')

            right_arm.set_pose_target(pose_target)

            # right_arm.set_position_target(pose_target.position,
            #                               'r_wrist_roll_link')
            # right_arm.set_rpy_target(eular, 'r_wrist_roll_link')
            right_arm.allow_replanning(True)
            # rospy.loginfo(pose_target)

            right_arm.set_goal_tolerance(0.1)

            plan1 = right_arm.plan()
            # rospy.sleep(2)

            right_arm.go(wait=True)
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

