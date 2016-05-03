#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

def pick_n_place():
    ## Inititialise moveit_commander and rospy
    print "========Inititialise moveit_commander and rospy"
    raw_input("Press Enter to continue...")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_n_place', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    left_arm = moveit_commander.MoveGroupCommander("left_arm");
    right_arm = moveit_commander.MoveGroupCommander("right_arm");


    display_trajectory_publisher = rospy.Publisher(
        '/move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory)

    # Wait for RVIZ
    #rospy.sleep(10)

    print "============ Generating plan"
    raw_input("Press Enter to continue if Rviz visable...")
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.5
    pose_target.position.y = -0.05
    pose_target.position.z = 1.0
    left_arm.set_pose_target(pose_target)

    plan1 = left_arm.plan()

    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)

    # Uncomment to execute on robot
    raw_input("Executed on real robot..")
    left_arm.go(wait=True)
    rospy.sleep(10)

    raw_input("Motion Completed\nPress Enter to Shutdown..")




if __name__=='__main__':
    try:
        pick_n_place()
    except rospy.ROSInterruptException:
        pass

