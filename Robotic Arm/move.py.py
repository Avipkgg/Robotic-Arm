#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('robo_arm_move', anonymous=True)
robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

cofee_1= [3.98,-2.15,0.56,-1.39,3.14]
cofee_2 =[3.9,-2.15,4.98,-2.15,3.98]


def go_to_joint_state(cofee):
    joint_goal = arm_group.get_current_joint_values()
    print(joint_goal)
    joint_goal[0] = cofee[0]
    joint_goal[1] = cofee[1]
    joint_goal[2] = cofee[2]
    joint_goal[3] = cofee[3]
    joint_goal[4] = cofee[4]

    arm_group.go(joint_goal, wait=True)
    arm_group.stop()

    current_joints = arm_group.get_current_joint_values()
    return (joint_goal, current_joints, 0.01)


def go_to_pose_goal():
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = 0.000
    pose_goal.orientation.y = 0.000
    pose_goal.orientation.z = 0.707
    pose_goal.orientation.w = 0.707

    pose_goal.position.x = 0.000
    pose_goal.position.y = 0.191
    pose_goal.position.z = 1.001
    arm_group.set_joint_value_target(pose_goal,True)

    plan = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    current_pose = arm_group.get_current_pose().pose
    return (pose_goal, current_pose, 0.01)

def arm_go_to_named_state(state):
    arm_group.set_named_target(state)
    plan = arm_group.go(wait=True)
    arm_group.stop()
    
def gripper_go_to_named_state(state):
	gripper_group.set_named_target(state)
	plan = gripper_group.go(wait=True)
	gripper_group.stop()
    
def main():
  try:
   
    go_to_joint_state(cofee_1)
    #go_to_pose_goal()
    #arm_go_to_named_state("initialize")
    gripper_go_to_named_state("open")
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
