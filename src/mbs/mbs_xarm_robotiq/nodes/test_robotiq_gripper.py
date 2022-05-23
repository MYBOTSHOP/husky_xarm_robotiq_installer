#! /usr/bin/env python3

import rospy
import actionlib
import actionlib_tutorials.msg
from mbs_xarm_robotiq.msg import GripperGraspAction, GripperGraspGoal, GripperGraspFeedback, GripperGraspResult
from mbs_xarm_robotiq.msg import GripperPositionAction, GripperPositionGoal, GripperPositionFeedback, GripperPositionResult

def gripper_grasp(state:str)->None:
    """Test cases for testing the robotiq gripper. For more information
    you may check the xARM lib in the src folder, alternatively the 
    robotiq documentation.

    Modifications for features can be made in the 
    robotiq_2f.py which is a ros action server.
    """     

    client = actionlib.SimpleActionClient('GripperGrasp', GripperGraspAction)
    client.wait_for_server()

    goal = GripperGraspGoal()

    if state == 'reset':
        goal.final_position = 0 # Reset first
        client.send_goal(goal)
        client.wait_for_result()

        goal.final_position = 1 # Open The gripper
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo('Completed Reset Action')

    if state == 'close':
        goal.final_position = 2 # Close The gripper
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo('Completed Close Action')

    if state == 'open':
        goal.final_position = 1 # Open The gripper
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo('Completed Open Action')


def gripper_position(position:int)->None:
    """Test cases for testing the robotiq gripper. For more information
    you may check the xARM lib in the src folder, alternatively the 
    robotiq documentation. 
    
    Modifications for features can be made in the 
    robotiq_2f.py which is a ros action server. 

    Args:
        position ([int]): 0-255, where 0 is the max open position and 255 is the closed 
                          position. The rosinfo displays the position in terms of cm 
                          which is 0cm - 0.7cm, mirrored.
    """  

    client = actionlib.SimpleActionClient('GripperPosition', GripperPositionAction)
    client.wait_for_server()

    goal = GripperPositionGoal()
    goal.final_position = position 
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo('Completed Position Action')

if __name__ == '__main__':
    try:
        rospy.init_node('gripper_grasp_py')

        gripper_grasp('reset')
        rospy.sleep(2)
        
        gripper_grasp('close')
        rospy.sleep(2)

        gripper_grasp('open')
        rospy.sleep(2)

        gripper_position(255)
        gripper_position(100)
        gripper_position(0)
        gripper_position(50)
        gripper_position(120)
        gripper_position(180)
        gripper_position(255)

    except rospy.ROSInterruptException:
        print("Program interrupted before completion")