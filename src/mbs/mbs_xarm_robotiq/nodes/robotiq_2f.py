#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
import actionlib
from sensor_msgs.msg import JointState

from mbs_xarm_robotiq.xarm.wrapper import XArmAPI
from mbs_xarm_robotiq.xarm.x3.code import APIState

from mbs_xarm_robotiq.msg import GripperGraspAction
from mbs_xarm_robotiq.msg import GripperPositionAction


class GripperNode(object):
    def __init__(self, ip: str = "192.168.132.1") -> None:
        """Starting and connecting to the xARM via their Python SDK.

        Args:
            ip (str): [description]. Defaults to "192.168.132.1".
        """
        self.ip = ip

        self.connect_xarm()
        self.activate_robotiq_gripper()

        self.open_gripper_server = actionlib.SimpleActionServer(
            'GripperGrasp', GripperGraspAction, execute_cb=self.cb_gripper_grasp, auto_start=False)
        self.position_gripper_server = actionlib.SimpleActionServer(
            'GripperPosition', GripperPositionAction, execute_cb=self.cb_gripper_position, auto_start=False)

        self.open_gripper_server.start()
        self.position_gripper_server.start()

    def connect_xarm(self) -> None:
        """Connecting xARM
        """
        self.arm = XArmAPI(self.ip)
        time.sleep(1)

    def disconnect_xarm(self) -> None:
        """Dis-connecting xARM
        """
        self.arm.disconnect()

    def activate_robotiq_gripper(self):
        """Initial activation of the the robotiq 2f-140 gripper.
        """
        code, ret = self.arm.robotiq_set_activate()
        self.error_check(code)

    def reset_robotiq_gripper(self):
        """Resets the robotiq 2f-140 gripper via xARM.
        """
        result = self.arm.robotiq_reset()

    def open_gripper(self) -> None:
        """Opens robotiq 2f-140 gripper via xARM.
        """
        if self.arm.connected:
            code, result = self.arm.robotiq_open()
            self.error_check(code)

    def close_gripper(self) -> None:
        """Closes robotiq 2f-140 gripper via xARM.
        """
        if self.arm.connected:
            code, result = self.arm.robotiq_close()
            self.error_check(code)

    def position_gripper(self, pos: int) -> None:
        """Gives a position to robotiq 2f-140 gripper via xARM.

        Args:
            pos (int): Takes position of 0-255 where 0 is the closed
                       position and 255 is the opened position.                       
        """
        if self.arm.connected:
            code, ret = self.arm.robotiq_set_position(pos)
            rospy.loginfo('Robotiq Gripper position {}'.format(
                round(self.current_position_gripper(), 2)))
            self.error_check(code)

    def error_check(self, code) -> None:
        """Checks whether error code has been given by xARM stopping the gripper.

        Args:
            code (int): Error code.
        """
        if code == APIState.END_EFFECTOR_HAS_FAULT:
            rospy.logerr('robotiq fault code: {}'.format(
                self.arm.robotiq_status['gFLT']))

    def current_position_gripper(self) -> float:
        """Informs about the current position to robotiq 2f-140 gripper via xARM.

        Returns:
            float: The mapped position from 0-255 to 0-0.7 where 0 is closed position
                   of the gripper, 0.7 is the opened position. 0.7 is in cm and is the
                   distance of a single finger to the center point of the gripper.
        """
        result = self.arm.robotiq_get_status()[1]
        position = result[6]
        mapped_position = self.range_mapper(position, 0, 255, 0, 0.7)
        return mapped_position

    def publish_joint_states(self):
        """Generates joint publishers for visualization in RVIZ.
        """
        joint_state_publisher = rospy.Publisher(
            'joint_states', JointState, queue_size=10)
        joint_state = JointState()
        rate = rospy.Rate(10)
        joint_state.name = ['left_finger_joint', 'right_finger_joint']
        while not rospy.is_shutdown():
            joint_state.header.stamp = rospy.Time.now()

            pos = self.current_position_gripper()
            robotiq_status = [pos, pos]

            joint_state.position = robotiq_status
            joint_state.velocity = [0]*2
            joint_state.effort = [0]*2
            joint_state_publisher.publish(joint_state)
            rate.sleep()

    def cb_gripper_grasp(self, goal):
        """Callback for the GripperGrasp action.

        Args:
            goal (int): 0 resets the gripper.
                        1 opens the gripper fully.
                        2 closes the gripper fully.
        """
        try:
            if goal.final_position == 0:
                rospy.loginfo('Resetting Robotiq Gripper')
                self.reset_robotiq_gripper()
            if goal.final_position == 1:
                rospy.loginfo('Opening Robotiq Gripper')
                self.open_gripper()
            if goal.final_position == 2:
                rospy.loginfo('Closing Robotiq Gripper')
                self.close_gripper()
            if goal.final_position == 3:
                rospy.loginfo('Status Robotiq Gripper')
                self.status_robotiq_gripper()
            self.open_gripper_server.set_succeeded()
        except:
            rospy.logfatal(
                'An error has occurred. Reset the gripper with the reset command in the action client.')

    def cb_gripper_position(self, goal) -> int:
        """Callback for the GripperPosition action.

        Args:
            goal (int): Value of 0-255 for the gripper. 0 is closed. 255 is open.
        """
        try:
            self.position_gripper(goal.final_position)
            self.position_gripper_server.set_succeeded()
        except ValueError:
            rospy.logwarn('Value must be between 0 - 255')
        except:
            rospy.logfatal(
                'An error has occurred. Reset the gripper with the reset command in the action client.')

    def range_mapper(self, val, input_min_val_range, input_max_val_range, output_min_val_range, output_max_val_range):
        """Maps values from one range to another

        Args:
            val (int): Target value

            input_min_val_range (int)
            input_max_val_range (int)

            output_min_val_range (int)
            output_max_val_range (int)

        Returns:
            result (int): Mapped value
        """
        result = (val-input_min_val_range)/(input_max_val_range-input_min_val_range) * \
            (output_max_val_range-output_min_val_range)+output_min_val_range
        return result


if __name__ == '__main__':
    rospy.init_node('robotiq_2f_gripper_action_server')
    robotiq = GripperNode("192.168.132.1")
    robotiq.publish_joint_states()
