#!/usr/bin/env python3
"""
ROS 2 Action Server for PickObject action.

This server simulates picking up objects in the environment.
In a real system, this would interface with motion planning and gripper control.
"""

import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from vla_interfaces.action import PickObject
from geometry_msgs.msg import Pose, Point, Quaternion


class PickObjectActionServer(Node):
    """Action server for picking up objects."""

    def __init__(self):
        super().__init__('pick_object_server')

        self.get_logger().info('Initializing PickObject action server...')

        # Create action server with reentrant callback group for concurrent handling
        self._action_server = ActionServer(
            self,
            PickObject,
            '/pick_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('PickObject action server ready at /pick_object')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f'Received pick goal request for object: {goal_request.object_id}'
        )

        # Validate goal parameters
        if not goal_request.object_id:
            self.get_logger().warn('Rejecting goal: object_id is empty')
            return GoalResponse.REJECT

        if goal_request.max_force <= 0:
            self.get_logger().warn('Rejecting goal: max_force must be positive')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the pick action."""
        self.get_logger().info(f'Executing pick action for: {goal_handle.request.object_id}')

        request = goal_handle.request
        feedback_msg = PickObject.Feedback()
        result = PickObject.Result()

        # Simulate pick sequence with multiple phases
        phases = [
            ('approaching', 0.25, 2.0),
            ('pre_grasp', 0.50, 1.5),
            ('grasping', 0.75, 2.0),
            ('lifting', 1.00, 1.5)
        ]

        for phase_name, phase_progress, phase_duration in phases:
            # Check if goal is still active
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                result.success = False
                result.error_message = 'Goal was aborted'
                return result

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.success = False
                result.error_message = 'Goal was canceled by client'
                return result

            # Simulate phase execution with progress updates
            steps = 10
            step_duration = phase_duration / steps

            for step in range(steps + 1):
                # Update feedback
                feedback_msg.current_phase = phase_name
                feedback_msg.progress_percent = (phase_progress * step / steps) * 100.0
                feedback_msg.status_message = f'{phase_name.capitalize()} object {request.object_id}'

                # Simulate current end-effector pose
                feedback_msg.current_ee_pose = Pose(
                    position=Point(x=0.5, y=0.2, z=0.3 + step * 0.01),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                )

                # Publish feedback
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().debug(
                    f'{phase_name}: {feedback_msg.progress_percent:.1f}%'
                )

                time.sleep(step_duration)

                # Check cancellation during sleep
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.error_message = 'Canceled during execution'
                    return result

        # Action succeeded
        goal_handle.succeed()

        result.success = True
        result.error_message = ''
        result.final_grasp_pose = Pose(
            position=Point(x=0.5, y=0.2, z=0.4),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        result.grasp_force = min(request.max_force, 45.0)  # Simulate actual force applied

        self.get_logger().info(
            f'Pick action completed successfully for {request.object_id}'
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    pick_server = PickObjectActionServer()

    # Use multi-threaded executor for concurrent action handling
    executor = MultiThreadedExecutor()
    executor.add_node(pick_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pick_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()