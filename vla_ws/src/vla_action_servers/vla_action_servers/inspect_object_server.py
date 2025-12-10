#!/usr/bin/env python3
"""
ROS 2 Action Server for InspectObject action.

This server simulates inspecting/observing objects in the environment.
In a real system, this would interface with cameras and perception systems.
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from vla_interfaces.action import InspectObject
from geometry_msgs.msg import Pose, Point, Quaternion


class InspectObjectActionServer(Node):
    """Action server for inspecting objects."""

    def __init__(self):
        super().__init__('inspect_object_server')

        self.get_logger().info('Initializing InspectObject action server...')

        self._action_server = ActionServer(
            self,
            InspectObject,
            '/inspect_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('InspectObject action server ready at /inspect_object')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f'Received inspect goal request for target: {goal_request.target_id}'
        )

        if not goal_request.target_id:
            self.get_logger().warn('Rejecting goal: target_id is empty')
            return GoalResponse.REJECT

        if goal_request.inspection_duration <= 0:
            self.get_logger().warn('Rejecting goal: inspection_duration must be positive')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the inspect action."""
        self.get_logger().info(f'Executing inspect action for: {goal_handle.request.target_id}')

        request = goal_handle.request
        feedback_msg = InspectObject.Feedback()
        result = InspectObject.Result()

        # Use requested inspection duration or default
        total_duration = request.inspection_duration if request.inspection_duration > 0 else 2.0

        # Simulate inspection sequence (adjust durations based on detailed_scan flag)
        if request.detailed_scan:
            phases = [
                ('orienting', 0.20, total_duration * 0.3),
                ('observing', 0.70, total_duration * 0.5),
                ('processing', 1.00, total_duration * 0.2)
            ]
        else:
            phases = [
                ('orienting', 0.30, total_duration * 0.3),
                ('observing', 0.80, total_duration * 0.5),
                ('processing', 1.00, total_duration * 0.2)
            ]

        images_captured = 0

        for phase_name, phase_progress, phase_duration in phases:
            if not goal_handle.is_active:
                result.success = False
                result.error_message = 'Goal was aborted'
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.error_message = 'Goal was canceled by client'
                return result

            steps = 10
            step_duration = phase_duration / steps

            for step in range(steps + 1):
                feedback_msg.current_phase = phase_name
                feedback_msg.progress_percent = (phase_progress * step / steps) * 100.0
                feedback_msg.time_remaining = (1.0 - phase_progress * step / steps) * total_duration
                feedback_msg.status_message = f'{phase_name.capitalize()} {request.target_id}'

                # Simulate image capture during observing phase
                if phase_name == 'observing' and step % 3 == 0:
                    images_captured += 1

                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().debug(
                    f'{phase_name}: {feedback_msg.progress_percent:.1f}%'
                )

                time.sleep(step_duration)

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.error_message = 'Canceled during execution'
                    return result

        # Action succeeded - simulate inspection results
        goal_handle.succeed()

        result.success = True
        result.error_message = ''
        result.observation_summary = (
            f'Inspected {request.target_id} at position '
            f'({request.target_position.x:.2f}, {request.target_position.y:.2f}, {request.target_position.z:.2f}). '
            f'Object appears to be a red cubic object with approximate dimensions 0.05m.'
        )
        result.num_images_captured = images_captured

        self.get_logger().info(
            f'Inspect action completed successfully for {request.target_id}, '
            f'captured {images_captured} images'
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    inspect_server = InspectObjectActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(inspect_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        inspect_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()