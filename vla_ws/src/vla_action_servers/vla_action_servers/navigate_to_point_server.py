#!/usr/bin/env python3
"""
ROS 2 Action Server for NavigateToPoint action.

This server simulates base navigation to target poses.
In a real system, this would interface with Nav2 or a custom navigation stack.
"""

import time
import math

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from vla_interfaces.action import NavigateToPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


class NavigateToPointActionServer(Node):
    """Action server for navigation to target poses."""

    def __init__(self):
        super().__init__('navigate_to_point_server')

        self.get_logger().info('Initializing NavigateToPoint action server...')

        # Simulated current robot pose
        self._current_pose = PoseStamped()
        self._current_pose.header.frame_id = 'map'
        self._current_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        self._current_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self._action_server = ActionServer(
            self,
            NavigateToPoint,
            '/navigate_to_point',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('NavigateToPoint action server ready at /navigate_to_point')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f'Received navigation goal: '
            f'({goal_request.target_pose.pose.position.x:.2f}, '
            f'{goal_request.target_pose.pose.position.y:.2f})'
        )

        # Basic validation
        if goal_request.max_speed <= 0:
            self.get_logger().warn('Rejecting goal: max_speed must be positive')
            return GoalResponse.REJECT

        if goal_request.tolerance_xy < 0:
            self.get_logger().warn('Rejecting goal: tolerance_xy must be non-negative')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def _calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses."""
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx * dx + dy * dy)

    async def execute_callback(self, goal_handle):
        """Execute the navigation action."""
        self.get_logger().info('Executing navigation action')

        request = goal_handle.request
        feedback_msg = NavigateToPoint.Feedback()
        result = NavigateToPoint.Result()

        # Calculate total distance
        start_pose = self._current_pose.pose
        target_pose = request.target_pose.pose
        total_distance = self._calculate_distance(start_pose, target_pose)

        self.get_logger().info(f'Total distance to travel: {total_distance:.2f} meters')

        # Simulate navigation with phases
        phases = [
            ('planning', 0.10, 1.0),
            ('navigating', 0.90, 5.0),
            ('finalizing', 1.00, 0.5)
        ]

        total_time_elapsed = 0.0
        distance_traveled = 0.0
        recovery_attempts = 0

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

            steps = 20 if phase_name == 'navigating' else 5
            step_duration = phase_duration / steps

            for step in range(steps + 1):
                feedback_msg.current_state = phase_name
                progress = phase_progress * (step / steps)
                feedback_msg.progress_percent = progress * 100.0

                # Simulate distance remaining
                distance_traveled = total_distance * progress
                feedback_msg.distance_remaining = max(0.0, total_distance - distance_traveled)

                # Update simulated current pose
                interpolation_factor = progress
                feedback_msg.current_pose = PoseStamped()
                feedback_msg.current_pose.header.frame_id = 'map'
                feedback_msg.current_pose.pose.position.x = (
                    start_pose.position.x +
                    (target_pose.position.x - start_pose.position.x) * interpolation_factor
                )
                feedback_msg.current_pose.pose.position.y = (
                    start_pose.position.y +
                    (target_pose.position.y - start_pose.position.y) * interpolation_factor
                )
                feedback_msg.current_pose.pose.position.z = 0.0
                feedback_msg.current_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

                # Simulate occasional recovery
                if phase_name == 'navigating' and step % 8 == 0 and step > 0:
                    recovery_attempts += 1
                    feedback_msg.status_message = f'Recovery behavior {recovery_attempts}'
                else:
                    feedback_msg.status_message = phase_name.capitalize()

                feedback_msg.recovery_attempts = recovery_attempts

                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().debug(
                    f'{phase_name}: {feedback_msg.progress_percent:.1f}%, '
                    f'distance remaining: {feedback_msg.distance_remaining:.2f}m'
                )

                time.sleep(step_duration)
                total_time_elapsed += step_duration

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.error_message = 'Canceled during execution'
                    return result

        # Update current pose
        self._current_pose.pose = target_pose

        # Action succeeded
        goal_handle.succeed()

        result.success = True
        result.error_message = ''
        result.final_pose = request.target_pose
        result.distance_traveled = total_distance
        result.time_elapsed = total_time_elapsed

        self.get_logger().info(
            f'Navigation completed: {total_distance:.2f}m in {total_time_elapsed:.1f}s'
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    navigate_server = NavigateToPointActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(navigate_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigate_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()