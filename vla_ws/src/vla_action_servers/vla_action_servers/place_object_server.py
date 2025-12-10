#!/usr/bin/env python3
"""
ROS 2 Action Server for PlaceObject action.

This server simulates placing objects at target locations.
In a real system, this would interface with motion planning and gripper control.
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from vla_interfaces.action import PlaceObject
from geometry_msgs.msg import Pose, Point, Quaternion


class PlaceObjectActionServer(Node):
    """Action server for placing objects."""

    def __init__(self):
        super().__init__('place_object_server')

        self.get_logger().info('Initializing PlaceObject action server...')

        self._action_server = ActionServer(
            self,
            PlaceObject,
            '/place_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('PlaceObject action server ready at /place_object')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(
            f'Received place goal request at position: '
            f'({goal_request.target_pose.position.x:.2f}, '
            f'{goal_request.target_pose.position.y:.2f}, '
            f'{goal_request.target_pose.position.z:.2f})'
        )

        # Basic validation
        if not goal_request.placement_type:
            self.get_logger().warn('Rejecting goal: placement_type is empty')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the place action."""
        self.get_logger().info('Executing place action')

        request = goal_handle.request
        feedback_msg = PlaceObject.Feedback()
        result = PlaceObject.Result()

        # Simulate place sequence
        phases = [
            ('moving_to_target', 0.25, 2.0),
            ('aligning', 0.50, 1.0),
            ('lowering', 0.75, 1.5),
            ('releasing', 1.00, 1.0)
        ]

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
                feedback_msg.status_message = f'{phase_name.replace("_", " ").capitalize()}'

                # Simulate current end-effector pose
                feedback_msg.current_ee_pose = Pose(
                    position=Point(
                        x=request.target_pose.position.x,
                        y=request.target_pose.position.y,
                        z=request.target_pose.position.z + (10 - step) * 0.02
                    ),
                    orientation=request.target_pose.orientation
                )

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

        # Action succeeded
        goal_handle.succeed()

        result.success = True
        result.error_message = ''
        result.final_object_pose = request.target_pose

        self.get_logger().info('Place action completed successfully')

        return result


def main(args=None):
    rclpy.init(args=args)

    place_server = PlaceObjectActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(place_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        place_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()