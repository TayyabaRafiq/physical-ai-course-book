"""
Helper functions for creating ROS 2 action goal messages from robot actions.
"""

from typing import Dict, Any
from ..models.robot_action import RobotAction, ActionType

try:
    from vla_interfaces.action import PickObject, PlaceObject, NavigateToPoint, InspectObject
    from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def create_pick_object_goal(action: RobotAction) -> 'PickObject.Goal':
    """Create PickObject goal from robot action."""
    if not ROS2_AVAILABLE:
        raise RuntimeError("ROS 2 libraries not available")

    goal_msg = PickObject.Goal()
    goal_data = action.goal_message

    goal_msg.object_id = goal_data.get('object_id', '')

    # Approach offset
    offset_data = goal_data.get('approach_offset', {})
    goal_msg.approach_offset = Vector3(
        x=float(offset_data.get('x', 0.0)),
        y=float(offset_data.get('y', 0.0)),
        z=float(offset_data.get('z', 0.1))
    )

    goal_msg.grasp_type = goal_data.get('grasp_type', 'top')
    goal_msg.max_force = float(goal_data.get('max_force', 50.0))

    return goal_msg


def create_place_object_goal(action: RobotAction) -> 'PlaceObject.Goal':
    """Create PlaceObject goal from robot action."""
    if not ROS2_AVAILABLE:
        raise RuntimeError("ROS 2 libraries not available")

    goal_msg = PlaceObject.Goal()
    goal_data = action.goal_message

    # Target pose
    pose_data = goal_data.get('target_pose', {})
    position = pose_data.get('position', {})
    orientation = pose_data.get('orientation', {})

    goal_msg.target_pose = Pose(
        position=Point(
            x=float(position.get('x', 0.0)),
            y=float(position.get('y', 0.0)),
            z=float(position.get('z', 0.0))
        ),
        orientation=Quaternion(
            x=float(orientation.get('x', 0.0)),
            y=float(orientation.get('y', 0.0)),
            z=float(orientation.get('z', 0.0)),
            w=float(orientation.get('w', 1.0))
        )
    )

    goal_msg.placement_type = goal_data.get('placement_type', 'gentle')

    return goal_msg


def create_navigate_to_point_goal(action: RobotAction) -> 'NavigateToPoint.Goal':
    """Create NavigateToPoint goal from robot action."""
    if not ROS2_AVAILABLE:
        raise RuntimeError("ROS 2 libraries not available")

    goal_msg = NavigateToPoint.Goal()
    goal_data = action.goal_message

    # Target pose
    pose_data = goal_data.get('target_pose', {})
    position = pose_data.get('position', {})
    orientation = pose_data.get('orientation', {})

    goal_msg.target_pose = PoseStamped()
    goal_msg.target_pose.header.frame_id = goal_data.get('frame_id', 'map')
    goal_msg.target_pose.pose = Pose(
        position=Point(
            x=float(position.get('x', 0.0)),
            y=float(position.get('y', 0.0)),
            z=float(position.get('z', 0.0))
        ),
        orientation=Quaternion(
            x=float(orientation.get('x', 0.0)),
            y=float(orientation.get('y', 0.0)),
            z=float(orientation.get('z', 0.0)),
            w=float(orientation.get('w', 1.0))
        )
    )

    goal_msg.tolerance_xy = float(goal_data.get('tolerance_xy', 0.1))
    goal_msg.tolerance_theta = float(goal_data.get('tolerance_theta', 0.15))
    goal_msg.max_speed = float(goal_data.get('max_speed', 0.5))
    goal_msg.avoid_obstacles = bool(goal_data.get('avoid_obstacles', True))

    return goal_msg


def create_inspect_object_goal(action: RobotAction) -> 'InspectObject.Goal':
    """Create InspectObject goal from robot action."""
    if not ROS2_AVAILABLE:
        raise RuntimeError("ROS 2 libraries not available")

    goal_msg = InspectObject.Goal()
    goal_data = action.goal_message

    goal_msg.target_id = goal_data.get('target_id', '')

    # Target position
    position_data = goal_data.get('target_position', {})
    goal_msg.target_position = Point(
        x=float(position_data.get('x', 0.0)),
        y=float(position_data.get('y', 0.0)),
        z=float(position_data.get('z', 0.0))
    )

    goal_msg.inspection_duration = float(goal_data.get('inspection_duration', 2.0))
    goal_msg.detailed_scan = bool(goal_data.get('detailed_scan', False))

    return goal_msg


def extract_feedback_data(feedback_msg: Any) -> Dict[str, Any]:
    """Extract feedback data from ROS message into dict."""
    feedback_dict = {}

    if hasattr(feedback_msg, 'progress_percent'):
        feedback_dict['progress_percent'] = feedback_msg.progress_percent

    if hasattr(feedback_msg, 'current_phase'):
        feedback_dict['current_phase'] = feedback_msg.current_phase
    elif hasattr(feedback_msg, 'current_state'):
        feedback_dict['current_phase'] = feedback_msg.current_state

    if hasattr(feedback_msg, 'status_message'):
        feedback_dict['status_message'] = feedback_msg.status_message

    return feedback_dict