import numpy as np

from rclpy.action import ActionClient

from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup


class MoveGroupActionClient():
    def __init__(self, node):
        self._action_client = ActionClient(node, MoveGroup, '/move_action')
        self.node = node

    def send_goal(self, target_angles):
        motion_plan_request = MotionPlanRequest()

        motion_plan_request.workspace_parameters.header.stamp = self.node.get_clock().now().to_msg()
        motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        motion_plan_request.workspace_parameters.min_corner.x = -1.0
        motion_plan_request.workspace_parameters.min_corner.y = -1.0
        motion_plan_request.workspace_parameters.min_corner.z = -1.0
        motion_plan_request.workspace_parameters.max_corner.x = 1.0
        motion_plan_request.workspace_parameters.max_corner.y = 1.0
        motion_plan_request.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()
        for idx, target_angle in enumerate(target_angles):
            jc = JointConstraint()
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            jc.joint_name = f'joint_{idx+1}'
            jc.position = np.deg2rad(target_angle)
            constraints.joint_constraints.append(jc)

        motion_plan_request.goal_constraints.append(constraints)

        motion_plan_request.pipeline_id = 'ompl'
        motion_plan_request.group_name = 'manipulator'
        motion_plan_request.num_planning_attempts = 4
        motion_plan_request.allowed_planning_time = 10.0
        motion_plan_request.max_velocity_scaling_factor = 0.4
        motion_plan_request.max_acceleration_scaling_factor = 0.4
        motion_plan_request.max_cartesian_speed = 0.0

        planning_options = PlanningOptions()
        planning_options.plan_only = False
        planning_options.look_around = True
        planning_options.look_around_attempts = 5
        planning_options.max_safe_execution_cost = 0.
        planning_options.replan = True
        planning_options.replan_attempts = 4
        planning_options.replan_delay = 0.1

        goal_msg = MoveGroup.Goal()
        goal_msg.request = motion_plan_request
        goal_msg.planning_options = planning_options

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
