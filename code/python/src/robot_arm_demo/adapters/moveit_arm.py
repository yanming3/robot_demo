"""ArmController 的 MoveIt2 实现。

逐字段复刻原 pick_place_state_machine.send_move_goal / send_joint_goal /
remove_coke_from_scene 的 goal 构造与判定逻辑（含成功后 post_move_settle），
仅把配置项改为从 ArmConfig/GraspConfig 读取。
"""

from __future__ import annotations

import time

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    CollisionObject,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from ..core.data import ArmConfig, GraspConfig


def _wait_future(node: Node, future, timeout=30.0) -> bool:
    """轮询等待 future 完成，不阻塞 executor（自原节点搬入）。"""
    start = time.time()
    while not future.done() and time.time() - start < timeout:
        time.sleep(0.01)
    return future.done()


class MoveItArm:
    """在宿主 Node 上创建 /move_action 客户端与 Planning Scene 服务客户端。

    构造即创建资源并 wait_for_server（与原 FSM __init__ 行为一致）。
    """

    def __init__(self, node: Node, arm_cfg: ArmConfig, grasp_cfg: GraspConfig):
        self.node = node
        self.arm = arm_cfg
        self.grasp = grasp_cfg
        self._log = node.get_logger()

        self.move_client = ActionClient(node, MoveGroup, "/move_action")
        # Planning Scene 服务：抓取前移除目标 collision object 避免碰撞检测失败
        self.get_scene_client = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.apply_scene_client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.move_client.wait_for_server()

    # ── 笛卡尔目标 ──

    def move_cartesian(self, position, orientation_wxyz=None,
                       velocity_scaling=None) -> bool:
        arm = self.arm
        if velocity_scaling is None:
            velocity_scaling = arm.default_velocity_scaling
        if orientation_wxyz is None:
            # eef 目标朝向 = home 姿态的 link8 朝向（wxyz），让指尖 pad 竖直
            # 向下、手指水平，精确对准目标中心（历史教训见 config 注释）。
            orientation_wxyz = arm.tip_orientation_wxyz

        goal = MoveGroup.Goal()
        goal.request.group_name = arm.move_group_name
        goal.request.num_planning_attempts = arm.num_planning_attempts
        goal.request.allowed_planning_time = arm.allowed_planning_time
        goal.request.start_state.is_diff = True
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = velocity_scaling

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = arm.base_frame
        pos_constraint.link_name = arm.eef_link
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [arm.position_tolerance]
        pos_constraint.constraint_region.primitives.append(region)
        region_pose = Pose()
        region_pose.position.x = position[0]
        region_pose.position.y = position[1]
        region_pose.position.z = position[2]
        region_pose.orientation.w = 1.0
        pos_constraint.constraint_region.primitive_poses.append(region_pose)
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = arm.base_frame
        ori_constraint.link_name = arm.eef_link
        ori_constraint.orientation.w = orientation_wxyz[0]
        ori_constraint.orientation.x = orientation_wxyz[1]
        ori_constraint.orientation.y = orientation_wxyz[2]
        ori_constraint.orientation.z = orientation_wxyz[3]
        ori_constraint.absolute_x_axis_tolerance = arm.orientation_tolerance
        ori_constraint.absolute_y_axis_tolerance = arm.orientation_tolerance
        ori_constraint.absolute_z_axis_tolerance = arm.orientation_tolerance
        ori_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)
        goal_constraints.orientation_constraints.append(ori_constraint)
        goal.request.goal_constraints.append(goal_constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = arm.replan_attempts

        self._log.info(f"Sending move goal to {position}")
        return self._execute_move_goal(goal)

    # ── 关节空间目标 ──

    def move_to_joint(self, joint_values, velocity_scaling=None) -> bool:
        """按关节约束移动到 joint_values（顺序=home_joint_names）。"""
        arm = self.arm
        if velocity_scaling is None:
            velocity_scaling = arm.default_velocity_scaling

        goal = MoveGroup.Goal()
        goal.request.group_name = arm.move_group_name
        goal.request.num_planning_attempts = arm.num_planning_attempts
        goal.request.allowed_planning_time = arm.allowed_planning_time
        goal.request.start_state.is_diff = True
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = velocity_scaling

        constraints = Constraints()
        for name, value in zip(arm.home_joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = arm.home_joint_tolerance
            jc.tolerance_below = arm.home_joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = arm.replan_attempts

        self._log.info(f"Sending joint goal to home: {joint_values}")
        return self._execute_move_goal(goal)

    # ── 共享执行路径 ──

    def _execute_move_goal(self, goal: MoveGroup.Goal) -> bool:
        """发送 → 等接受 → 等结果；error_code==1 视为成功，成功后 settle。"""
        future = self.move_client.send_goal_async(goal)
        if not _wait_future(self.node, future, timeout=10.0):
            self._log.error("Move goal send timed out")
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._log.error("Move goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        if not _wait_future(self.node, result_future, timeout=60.0):
            self._log.error("Move goal timed out")
            return False
        result = result_future.result()
        if result is None:
            self._log.error("Move result None")
            return False
        success = result.result.error_code.val == 1
        self._log.info(f"Move result: error_code={result.result.error_code.val}")
        if success:
            # 给 PlanningSceneMonitor 时间吸收 ros2_control 上报的新关节状态；
            # 否则下一 move 的 allowed_start_tolerance 会拿陈旧场景对比实时
            # 状态而拒轨迹（START_STATE_INVALID / CONTROL_FAILED 成因之一）。
            time.sleep(self.grasp.post_move_settle)
        return success

    # ── Planning Scene 编辑 ──

    def remove_object_from_scene(self, object_id: str) -> bool:
        """从 Planning Scene 移除 collision object（不存在时无害返回 True）。"""
        if not self.get_scene_client.wait_for_service(timeout_sec=2.0):
            self._log.warn("get_planning_scene service not available, skipping.")
            return False
        if not self.apply_scene_client.wait_for_service(timeout_sec=2.0):
            self._log.warn("apply_planning_scene service not available, skipping.")
            return False

        req = GetPlanningScene.Request()
        req.components.components = 0  # full scene
        future = self.get_scene_client.call_async(req)
        if not _wait_future(self.node, future, timeout=5.0):
            self._log.error("get_planning_scene timed out")
            return False
        scene = future.result().scene

        removed = False
        for obj in scene.world.collision_objects:
            if obj.id == object_id:
                obj.operation = CollisionObject.REMOVE
                removed = True
                break
        if not removed:
            self._log.info(
                f"No '{object_id}' in planning scene, nothing to remove."
            )
            return True

        apply_req = ApplyPlanningScene.Request()
        apply_req.scene = scene
        apply_future = self.apply_scene_client.call_async(apply_req)
        if not _wait_future(self.node, apply_future, timeout=5.0):
            self._log.error("apply_planning_scene timed out")
            return False
        self._log.info(f"Removed '{object_id}' from planning scene.")
        return True
