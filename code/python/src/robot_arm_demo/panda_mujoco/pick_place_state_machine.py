#!/usr/bin/env python3
"""V3-T004: Panda pick-place state machine."""

import json
import math
import time
import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    Constraints,
    PlanningScene,
    CollisionObject,

)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from mujoco_ros2_control_msgs.msg import FreeJointStateArray

from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

from robot_arm_demo.demos.panda_mujoco.config import build_panda_mujoco_config


def _wait_future(node, future, timeout=30.0):
    """Wait for a future without deadlocking."""
    start = time.time()
    while not future.done() and time.time() - start < timeout:
        time.sleep(0.01)
    return future.done()


class PickPlaceStateMachine(Node):
    def __init__(self):
        super().__init__("pick_place_state_machine")
        # 全部机器人/物体/抓取参数来自 demo 配置（原硬编码常量的唯一来源）
        self.cfg = build_panda_mujoco_config()
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.gripper_client = ActionClient(
            self, GripperCommand, "/panda_hand_controller/gripper_cmd"
        )
        self.move_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info("Action servers connected.")
        self.state = "IDLE"
        self.object_pose = [0.5, 0.0, 0.3]
        self.command_sub = self.create_subscription(
            String, "/robot_command", self.command_callback, 10
        )
        # M0: 订阅 /joint_states 读真实手指位置。历史教训：Gripper Action 的
        # reached_goal 与真实 /joint_states 不一致，不能当作夹持成功的证据。
        self.latest_joint_state = None
        self.joint_state_lock = threading.Lock()
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )
        # M2: 订阅 /free_joint_states 读可乐实时 world 位姿（MuJoCo ground truth）。
        # scene.xml 里 panda_link0 即世界原点，world == panda_link0，坐标可直接用于微调。
        self.latest_coke_pose = None
        self.coke_pose_lock = threading.Lock()
        self.coke_pose_sub = self.create_subscription(
            FreeJointStateArray,
            "/free_joint_state_publisher/free_joint_states",
            self.coke_pose_callback,
            10,
        )
        self._busy = False
        # Planning Scene 服务：抓取前移除可乐 collision object 避免碰撞检测失败
        self.get_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.get_logger().info("Waiting for commands on /robot_command ...")

    def remove_coke_from_scene(self):
        """从 Planning Scene 移除可乐 collision object，避免夹爪碰撞检测失败。"""
        if not self.get_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("get_planning_scene service not available, skipping.")
            return
        if not self.apply_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("apply_planning_scene service not available, skipping.")
            return

        # 获取当前 Planning Scene
        req = GetPlanningScene.Request()
        req.components.components = 0  # full scene
        future = self.get_scene_client.call_async(req)
        if not _wait_future(self, future, timeout=5.0):
            self.get_logger().error("get_planning_scene timed out")
            return
        scene = future.result().scene

        # 查找并移除目标物体
        removed = False
        oid = self.cfg.object.object_id
        for obj in scene.world.collision_objects:
            if obj.id == oid:
                obj.operation = CollisionObject.REMOVE
                removed = True
                break
        if not removed:
            self.get_logger().info(f"No '{oid}' in planning scene, nothing to remove.")
            return

        # 应用更新
        apply_req = ApplyPlanningScene.Request()
        apply_req.scene = scene
        apply_future = self.apply_scene_client.call_async(apply_req)
        if not _wait_future(self, apply_future, timeout=5.0):
            self.get_logger().error("apply_planning_scene timed out")
            return
        self.get_logger().info(f"Removed '{oid}' from planning scene.")

    def joint_state_callback(self, msg):
        """缓存最新 /joint_states 消息，供闭合后读取真实手指位置。"""
        with self.joint_state_lock:
            self.latest_joint_state = msg

    def coke_pose_callback(self, msg):
        """缓存可乐 free joint 的 world 位姿（MuJoCo ground truth）。"""
        for fj in msg.free_joints:
            if fj.name == self.cfg.object.object_id:
                with self.coke_pose_lock:
                    self.latest_coke_pose = (
                        fj.pose.pose.position.x,
                        fj.pose.pose.position.y,
                        fj.pose.pose.position.z,
                    )
                return

    def read_coke_pose(self):
        """读最新可乐 world 位姿 (x, y, z)，未收到返回 None。"""
        with self.coke_pose_lock:
            return self.latest_coke_pose

    def read_finger_positions(self):
        """读 /joint_states 里 panda_finger_joint1/2 的真实位置。"""
        with self.joint_state_lock:
            if self.latest_joint_state is None:
                return None
            names = list(self.latest_joint_state.name)
            positions = list(self.latest_joint_state.position)
        fingers = {}
        gripper_names = self.cfg.arm.gripper_joint_names
        for i, name in enumerate(names):
            if name in gripper_names:
                fingers[name] = positions[i] if i < len(positions) else float("nan")
        return fingers

    def command_callback(self, msg):
        """收到 LLM Planner 发布的 JSON 指令，解析并执行 pick-place。"""
        if self._busy:
            self.get_logger().warn("Busy, ignoring command.")
            return
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON: {msg.data}")
            return
        target = cmd.get("target_object")
        action = cmd.get("action")
        self.get_logger().info(f"Received command: target={target}, action={action}")

        if action != "pick":
            self.get_logger().warn(f"Unsupported action: {action}, staying IDLE.")
            return

        pose = cmd.get("position")
        if pose is None:
            self.get_logger().warn("No position in command, staying IDLE.")
            return

        self._busy = True
        self.object_pose = pose
        thread = threading.Thread(target=self._execute_pick_place, daemon=True)
        thread.start()

    def _execute_pick_place(self):
        """在单独线程里执行 pick-place，避免阻塞 executor。"""
        success = self.run()
        self._busy = False
        if success:
            self.get_logger().info("Pick-place completed successfully.")
        else:
            self.get_logger().error("Pick-place failed.")

    def send_move_goal(self, position, orientation=None, velocity_scaling=None):
        if velocity_scaling is None:
            velocity_scaling = self.cfg.arm.default_velocity_scaling
        if orientation is None:
            # link8 目标朝向 = home 姿态的 link8 朝向（wxyz），让指尖 pad 竖直向下、
            # 手指水平，精确对准可乐中心。此前误用 [0,1,0,0]（那是 panda_hand 的朝向）
            # 且 z 轴容差 3.14 过宽，MoveIt 解出 z 轴乱转的姿态 → pad 偏离可乐 → 滑脱。
            orientation = list(self.cfg.arm.tip_orientation_wxyz)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.cfg.arm.move_group_name
        goal.request.num_planning_attempts = self.cfg.arm.num_planning_attempts
        goal.request.allowed_planning_time = self.cfg.arm.allowed_planning_time
        goal.request.start_state.is_diff = True
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = velocity_scaling

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.cfg.arm.base_frame
        pos_constraint.link_name = self.cfg.arm.eef_link
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [self.cfg.arm.position_tolerance]   # 容差球，给 IK 更多求解空间
        pos_constraint.constraint_region.primitives.append(region)
        region_pose = Pose()
        region_pose.position.x = position[0]
        region_pose.position.y = position[1]
        region_pose.position.z = position[2]
        region_pose.orientation.w = 1.0
        pos_constraint.constraint_region.primitive_poses.append(region_pose)
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.cfg.arm.base_frame
        ori_constraint.link_name = self.cfg.arm.eef_link
        ori_constraint.orientation.w = orientation[0]
        ori_constraint.orientation.x = orientation[1]
        ori_constraint.orientation.y = orientation[2]
        ori_constraint.orientation.z = orientation[3]
        ori_constraint.absolute_x_axis_tolerance = self.cfg.arm.orientation_tolerance
        ori_constraint.absolute_y_axis_tolerance = self.cfg.arm.orientation_tolerance
        ori_constraint.absolute_z_axis_tolerance = self.cfg.arm.orientation_tolerance
        ori_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pos_constraint)
        goal_constraints.orientation_constraints.append(ori_constraint)
        goal.request.goal_constraints.append(goal_constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = self.cfg.arm.replan_attempts

        self.get_logger().info(f"Sending move goal to {position}")
        future = self.move_client.send_goal_async(goal)
        if not _wait_future(self, future, timeout=10.0):
            self.get_logger().error("Move goal send timed out")
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Move goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        if not _wait_future(self, result_future, timeout=60.0):
            self.get_logger().error("Move goal timed out")
            return False
        result = result_future.result()
        if result is None:
            self.get_logger().error("Move result None")
            return False
        success = result.result.error_code.val == 1
        self.get_logger().info(f"Move result: error_code={result.result.error_code.val}")
        if success:
            # Give the PlanningSceneMonitor time to absorb the new joint_state
            # reported by ros2_control after a move completes. Without this,
            # the next move's allowed_start_tolerance check compares a stale
            # scene state against the live state and rejects the trajectory.
            time.sleep(self.cfg.grasp.post_move_settle)
        return success

    def send_joint_goal(self, joint_values, velocity_scaling=None):
        """按关节角复位到目标位形（JointConstraint）。

        与 send_move_goal 的笛卡尔 position/orientation 约束不同，这里直接把
        每个关节钉到目标值（±容差）。7-DOF 冗余臂的笛卡尔约束有无数关节解，
        IK 可能把 wrist（joint2/4）拧到关节极限外；关节约束能解出唯一、明确的
        位形。用于 RETURN_HOME 复位到 home 关节位形，保证下一轮起点不越界。
        """
        if velocity_scaling is None:
            velocity_scaling = self.cfg.arm.default_velocity_scaling

        goal = MoveGroup.Goal()
        goal.request.group_name = self.cfg.arm.move_group_name
        goal.request.num_planning_attempts = self.cfg.arm.num_planning_attempts
        goal.request.allowed_planning_time = self.cfg.arm.allowed_planning_time
        goal.request.start_state.is_diff = True
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = velocity_scaling

        constraints = Constraints()
        for name, value in zip(self.cfg.arm.home_joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = self.cfg.arm.home_joint_tolerance
            jc.tolerance_below = self.cfg.arm.home_joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = self.cfg.arm.replan_attempts

        self.get_logger().info(
            f"Sending joint goal to home: {joint_values}"
        )
        future = self.move_client.send_goal_async(goal)
        if not _wait_future(self, future, timeout=10.0):
            self.get_logger().error("Joint goal send timed out")
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Joint goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        if not _wait_future(self, result_future, timeout=60.0):
            self.get_logger().error("Joint goal timed out")
            return False
        result = result_future.result()
        if result is None:
            self.get_logger().error("Joint goal result None")
            return False
        success = result.result.error_code.val == 1
        self.get_logger().info(f"Joint goal result: error_code={result.result.error_code.val}")
        if success:
            time.sleep(self.cfg.grasp.post_move_settle)
        return success

    def send_gripper_goal(self, position, max_effort=10.0):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self.get_logger().info(f"Sending gripper goal: position={position}")
        future = self.gripper_client.send_goal_async(goal)
        if not _wait_future(self, future, timeout=10.0):
            self.get_logger().error("Gripper goal send timed out")
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        if not _wait_future(self, result_future, timeout=10.0):
            self.get_logger().error("Gripper goal timed out")
            return False
        result = result_future.result()
        if result is None:
            self.get_logger().error("Gripper result None")
            return False
        self.get_logger().info(
            f"Gripper result: position={result.result.position:.4f}, reached={result.result.reached_goal}"
        )
        return result.result.reached_goal

    # ── 逐步收紧 + 微抬验证（防滑）──

    def _read_finger_avg(self):
        """读两指真实 qpos 的平均值；读不到返回 None。"""
        fingers = self.read_finger_positions()
        if not fingers:
            return None
        f1 = fingers.get(self.cfg.arm.gripper_joint_names[0])
        f2 = fingers.get(self.cfg.arm.gripper_joint_names[1])
        if f1 is None or f2 is None:
            return None
        return (f1 + f2) / 2.0

    def _coke_displacement_from(self, ref):
        """可乐当前 world 位姿相对 ref 的欧氏位移（m）；读不到返回 None。"""
        pose = self.read_coke_pose()
        if pose is None or ref is None:
            return None
        dx = pose[0] - ref[0]
        dy = pose[1] - ref[1]
        dz = pose[2] - ref[2]
        return math.hypot(math.hypot(dx, dy), dz)

    def _grasp_verified(self, coke_ref):
        """双信号判定：两指 qpos ∈ [0.030,0.033] 且可乐位移 < 3mm，连续两次达标。"""
        ok_count = 0
        for sample in range(self.cfg.grasp.stable_samples):
            avg = self._read_finger_avg()
            disp = self._coke_displacement_from(coke_ref)
            if avg is None or disp is None:
                self.get_logger().warn(
                    f"[VERIFY_GRASP] 信号缺失 finger_avg={avg} coke_disp={disp}"
                )
                return False
            qpos_ok = self.cfg.grasp.contact_qpos_min <= avg <= self.cfg.grasp.contact_qpos_max
            disp_ok = disp < self.cfg.grasp.displacement_tolerance
            self.get_logger().info(
                f"[VERIFY_GRASP] sample={sample + 1} finger_avg={avg:.4f} "
                f"coke_disp={disp:.4f} qpos_ok={qpos_ok} disp_ok={disp_ok}"
            )
            if qpos_ok and disp_ok:
                ok_count += 1
            if sample < self.cfg.grasp.stable_samples - 1:
                time.sleep(self.cfg.grasp.stable_interval)
        return ok_count == self.cfg.grasp.stable_samples

    def _step_gripper(self, target, coke_ref, phase):
        """单步收紧并检查：可乐不被推走、手指不穿透。返回是否可继续。"""
        self.send_gripper_goal(target, max_effort=10.0)
        time.sleep(self.cfg.grasp.step_settle)
        avg = self._read_finger_avg()
        disp = self._coke_displacement_from(coke_ref)
        avg_str = f"{avg:.4f}" if avg is not None else "None"
        disp_str = f"{disp:.4f}" if disp is not None else "None"
        self.get_logger().info(
            f"[CLOSE_GRIPPER] {phase} target={target:.4f} "
            f"finger_avg={avg_str} coke_disp={disp_str}"
        )
        if avg is None:
            self.get_logger().error("[CLOSE_GRIPPER] 读不到手指 qpos，中止收紧")
            return False
        if avg < self.cfg.grasp.min_qpos:
            self.get_logger().warn(
                f"[CLOSE_GRIPPER] 手指 qpos={avg:.4f} 低于安全下限 "
                f"{self.cfg.grasp.min_qpos}，疑似穿透，中止"
            )
            return False
        if disp is None or disp >= self.cfg.grasp.displacement_tolerance:
            self.get_logger().warn(
                f"[CLOSE_GRIPPER] 可乐被推动 disp={disp_str}，中止（推太狠）"
            )
            return False
        return True

    def close_gripper_staged(self, coke_ref):
        """两段式逐步收紧夹爪，返回 (是否夹住, 最终平均 qpos)。

        阶段1 粗调：大步逼近到手指 qpos 进入接触区上沿（≤0.034），
        阶段2 精调：小步收紧，每步双信号判定，连续达标即成功。
        全程每步检查可乐不被推走、手指不穿透。
        """
        target = self.cfg.arm.gripper_open_pos
        final_avg = None

        self.get_logger().info(
            f"[CLOSE_GRIPPER] 阶段1 粗调开始（步长 {self.cfg.grasp.coarse_step * 1000:.0f}mm，"
            f"起点 {self.cfg.arm.gripper_open_pos}）"
        )
        while target > self.cfg.grasp.min_qpos:
            target = max(self.cfg.grasp.min_qpos, target - self.cfg.grasp.coarse_step)
            if not self._step_gripper(target, coke_ref, "coarse"):
                return False, self._read_finger_avg()
            final_avg = self._read_finger_avg()
            # 手指 qpos 进入接触区上沿（可乐表面附近），转精调
            if final_avg is not None and final_avg <= self.cfg.grasp.contact_qpos_max + 0.001:
                self.get_logger().info(
                    f"[CLOSE_GRIPPER] 粗调到达接触区（avg={final_avg:.4f}），转精调"
                )
                break
        else:
            self.get_logger().warn(
                "[CLOSE_GRIPPER] 粗调收到安全下限仍未进入接触区（可能空闭合/穿透）"
            )
            return False, final_avg

        self.get_logger().info(
            f"[CLOSE_GRIPPER] 阶段2 精调开始（步长 {self.cfg.grasp.fine_step * 1000:.0f}mm）"
        )
        while target > self.cfg.grasp.min_qpos:
            target = max(self.cfg.grasp.min_qpos, target - self.cfg.grasp.fine_step)
            if not self._step_gripper(target, coke_ref, "fine"):
                return False, self._read_finger_avg()
            if self._grasp_verified(coke_ref):
                final_avg = self._read_finger_avg()
                self.get_logger().info(
                    f"[CLOSE_GRIPPER] 双信号达标，夹紧成功（avg={final_avg:.4f}）"
                )
                return True, final_avg
        self.get_logger().warn(
            "[CLOSE_GRIPPER] 精调收到安全下限仍未双信号达标"
        )
        return False, self._read_finger_avg()

    def verify_micro_lift(self, ox, oy, grasp_z):
        """慢速微抬 1.5cm，验证可乐 z 跟着抬升（物理抓牢）。"""
        before = self.read_coke_pose()
        if before is None:
            self.get_logger().error("[MICRO_LIFT] 微抬前读不到可乐位姿")
            return False
        target_z = grasp_z + self.cfg.grasp.micro_lift_height
        vsc = self.cfg.grasp.micro_lift_velocity_scaling
        self.get_logger().info(
            f"[MICRO_LIFT] 慢速微抬到 z={target_z:.4f} "
            f"（抬 {self.cfg.grasp.micro_lift_height * 100:.1f}cm，velocity_scaling={vsc}）"
        )
        if not self.send_move_goal([ox, oy, target_z], velocity_scaling=vsc):
            self.get_logger().error("[MICRO_LIFT] 微抬移动失败")
            return False
        after = self.read_coke_pose()
        if after is None:
            self.get_logger().error("[MICRO_LIFT] 微抬后读不到可乐位姿")
            return False
        dz = after[2] - before[2]
        lateral = math.hypot(after[0] - before[0], after[1] - before[1])
        self.get_logger().info(
            f"[MICRO_LIFT] 可乐位移 dz={dz:.4f} lateral={lateral:.4f} "
            f"（需 dz≥{self.cfg.grasp.micro_lift_z_min} 且 lateral<{self.cfg.grasp.micro_lift_lateral_max}）"
        )
        if dz < self.cfg.grasp.micro_lift_z_min:
            self.get_logger().warn("[MICRO_LIFT] 可乐 z 未跟着抬升 → 未抓牢")
            return False
        if lateral >= self.cfg.grasp.micro_lift_lateral_max:
            self.get_logger().warn("[MICRO_LIFT] 可乐侧向漂移过大 → 可能滑脱")
            return False
        self.get_logger().info("[MICRO_LIFT] 微抬验证通过，可乐物理抓牢")
        return True

    def recover_grasp(self):
        """失败恢复：张开夹爪 → 读可乐当前坐标 → 重新对准下降。

        重试必须基于可乐「当前」坐标（上一次尝试可能把它推偏）和机械臂
        「当前」坐姿（MoveIt 会从当前关节状态规划），不能复用初始 object_pose。
        返回 (cx, cy, grasp_z, hover_z)；读不到可乐位姿返回 None。
        """
        self.get_logger().warn("[RECOVER] 张开夹爪并重新对准可乐当前位置重试")
        self.send_gripper_goal(self.cfg.arm.gripper_open_pos, max_effort=0.0)
        time.sleep(self.cfg.grasp.recover_settle)
        cur = self.read_coke_pose()
        if cur is None:
            self.get_logger().error("[RECOVER] 读不到可乐当前位置，无法重新对准")
            return None
        cx, cy, cz = cur
        new_grasp_z = max(cz + self.cfg.arm.finger_tip_offset, self.cfg.arm.min_eef_z)
        new_hover_z = new_grasp_z + self.cfg.grasp.hover_offset
        self.get_logger().info(
            f"[RECOVER] 可乐当前 ({cx:.4f}, {cy:.4f}, {cz:.4f}) "
            f"→ grasp_z={new_grasp_z:.4f}"
        )
        # 从机械臂当前坐姿直接规划到可乐当前坐标上方（悬停），再下降
        if not self.send_move_goal([cx, cy, new_hover_z]):
            self.get_logger().error("[RECOVER] 回悬停位失败")
            return None
        if not self.send_move_goal([cx, cy, new_grasp_z]):
            self.get_logger().error("[RECOVER] 重新下降到抓取位失败")
            return None
        return cx, cy, new_grasp_z, new_hover_z

    def run(self):
        """执行 pick-place 流程。

        感知节点给出的 object_pose 是可乐中心在 panda_link0 坐标系下的位置。
        position constraint 作用在 panda_link8，link8 到指尖有 self.cfg.arm.finger_tip_offset 偏移。
        所以要让指尖到达可乐中心高度，link8 目标 z = oz + self.cfg.arm.finger_tip_offset。
        """
        ox, oy, oz = self.object_pose

        # 可乐中心高度，夹爪需要让指尖对准这个高度
        # link8 目标 = 可乐中心 + link8 到指尖偏移
        grasp_z = oz + self.cfg.arm.finger_tip_offset
        # link8 最低下降高度（安全下限）。理想抓取点让指尖对准可乐中心
        # （oz≈0.059 → grasp_z≈0.162），但 0.162 时 hand_c 掌心底部(≈0.1197)与
        # 可乐顶面(0.122)几乎重合，DESCEND 会把可乐撞倒。故下限提到 0.20
        # （见 self.cfg.arm.min_eef_z 常量注释），指尖改夹可乐上半部分。
        if grasp_z < self.cfg.arm.min_eef_z:
            self.get_logger().warn(
                f"grasp_z={grasp_z:.3f} below min {self.cfg.arm.min_eef_z}, clamping"
            )
            grasp_z = self.cfg.arm.min_eef_z
        hover_z = grasp_z + self.cfg.grasp.hover_offset   # 悬停在抓取点上方 12cm

        # 1. 张开夹爪
        self.state = "OPEN_GRIPPER"
        self.get_logger().info(f"=== State: {self.state} ===")
        if not self.send_gripper_goal(self.cfg.arm.gripper_open_pos, max_effort=0.0):
            self.state = "IDLE"
            return False

        # 2. 移动到可乐正上方（悬停）
        self.state = "MOVE_ABOVE_OBJECT"
        self.get_logger().info(f"=== State: {self.state} ===")
        self.get_logger().info(f"  coke center: ({ox:.3f}, {oy:.3f}, {oz:.3f})")
        self.get_logger().info(f"  link8 target: ({ox:.3f}, {oy:.3f}, {hover_z:.3f})")
        if not self.send_move_goal([ox, oy, hover_z]):
            self.state = "IDLE"
            return False

        # 3. 从 Planning Scene 移除可乐（避免碰撞检测阻止下降）
        self.state = "REMOVE_COKE_FROM_SCENE"
        self.get_logger().info(f"=== State: {self.state} ===")
        self.remove_coke_from_scene()

        # 4. 下降到抓取高度（指尖对准可乐中心）
        self.state = "DESCEND"
        self.get_logger().info(f"=== State: {self.state} ===")
        self.get_logger().info(f"  link8 target: ({ox:.3f}, {oy:.3f}, {grasp_z:.3f})")
        if not self.send_move_goal([ox, oy, grasp_z]):
            self.state = "MOVE_ABOVE_OBJECT"
            return False

        # 4. 逐步收紧夹爪 + 微抬验证（防滑核心，参考 so101 contact_hold/micro_lift）
        # 不要一次性命令手指到过盈位置（会穿透/夹不紧），而是
        # 收紧 → 双信号判定 → 微抬验证可乐物理抓牢 → 才完整抬起。
        grasped = False
        # 重试时的「当前」目标坐标：首次用感知给出的 object_pose，
        # 每次失败恢复后按可乐实时坐标 + 机械臂当前坐姿刷新（见 recover_grasp）。
        cur_ox, cur_oy = ox, oy
        cur_grasp_z = grasp_z
        cur_hover_z = hover_z
        for attempt in range(self.cfg.grasp.max_grasp_attempts):
            self.get_logger().info(
                f"=== State: CLOSE_GRIPPER (attempt {attempt + 1}/{self.cfg.grasp.max_grasp_attempts}) ==="
            )
            if attempt > 0:
                recovered = self.recover_grasp()
                if recovered is None:
                    self.state = "IDLE"
                    return False
                cur_ox, cur_oy, cur_grasp_z, cur_hover_z = recovered
                time.sleep(self.cfg.grasp.recover_settle)  # 恢复后让可乐在桌面上落定

            coke_ref = self.read_coke_pose()
            if coke_ref is None:
                self.get_logger().error(
                    "[CLOSE_GRIPPER] 读不到可乐初始位姿，无法判定位移"
                )
                self.state = "IDLE"
                return False
            self.get_logger().info(
                f"[CLOSE_GRIPPER] 可乐初始位姿 ref=({coke_ref[0]:.4f}, "
                f"{coke_ref[1]:.4f}, {coke_ref[2]:.4f})"
            )

            self.state = "CLOSE_GRIPPER"
            ok, avg = self.close_gripper_staged(coke_ref)
            if not ok:
                self.get_logger().warn(
                    f"[CLOSE_GRIPPER] 收紧未达标（avg={avg}），准备重试"
                )
                continue

            self.state = "VERIFY_GRASP"
            self.get_logger().info("=== State: VERIFY_GRASP ===")
            if not self._grasp_verified(coke_ref):
                self.get_logger().warn("[VERIFY_GRASP] 双信号未稳定达标，准备重试")
                continue

            self.state = "MICRO_LIFT"
            self.get_logger().info("=== State: MICRO_LIFT ===")
            if not self.verify_micro_lift(cur_ox, cur_oy, cur_grasp_z):
                self.get_logger().warn("[MICRO_LIFT] 微抬验证失败，准备重试")
                continue

            grasped = True
            self.get_logger().info(
                f"[GRASP] 物理抓牢验证通过（attempt {attempt + 1}/{self.cfg.grasp.max_grasp_attempts}）"
            )
            break

        if not grasped:
            self.get_logger().error(
                f"[GRASP] {self.cfg.grasp.max_grasp_attempts} 次尝试均未物理抓牢，放弃"
            )
            # 放弃：张开夹爪，回到悬停位，避免可乐卡在夹爪里
            self.send_gripper_goal(self.cfg.arm.gripper_open_pos, max_effort=0.0)
            self.send_move_goal([cur_ox, cur_oy, cur_hover_z])
            self.state = "IDLE"
            return False

        # 5. 完整抬起（物理抓牢已通过微抬验证）。抬到比 hover 更高，为旋转留空间，
        #    旋转过程中可乐离桌面更高、更不容易碰到桌子/机械臂。
        self.state = "LIFT"
        self.get_logger().info(f"=== State: {self.state} ===")
        lift_z = max(cur_hover_z, self.cfg.grasp.lift_z_min)
        if not self.send_move_goal([cur_ox, cur_oy, lift_z]):
            self.state = "IDLE"
            return False

        # 6. 旋转到放置点上方：保持 lift_z 高度，水平移动到 +Y 方向的放置点。
        #    这一步让 joint1 绕 base 旋转 ~90°，可乐随手臂「转身」到新位置，
        #    旋转动作肉眼清晰可见（演示「抓起来→转身→放到别处」）。
        self.state = "MOVE_TO_PLACE"
        self.get_logger().info(f"=== State: {self.state} ===")
        px, py, pz = self.cfg.object.place_position
        place_grasp_z = pz + self.cfg.arm.finger_tip_offset
        self.get_logger().info(
            f"  place target: ({px:.3f}, {py:.3f}, {pz:.3f}), "
            f"grasp_z={place_grasp_z:.3f}"
        )
        if not self.send_move_goal([px, py, lift_z]):
            self.state = "IDLE"
            return False

        # 7. 下降到放置高度（可乐底部贴桌面）
        self.state = "DESCEND_TO_PLACE"
        self.get_logger().info(f"=== State: {self.state} ===")
        if not self.send_move_goal([px, py, place_grasp_z]):
            self.state = "IDLE"
            return False

        # 8. 张开夹爪，可乐落到桌面
        self.state = "RELEASE_GRIPPER"
        self.get_logger().info(f"=== State: {self.state} ===")
        if not self.send_gripper_goal(self.cfg.arm.gripper_open_pos, max_effort=0.0):
            self.state = "IDLE"
            return False
        time.sleep(self.cfg.grasp.release_settle)   # 让可乐在桌面上落定

        # 9. 抬起夹爪离开（避免扫倒可乐）
        self.state = "MOVE_AWAY"
        self.get_logger().info(f"=== State: {self.state} ===")
        if not self.send_move_goal([px, py, lift_z]):
            self.state = "IDLE"
            return False

        # 10. 复位 arm 到 home 关节位形。必须用关节空间约束（send_joint_goal），
        #     不能用笛卡尔 position constraint（旧版 [0.3,0,0.4]）：7-DOF 冗余臂的
        #     笛卡尔解不唯一，IK 会把 panda_joint2 拧到极限 -1.83299（越界），下一轮
        #     MOVE_ABOVE 起点即 CheckStartStateBounds 拒 → START_STATE_INVALID。
        #     关节约束把每个关节钉回 home 值，保证下一轮抓取起点干净、不越界。
        self.state = "RETURN_HOME"
        self.get_logger().info(f"=== State: {self.state} ===")
        if not self.send_joint_goal(self.cfg.arm.home_joint_values):
            self.get_logger().error("RETURN_HOME 复位失败，arm 未回到 home，下一轮可能越界")
            self.state = "IDLE"
            return False

        self.state = "DONE"
        self.get_logger().info(
            f"=== State: {self.state} === Pick-place cycle complete!"
        )
        self.state = "IDLE"
        return True


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = PickPlaceStateMachine()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.get_logger().info("Pick-place state machine ready. Waiting for commands...")
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
