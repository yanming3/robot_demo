"""机器人无关的 pick-place 状态机控制器。

逻辑自 pick_place_state_machine.py 原样搬入；所有 ROS 调用经
core.interfaces 的 Protocol 注入（ArmController/Gripper/ObjectPoseSource），
日志走 Logger。状态字符串、日志文案、时序与阈值语义逐字保留。
"""

from __future__ import annotations

import math
import threading
import time

from .data import PickPlaceConfig, TaskCommand
from .interfaces import ArmController, Gripper, Logger, ObjectPoseSource


def _wait_future(future, timeout=30.0) -> bool:
    """轮询等待 future 完成，不阻塞 executor。"""
    start = time.time()
    while not future.done() and time.time() - start < timeout:
        time.sleep(0.01)
    return future.done()


class PickPlaceController:
    def __init__(
        self,
        *,
        arm: ArmController,
        gripper: Gripper,
        pose_source: ObjectPoseSource,
        config: PickPlaceConfig,
        logger: Logger,
    ):
        self.arm = arm
        self.gripper = gripper
        self.pose_source = pose_source
        self.cfg = config
        self.log = logger
        self.state = "IDLE"
        self.object_pose = [0.5, 0.0, 0.3]
        self._busy = False

    # ── 指令入口 ──

    def handle_command(self, task: TaskCommand) -> None:
        """busy/合法性检查后在工作线程执行抓放循环。

        分支与日志与旧 command_callback 逐条对应。
        """
        if self._busy:
            self.log.warn("Busy, ignoring command.")
            return
        if task.action != "pick":
            self.log.warn(f"Unsupported action: {task.action}, staying IDLE.")
            return
        if task.position is None:
            self.log.warn("No position in command, staying IDLE.")
            return

        self._busy = True
        self.object_pose = list(task.position)
        thread = threading.Thread(target=self._execute_pick_place, daemon=True)
        thread.start()

    def _execute_pick_place(self):
        """在单独线程里执行 pick-place，避免阻塞 executor。"""
        success = self.run()
        self._busy = False
        if success:
            self.log.info("Pick-place completed successfully.")
        else:
            self.log.error("Pick-place failed.")

    # ── 抓取判定辅助 ──

    def read_object_pose(self):
        """读目标物体最新 world 位姿 (x, y, z)，未收到返回 None。"""
        return self.pose_source.get_object_pose(self.cfg.object.object_id)

    def read_finger_positions(self):
        """读各手指真实位置 dict；来自夹爪适配器。"""
        return self.gripper.read_finger_positions()

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

    def _object_displacement_from(self, ref):
        """物体当前 world 位姿相对 ref 的欧氏位移（m）；读不到返回 None。"""
        pose = self.read_object_pose()
        if pose is None or ref is None:
            return None
        dx = pose[0] - ref[0]
        dy = pose[1] - ref[1]
        dz = pose[2] - ref[2]
        return math.hypot(math.hypot(dx, dy), dz)

    def _grasp_verified(self, object_ref):
        """双信号判定：两指 qpos 落在接触窗口且位移小于容差，连续两次达标。"""
        ok_count = 0
        for sample in range(self.cfg.grasp.stable_samples):
            avg = self._read_finger_avg()
            disp = self._object_displacement_from(object_ref)
            if avg is None or disp is None:
                self.log.warn(
                    f"[VERIFY_GRASP] 信号缺失 finger_avg={avg} coke_disp={disp}"
                )
                return False
            qpos_ok = (
                self.cfg.grasp.contact_qpos_min
                <= avg
                <= self.cfg.grasp.contact_qpos_max
            )
            disp_ok = disp < self.cfg.grasp.displacement_tolerance
            self.log.info(
                f"[VERIFY_GRASP] sample={sample + 1} finger_avg={avg:.4f} "
                f"coke_disp={disp:.4f} qpos_ok={qpos_ok} disp_ok={disp_ok}"
            )
            if qpos_ok and disp_ok:
                ok_count += 1
            if sample < self.cfg.grasp.stable_samples - 1:
                time.sleep(self.cfg.grasp.stable_interval)
        return ok_count == self.cfg.grasp.stable_samples

    def _step_gripper(self, target, object_ref, phase):
        """单步收紧并检查：物体不被推走、手指不穿透。返回是否可继续。"""
        self.gripper.move_to(target, max_effort=self.cfg.grasp.close_max_effort)
        time.sleep(self.cfg.grasp.step_settle)
        avg = self._read_finger_avg()
        disp = self._object_displacement_from(object_ref)
        avg_str = f"{avg:.4f}" if avg is not None else "None"
        disp_str = f"{disp:.4f}" if disp is not None else "None"
        self.log.info(
            f"[CLOSE_GRIPPER] {phase} target={target:.4f} "
            f"finger_avg={avg_str} coke_disp={disp_str}"
        )
        if avg is None:
            self.log.error("[CLOSE_GRIPPER] 读不到手指 qpos，中止收紧")
            return False
        if avg < self.cfg.grasp.min_qpos:
            self.log.warn(
                f"[CLOSE_GRIPPER] 手指 qpos={avg:.4f} 低于安全下限 "
                f"{self.cfg.grasp.min_qpos}，疑似穿透，中止"
            )
            return False
        if disp is None or disp >= self.cfg.grasp.displacement_tolerance:
            self.log.warn(
                f"[CLOSE_GRIPPER] 可乐被推动 disp={disp_str}，中止（推太狠）"
            )
            return False
        return True

    def close_gripper_staged(self, object_ref):
        """两段式逐步收紧夹爪，返回 (是否夹住, 最终平均 qpos)。

        阶段1 粗调：大步逼近到手指 qpos 进入接触区上沿，
        阶段2 精调：小步收紧，每步双信号判定，连续达标即成功。
        全程每步检查物体不被推走、手指不穿透。
        """
        g = self.cfg.grasp
        target = self.cfg.arm.gripper_open_pos
        final_avg = None

        self.log.info(
            f"[CLOSE_GRIPPER] 阶段1 粗调开始（步长 {g.coarse_step * 1000:.0f}mm，"
            f"起点 {self.cfg.arm.gripper_open_pos}）"
        )
        while target > g.min_qpos:
            target = max(g.min_qpos, target - g.coarse_step)
            if not self._step_gripper(target, object_ref, "coarse"):
                return False, self._read_finger_avg()
            final_avg = self._read_finger_avg()
            # 手指 qpos 进入接触区上沿（物体表面附近），转精调
            if final_avg is not None and final_avg <= g.contact_qpos_max + 0.001:
                self.log.info(
                    f"[CLOSE_GRIPPER] 粗调到达接触区（avg={final_avg:.4f}），转精调"
                )
                break
        else:
            self.log.warn(
                "[CLOSE_GRIPPER] 粗调收到安全下限仍未进入接触区（可能空闭合/穿透）"
            )
            return False, final_avg

        self.log.info(
            f"[CLOSE_GRIPPER] 阶段2 精调开始（步长 {g.fine_step * 1000:.0f}mm）"
        )
        while target > g.min_qpos:
            target = max(g.min_qpos, target - g.fine_step)
            if not self._step_gripper(target, object_ref, "fine"):
                return False, self._read_finger_avg()
            if self._grasp_verified(object_ref):
                final_avg = self._read_finger_avg()
                self.log.info(
                    f"[CLOSE_GRIPPER] 双信号达标，夹紧成功（avg={final_avg:.4f}）"
                )
                return True, final_avg
        self.log.warn(
            "[CLOSE_GRIPPER] 精调收到安全下限仍未双信号达标"
        )
        return False, self._read_finger_avg()

    def verify_micro_lift(self, ox, oy, grasp_z):
        """慢速微抬，验证物体 z 跟着抬升（物理抓牢）。"""
        g = self.cfg.grasp
        before = self.read_object_pose()
        if before is None:
            self.log.error("[MICRO_LIFT] 微抬前读不到可乐位姿")
            return False
        target_z = grasp_z + g.micro_lift_height
        vsc = g.micro_lift_velocity_scaling
        self.log.info(
            f"[MICRO_LIFT] 慢速微抬到 z={target_z:.4f} "
            f"（抬 {g.micro_lift_height * 100:.1f}cm，velocity_scaling={vsc}）"
        )
        if not self.arm.move_cartesian([ox, oy, target_z], velocity_scaling=vsc):
            self.log.error("[MICRO_LIFT] 微抬移动失败")
            return False
        after = self.read_object_pose()
        if after is None:
            self.log.error("[MICRO_LIFT] 微抬后读不到可乐位姿")
            return False
        dz = after[2] - before[2]
        lateral = math.hypot(after[0] - before[0], after[1] - before[1])
        self.log.info(
            f"[MICRO_LIFT] 可乐位移 dz={dz:.4f} lateral={lateral:.4f} "
            f"（需 dz≥{g.micro_lift_z_min} 且 lateral<{g.micro_lift_lateral_max}）"
        )
        if dz < g.micro_lift_z_min:
            self.log.warn("[MICRO_LIFT] 可乐 z 未跟着抬升 → 未抓牢")
            return False
        if lateral >= g.micro_lift_lateral_max:
            self.log.warn("[MICRO_LIFT] 可乐侧向漂移过大 → 可能滑脱")
            return False
        self.log.info("[MICRO_LIFT] 微抬验证通过，可乐物理抓牢")
        return True

    def recover_grasp(self):
        """失败恢复：张开夹爪 → 读物体当前坐标 → 重新对准下降。

        重试必须基于物体「当前」坐标和机械臂「当前」坐姿，不能复用初始
        object_pose。返回 (cx, cy, grasp_z, hover_z)；读不到位姿返回 None。
        """
        arm = self.cfg.arm
        g = self.cfg.grasp
        self.log.warn("[RECOVER] 张开夹爪并重新对准可乐当前位置重试")
        self.gripper.move_to(arm.gripper_open_pos, max_effort=g.open_max_effort)
        time.sleep(g.recover_settle)
        cur = self.read_object_pose()
        if cur is None:
            self.log.error("[RECOVER] 读不到可乐当前位置，无法重新对准")
            return None
        cx, cy, cz = cur
        new_grasp_z = max(cz + arm.finger_tip_offset, arm.min_eef_z)
        new_hover_z = new_grasp_z + g.hover_offset
        self.log.info(
            f"[RECOVER] 可乐当前 ({cx:.4f}, {cy:.4f}, {cz:.4f}) "
            f"→ grasp_z={new_grasp_z:.4f}"
        )
        # 从机械臂当前坐姿直接规划到物体当前坐标上方（悬停），再下降
        if not self.arm.move_cartesian([cx, cy, new_hover_z]):
            self.log.error("[RECOVER] 回悬停位失败")
            return None
        if not self.arm.move_cartesian([cx, cy, new_grasp_z]):
            self.log.error("[RECOVER] 重新下降到抓取位失败")
            return None
        return cx, cy, new_grasp_z, new_hover_z

    # ── 主流程 ──

    def run(self):
        """执行 pick-place 流程。

        感知节点给出的 object_pose 是目标中心在 base frame 下的位置；
        eef 目标 z = 目标中心 + finger_tip_offset（钳制到 min_eef_z）。
        """
        cfg = self.cfg
        ox, oy, oz = self.object_pose

        # 物体中心高度处夹持：eef_link 目标 = 中心 + 指尖偏移
        grasp_z = oz + cfg.arm.finger_tip_offset
        # 安全下限：过降会以掌心压倒目标物（历史 bug 见配置注释）
        if grasp_z < cfg.arm.min_eef_z:
            self.log.warn(
                f"grasp_z={grasp_z:.3f} below min {cfg.arm.min_eef_z}, clamping"
            )
            grasp_z = cfg.arm.min_eef_z
        hover_z = grasp_z + cfg.grasp.hover_offset   # 悬停在抓取点上方 12cm

        # 1. 张开夹爪
        self.state = "OPEN_GRIPPER"
        self.log.info(f"=== State: {self.state} ===")
        if not self.gripper.move_to(
            cfg.arm.gripper_open_pos, max_effort=cfg.grasp.open_max_effort
        ):
            self.state = "IDLE"
            return False

        # 2. 移动到目标正上方（悬停）
        self.state = "MOVE_ABOVE_OBJECT"
        self.log.info(f"=== State: {self.state} ===")
        self.log.info(f"  coke center: ({ox:.3f}, {oy:.3f}, {oz:.3f})")
        self.log.info(f"  link8 target: ({ox:.3f}, {oy:.3f}, {hover_z:.3f})")
        if not self.arm.move_cartesian([ox, oy, hover_z]):
            self.state = "IDLE"
            return False

        # 3. 从 Planning Scene 移除目标（避免碰撞检测阻止下降）
        self.state = "REMOVE_COKE_FROM_SCENE"
        self.log.info(f"=== State: {self.state} ===")
        self.arm.remove_object_from_scene(cfg.object.object_id)

        # 4. 下降到抓取高度（指尖对准物体中心）
        self.state = "DESCEND"
        self.log.info(f"=== State: {self.state} ===")
        self.log.info(f"  link8 target: ({ox:.3f}, {oy:.3f}, {grasp_z:.3f})")
        if not self.arm.move_cartesian([ox, oy, grasp_z]):
            self.state = "MOVE_ABOVE_OBJECT"
            return False

        # 4. 逐步收紧夹爪 + 微抬验证（防滑核心，参考 so101 contact_hold/micro_lift）
        grasped = False
        cur_ox, cur_oy = ox, oy
        cur_grasp_z = grasp_z
        cur_hover_z = hover_z
        for attempt in range(cfg.grasp.max_grasp_attempts):
            self.log.info(
                f"=== State: CLOSE_GRIPPER "
                f"(attempt {attempt + 1}/{cfg.grasp.max_grasp_attempts}) ==="
            )
            if attempt > 0:
                recovered = self.recover_grasp()
                if recovered is None:
                    self.state = "IDLE"
                    return False
                cur_ox, cur_oy, cur_grasp_z, cur_hover_z = recovered
                time.sleep(cfg.grasp.recover_settle)  # 恢复后让可乐在桌面上落定

            coke_ref = self.read_object_pose()
            if coke_ref is None:
                self.log.error(
                    "[CLOSE_GRIPPER] 读不到可乐初始位姿，无法判定位移"
                )
                self.state = "IDLE"
                return False
            self.log.info(
                f"[CLOSE_GRIPPER] 可乐初始位姿 ref=({coke_ref[0]:.4f}, "
                f"{coke_ref[1]:.4f}, {coke_ref[2]:.4f})"
            )

            self.state = "CLOSE_GRIPPER"
            ok, avg = self.close_gripper_staged(coke_ref)
            if not ok:
                self.log.warn(
                    f"[CLOSE_GRIPPER] 收紧未达标（avg={avg}），准备重试"
                )
                continue

            self.state = "VERIFY_GRASP"
            self.log.info("=== State: VERIFY_GRASP ===")
            if not self._grasp_verified(coke_ref):
                self.log.warn("[VERIFY_GRASP] 双信号未稳定达标，准备重试")
                continue

            self.state = "MICRO_LIFT"
            self.log.info("=== State: MICRO_LIFT ===")
            if not self.verify_micro_lift(cur_ox, cur_oy, cur_grasp_z):
                self.log.warn("[MICRO_LIFT] 微抬验证失败，准备重试")
                continue

            grasped = True
            self.log.info(
                f"[GRASP] 物理抓牢验证通过（attempt {attempt + 1}/"
                f"{cfg.grasp.max_grasp_attempts}）"
            )
            break

        if not grasped:
            self.log.error(
                f"[GRASP] {cfg.grasp.max_grasp_attempts} 次尝试均未物理抓牢，放弃"
            )
            # 放弃：张开夹爪，回到悬停位，避免可乐卡在夹爪里
            self.gripper.move_to(
                cfg.arm.gripper_open_pos, max_effort=cfg.grasp.open_max_effort
            )
            self.arm.move_cartesian([cur_ox, cur_oy, cur_hover_z])
            self.state = "IDLE"
            return False

        # 5. 完整抬起（物理抓牢已通过微抬验证）。抬到比 hover 更高，为旋转留空间
        self.state = "LIFT"
        self.log.info(f"=== State: {self.state} ===")
        lift_z = max(cur_hover_z, cfg.grasp.lift_z_min)
        if not self.arm.move_cartesian([cur_ox, cur_oy, lift_z]):
            self.state = "IDLE"
            return False

        # 6. 水平移动到放置点上方（保持 lift_z 高度）
        self.state = "MOVE_TO_PLACE"
        self.log.info(f"=== State: {self.state} ===")
        px, py, pz = cfg.object.place_position
        place_grasp_z = pz + cfg.arm.finger_tip_offset
        self.log.info(
            f"  place target: ({px:.3f}, {py:.3f}, {pz:.3f}), "
            f"grasp_z={place_grasp_z:.3f}"
        )
        if not self.arm.move_cartesian([px, py, lift_z]):
            self.state = "IDLE"
            return False

        # 7. 下降到放置高度（物体底部贴桌面）
        self.state = "DESCEND_TO_PLACE"
        self.log.info(f"=== State: {self.state} ===")
        if not self.arm.move_cartesian([px, py, place_grasp_z]):
            self.state = "IDLE"
            return False

        # 8. 张开夹爪，物体落到桌面
        self.state = "RELEASE_GRIPPER"
        self.log.info(f"=== State: {self.state} ===")
        if not self.gripper.move_to(
            cfg.arm.gripper_open_pos, max_effort=cfg.grasp.open_max_effort
        ):
            self.state = "IDLE"
            return False
        time.sleep(cfg.grasp.release_settle)   # 让可乐在桌面上落定

        # 9. 抬起夹爪离开（避免扫倒物体）
        self.state = "MOVE_AWAY"
        self.log.info(f"=== State: {self.state} ===")
        if not self.arm.move_cartesian([px, py, lift_z]):
            self.state = "IDLE"
            return False

        # 10. 复位到 home 关节位形（必须关节空间约束——笛卡尔解对 7-DOF 不唯一，
        #     会把 wrist 拧越界导致下一轮 START_STATE_INVALID）
        self.state = "RETURN_HOME"
        self.log.info(f"=== State: {self.state} ===")
        if not self.arm.move_to_joint(cfg.arm.home_joint_values):
            self.log.error("RETURN_HOME 复位失败，arm 未回到 home，下一轮可能越界")
            self.state = "IDLE"
            return False

        self.state = "DONE"
        self.log.info(
            f"=== State: {self.state} === Pick-place cycle complete!"
        )
        self.state = "IDLE"
        return True
