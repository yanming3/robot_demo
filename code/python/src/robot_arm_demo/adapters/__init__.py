"""core.interfaces 中 Protocol 的具体后端实现（ROS 耦合层）。

每个适配器实现一个接口：moveit_arm(MoveItArm→ArmController)、
gripper_action(GripperAction→Gripper)、mujoco_free_joint(→ObjectPoseSource)、
detectors(ColorDetector/QwenVlDetector→ObjectDetector)。
core 与 pytest 永远不 import 本包；仅 demo 入口装配时引用。
"""
