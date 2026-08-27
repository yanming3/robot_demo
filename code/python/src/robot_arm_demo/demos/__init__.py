"""各机械臂 demo 的入口：每 demo 一个子目录（config.py + thin 启动节点）。

新增一个 mujoco 机械臂 demo = 新建 demos/<name>_mujoco/（config.py 填入该臂的
ArmConfig/ObjectConfig 数值 + 薄节点）+ 一个 ros2_ws/src/<name>_demo 配置包 +
一个 start 脚本副本。core/ 与 adapters/ 零改动。
"""
