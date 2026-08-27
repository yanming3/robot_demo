"""机器人无关的核心逻辑。

本包只依赖标准库（dataclasses/math/json/threading/typing），绝不 import
rclpy / ROS 消息 / openai —— 这保证 `uv run pytest` 无需 ROS 即可验证。
与 ROS 的所有交互通过 core.interfaces 中的 Protocol 抽象，由 adapters/
包提供具体实现。
"""
