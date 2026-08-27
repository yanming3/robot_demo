"""/llm_command 与 /robot_command 的 JSON 契约解析（纯函数）。"""

from __future__ import annotations

import json

from .data import TaskCommand


def parse_task_command(text: str) -> TaskCommand | None:
    """解析 /robot_command 的 JSON → TaskCommand。

    - 非法 JSON 返回 None（调用方负责告警日志，行为与旧节点一致）；
    - destination/constraints 解析保留但不消费（维持现状）；
    - action 缺失/非 pick、position 缺失不抛错，由 TaskCommand.supported
      为 False 表达（对应旧的 warn+stay IDLE 分支）。
    """
    try:
        cmd = json.loads(text)
    except json.JSONDecodeError:
        return None
    position = cmd.get("position")
    return TaskCommand(
        target_object=cmd.get("target_object"),
        action=cmd.get("action"),
        position=tuple(position) if position is not None else None,
        destination=cmd.get("destination"),
        constraints=tuple(cmd.get("constraints") or ()),
    )
