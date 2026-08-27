#!/usr/bin/env python3
"""V5-T003: LLM Planner 节点。

读取用户文本指令 -> 调用 DeepSeek API 解析为 JSON -> 发布到 /llm_command
感知节点订阅 /llm_command，做 VLM 检测和坐标转换后发布 /robot_command 给状态机。

环境变量:
    DEEPSEEK_API_KEY  DeepSeek API 密钥（必需）
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import (
    APIConnectionError,
    APITimeoutError,
    InternalServerError,
    RateLimitError,
    OpenAI,
)

# DeepSeek 瞬时故障（断连/超时/限流/5xx）的自动重试；
# 重试耗尽仅报错不退出 —— 演示中网络抖动不应杀死整个节点进程。
LLM_MAX_RETRIES = 3
LLM_RETRY_INTERVAL = 2.0   # s

SYSTEM_PROMPT = """你是机器人指令解析器。把用户指令转成 JSON。
action 只能是 "pick"（抓取）或 "place"（放置）。
格式：{"target_object": string, "action": "pick"|"place", "destination": string|null, "constraints": string[]}
例：帮我拿可乐 → {"target_object": "可乐", "action": "pick", "destination": null, "constraints": []}
"""

# 可重试的瞬时错误类型
_TRANSIENT_ERRORS = (APIConnectionError, APITimeoutError, InternalServerError, RateLimitError)


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__("llm_planner")
        api_key = os.environ.get("DEEPSEEK_API_KEY")
        if not api_key:
            self.get_logger().error("DEEPSEEK_API_KEY not set, exiting.")
            raise SystemExit(1)
        self.client = OpenAI(api_key=api_key, base_url="https://api.deepseek.com")
        self.command_pub = self.create_publisher(String, "/llm_command", 10)
        self.get_logger().info("LLM Planner ready. Publishing to /llm_command. Type a command and press Enter.")

    def parse_and_publish(self, user_text: str) -> dict:
        """调 DeepSeek 把自然语言解析为 JSON 指令，发布到 /llm_command。

        瞬时错误自动重试至 LLM_MAX_RETRIES 次；重试耗尽返回 {}（不发布）。
        """
        raw = None
        for attempt in range(1, LLM_MAX_RETRIES + 1):
            try:
                response = self.client.chat.completions.create(
                    model="deepseek-chat",
                    messages=[
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": user_text},
                    ],
                    response_format={"type": "json_object"},
                )
                raw = response.choices[0].message.content
                break
            except _TRANSIENT_ERRORS as e:
                self.get_logger().warn(
                    f"LLM API transient error (attempt {attempt}/{LLM_MAX_RETRIES}): {e}"
                )
                if attempt < LLM_MAX_RETRIES:
                    time.sleep(LLM_RETRY_INTERVAL)
            except Exception as e:
                # 不可恢复错误（鉴权/参数等）：报错并放弃本次指令
                self.get_logger().error(f"LLM API call failed, giving up: {e}")
                return {}

        if raw is None:
            self.get_logger().error(
                f"LLM unreachable after {LLM_MAX_RETRIES} retries, command dropped."
            )
            return {}

        self.get_logger().info(f"LLM response: {raw}")
        try:
            cmd = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().error("LLM returned invalid JSON, not publishing.")
            return {}
        msg = String()
        msg.data = json.dumps(cmd, ensure_ascii=False)
        self.command_pub.publish(msg)
        self.get_logger().info(f"Published to /llm_command: {msg.data}")
        return cmd


def main():
    rclpy.init()
    node = LLMPlannerNode()
    try:
        while rclpy.ok():
            try:
                user_text = input("\n指令> ").strip()
            except EOFError:
                break
            if not user_text:
                continue
            try:
                node.parse_and_publish(user_text)
            except Exception as e:
                # 兜底：任何未预期异常只报错，保持节点存活等待下一条指令
                node.get_logger().error(f"Unexpected error handling command: {e}")
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
