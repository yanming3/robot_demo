"""TaskCommand 解析契约测试（/robot_command JSON）。"""

from robot_arm_demo.core.command import parse_task_command


def test_valid_pick_command():
    task = parse_task_command(
        '{"target_object":"可乐","action":"pick","position":[0.3,0.0,0.061]}'
    )
    assert task is not None
    assert task.target_object == "可乐"
    assert task.action == "pick"
    assert task.position == (0.3, 0.0, 0.061)
    assert task.supported


def test_invalid_json_returns_none():
    assert parse_task_command("not json {") is None


def test_unsupported_action_not_supported():
    task = parse_task_command('{"target_object":"x","action":"place","position":[1,2,3]}')
    assert task is not None
    assert not task.supported


def test_missing_position_not_supported():
    task = parse_task_command('{"target_object":"可乐","action":"pick"}')
    assert task is not None
    assert not task.supported


def test_destination_constraints_roundtrip():
    """schema 里的 destination/constraints 解析保留但下游不消费。"""
    task = parse_task_command(
        '{"target_object":"x","action":"pick","position":[1,1,1],'
        '"destination":"bin_a","constraints":["upright"]}'
    )
    assert task.destination == "bin_a"
    assert task.constraints == ("upright",)


def test_orientation_roundtrip():
    """云端 6D 链路：position + orientation(xyzw)。"""
    task = parse_task_command(
        '{"target_object":"可乐","action":"pick",'
        '"position":[0.3,-0.05,0.3],'
        '"orientation":[0.0,0.0,0.0,1.0]}'
    )
    assert task is not None
    assert task.position == (0.3, -0.05, 0.3)
    assert task.orientation == (0.0, 0.0, 0.0, 1.0)
    assert task.supported


def test_grasp_pose_roundtrip():
    task = parse_task_command(
        '{"target_object":"可乐","action":"pick",'
        '"position":[0.3,-0.05,0.3],'
        '"grasp_pose":[0.3,-0.05,0.3,0.0,0.0,0.0,1.0]}'
    )
    assert task is not None
    assert task.grasp_pose == (0.3, -0.05, 0.3, 0.0, 0.0, 0.0, 1.0)
    assert task.orientation is None  # 未给 orientation 时保持 None


def test_position_only_still_supported():
    """无 orientation 的旧 payload 仍受支持（不破坏向后兼容）。"""
    task = parse_task_command(
        '{"target_object":"可乐","action":"pick","position":[0.3,0.0,0.061]}'
    )
    assert task is not None
    assert task.orientation is None
    assert task.supported
