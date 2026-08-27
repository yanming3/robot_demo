# Python 机械臂 Demo

一个用 uv 管理的二维两连杆机械臂学习 demo。

## 运行

```bash
uv run robot-arm-demo --target 1.2 0.8 --steps 8
```

也可以直接运行模块：

```bash
uv run python -m robot_arm_demo.demo --target 1.2 0.8 --steps 8
```

导出 CSV：

```bash
uv run robot-arm-demo --target 1.2 0.8 --steps 8 --output trajectory.csv
```

## 测试

```bash
uv run pytest
```
