# 感知位姿云服务 — API 契约 (v1)

> 本文档把「本地 ROS2 节点 ↔ 云端推理服务」的接口**一次性定死**，让云端与本地能各自独立实现、并用 mock 打通。
> 目标：解决 `PerceptionNode` 的两个问题（① 始终假设可乐垂直；② 用点云/单像素反投影算中心，无姿态），
> 面向**真实机械臂抓取可乐**（瓶子**可能倾斜/躺倒**），控制侧一并支持**姿态感知抓取**。

- 状态：**草案 v1**（可随实现微调，但字段/坐标系/单位是硬约定，改动需走版本号）
- 阶段：Stage 1 使用「分割(mask) + 几何位姿」；Stage 3 将云端位姿实现换成 FoundationPose，**本地契约不变**。
- 暂缓项（TBD，标注处）：FoundationPose 物体参考（mesh/图）、物体系抓取点、夹持时位姿重估替代 GT。

---

## 1. 目标架构与数据流

```
[本地 sim / 真机]
  /camera        (sensor_msgs/Image, RGB)
  /camera/depth  (sensor_msgs/Image, 32FC1 米)
  /camera/camera_info  (CameraInfo → 内参)

perception_node (本地, rclpy)
  ① 采集 RGB + 对齐深度 + 内参        ← 修复深度/tf/RGB 错位(见 §6.4)
  ② 组 HTTP 请求 POST /v1/estimate_pose
  ③ 收 6D pose (camera 系) + mask + axis
  ④ tf2 变换「完整位姿」camera→base（PoseStamped，不是 PointStamped）
  ⑤ 由物体位姿 + 物体系抓取点 → 世界抓取点/接近轴（§6.5, Stage 2 项）
  ⑥ 发布 /robot_command { position, orientation(xyzw), grasp_pose }

pick_place_state_machine (本地)
  PickPlaceController: 用四元数调 move_cartesian + 预抓取点 + 沿瓶轴接近
```

**边界**：云端只做「分割 + 位姿估计」，**不知道**机器人/base/抓取。一切与机械臂/坐标系推导相关（tf、抓取点、sanity box、接近轴）都在本地。

---

## 2. 传输与端点约定

- **协议**：HTTP/1.1 + JSON (`application/json`)。图像帧编码进请求体（见 §4）。
- **推荐框架**：云端用 FastAPI（或任意能建同契约的），本地用 `requests`/`httpx`。
- **Base URL**：`POST {BASE_URL}/v1/estimate_pose`。`BASE_URL` 由本地配置（如 `http://<host>:8000`）。
- **健康检查**：`GET /health` → `{"status":"ok","models":{...},"elapsed_ms":...}`。本地启动/重试前探测就绪。
- **超时与重试**：本地默认 `timeout=15s`，失败重试 `2` 次（指数退避），详见 §8。

---

## 3. 坐标系与单位（**硬约定**）

| 项 | 约定 | 说明 |
|---|---|---|
| 位置单位 | **米 (m)**，float32 | |
| 旋转 | **单位四元数 `[x, y, z, w]`** | **xyzw 顺序**，右手系 |
| 位姿参考系 | **相机光心光学坐标系**（内参 fx/fy/cx/cy 所在的那个坐标系） | 请求里用 `camera_frame_id` 声明，云端默认认为是**光学系**（X 右、Y 下、Z 前，内参由此系给出） |
| 深度单位 | **米**，`NaN`/`Inf` 表示无效 | 与内参同帧对齐 |
| 图像 | RGB 8-bit（0-255），深度 32-bit float（米） | |
| axis 输出 | 瓶轴（圆柱旋转轴）方向，**单位向量**，camera 系 | 供 Stage 1 几何位姿与 Stage 3 校验用 |

> 关于 ROS 光学帧与 `camera_link` 的差异：ROS 中 `camera_link`（link 系）与 `camera_optical_frame`（光学系）通常差一个固定旋转。若你发送的是光学系内参，则 `camera_frame_id` 应填光学系 id；返回位姿也在此系。**到底用哪个系必须在本地与云端一致**（推荐统一用光学系），否则 6D 会错位。详见 §6.3。

---

## 4. 请求 Schema

`POST /v1/estimate_pose`，`Content-Type: application/json`

```jsonc
{
  "version": "v1",

  // ---- 图像 ----
  "image": {
    "encoding": "rgb8",              // 固定 rgb8
    "width": 640,
    "height": 480,
    "data_b64": "<base64 of row-major RGB bytes>"   // 长度 = w*h*3
  },
  "depth": {
    "encoding": "32FC1",             // float32, 米
    "width": 640,
    "height": 480,
    "data_b64": "<base64 of little-endian float32, row-major>"
  },

  // ---- 内参（与 image 同系）----
  "camera_intrinsics": {
    "fx": 554.0, "fy": 554.0, "cx": 320.0, "cy": 240.0,
    "width": 640, "height": 480,
    "distortion_k": [0.0, 0.0, 0.0, 0.0, 0.0]   // k1,k2,p1,p2,k3；若无畸变填 0
  },
  "camera_frame_id": "camera_link",   // 声明内参/位姿所在系（见 §3）

  // ---- 目标与提示 ----
  "prompt": "coke bottle",            // 开放词表提示（Grounding-DINO 用）
  "object_id": "coke",                // 本地语义 id（日志、匹配用）

  // ---- 可选控制 ----
  "return_mask": true,                // 是否回传 mask（调试/校验用）
  "max_detections": 1,                // 只取置信度最高目标
  "reference_object_id": null
}
```

约束：
- `image.data_b64` 解码后长度必须等于 `width*height*3`（否则 400）。
- `depth.*` 长度 = `width*height*4`（float32）。
- `depth` 与 `image` 的 `width/height` 必须一致（否则 400）。
- 若云端/真机深度读不齐，**最少必须提供 `image` + `camera_intrinsics`**；缺深度时云端可仅用 RGB 做分割，并在此需求下用可选几何位姿（缺深度可能降级，见 §7 错误码 `2101`）。

---

## 5. 响应 Schema

`200 OK`：

```jsonc
{
  "version": "v1",
  "success": true,
  "elapsed_ms": 423,
  "object_id": "coke",

  "pose": {
    "position": [0.3, -0.05, 0.60],       // 米, camera 系
    "orientation_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],  // 单位四元数, xyzw
    "confidence": 0.93
  },

  // 额外信息
  "axis": [0.0, 0.0, 1.0],                // 瓶轴方向（camera 系, 单位向量）；几何位姿/校验用
  "mask": null,                            // return_mask=true 时给 base64 PNG（单通道, 灰度）
  "segmentation": {
    "method": "grounding_dino_sam2",       // "yolo11_seg" | "grounding_dino_sam2" | "geometry"
    "mask_confidence": 0.97
  },

  "warnings": []                           // 非致命提示数组
}
```

---

## 6. 错误模型

统一响应包裹，HTTP 状态码只在「服务不可达/输入非法」层面用，业务失败用 `success=false` + 业务错误码。

```jsonc
{
  "version": "v1",
  "success": false,
  "elapsed_ms": 12,
  "error": { "code": "1501", "message": "depth not registered", "detail": "..." }
}
```

| HTTP | 业务 code | 含义 | 本地处理 |
|---|---|---|---|
| 400 | `1001` | 请求 JSON 非法 / 字段缺失 | 记 error，丢弃本次 |
| 400 | `1002` | 图像长度与 width/height 不符 | 记 error |
| 400 | `1003` | image/depth 尺寸不一致 | 记 error |
| 400 | `1004` | 内参非法（fx/fy≤0, cx/cy 越界） | 记 error |
| 404 | `1005` | `camera_frame_id` 未知 | 记 error |
| 422 | `1500` | 未检测到目标（mask 为空） | 本地重试一次 / 走兜底（见 §7） |
| 422 | `1501` | 检测到但无有效 3D（深度缺） | 本地返回失败，不再走 assumed_depth |
| 500 | `1502` | 模型/服务内部错误 | 重试 2 次（退避），仍失败则报错 |
| 503 | `1503` | 模型未就绪（未 warm，见 §8.1） | 本地先 `GET /health` 探测 |

> **注意**：本地**不再回退到 `assumed_depth` 的 2D 反投影**（那是旧 `perception_node.py` 的问题根源）。检测到目标但拿不到 3D 时，就显式失败，这是行为改变，也是去掉「固定深度」的正确做法。

---

## 7. 本地侧消费映射（改造哪些代码）

| 现有代码 | 改动 |
|---|---|
| `code/python/src/robot_arm_demo/demos/panda_mujoco/perception_node.py` | 已改：去 `ColorDetector`/`QwenVlDetector`/`_backproject`/`QwenVlDetector`，改走云端 + 本地几何兜底；采 RGBD+`camera_info`→组请求→收位姿→tf2 完整位姿→发布 `{position, orientation}`。 |
| `code/python/src/robot_arm_demo/adapters/logger.py` `Tf2PointTransform` | 现有只变换 `PointStamped`；需新增/扩展为**变换完整 pose**（`PoseStamped`）或在节点内用 `tf2_geometry_msgs` 直接做 `transform_pose`。 |
| `code/python/src/robot_arm_demo/core/data.py` `TaskCommand` | 加 `orientation: tuple[float,float,float,float] \| None`（xyzw）+ `grasp_pose: tuple[float,float,float,float,float,float] \| None`（x,y,z,qx,qy,qz,qw，或拆成 position+orientation）。 |
| `code/python/src/robot_arm_demo/core/command.py` `parse_task_command` | 解析 `orientation` / `grasp_pose`，不解析则保持 `None`（兼容旧 payload，`supported` 判定相应更新）。 |
| `code/python/src/robot_arm_demo/core/interfaces.py` | 如需在 core 中抽象，新增 `PoseEstimator` Protocol（`estimate_pose(...) -> dict\|None`）。`ObjectDetector` 契约可保留但感知节点不再走它。 |
| `code/python/src/robot_arm_demo/demos/panda_mujoco/config.py` | ① 相机改为读真实 `camera_info`（frame_id/fx/fy/cx/cy 用 `/camera/camera_info`，不再用固定 `assumed_depth`）；② 加 `service:{base_url,timeout,retries}`；③ 物体系抓取点（Stage 2）。 |
| `code/python/src/robot_arm_demo/core/pick_place.py` `PickPlaceController` | ① `run()` 用 `grasp_pose`（而非纯 `position`）；② 传四元数给 `move_cartesian`；③ 预抓取点 + 沿海轴接近（两个 cartesian goal 或一段路径）；④ 位移/微抬校验改为不依赖 MuJoCo GT（Stage 2/3，见 §9）。 |
| `code/python/src/robot_arm_demo/adapters/moveit_arm.py` `move_cartesian` | 形参 `orientation_wxyz` 已存在，只需 FSM 传入（当前 `pick_place.py` 全都不传，落到固定 `tip_orientation_wxyz`）。 |

**消息流**（`/robot_command` 新形态，`TaskCommand`/`parse_task_command` 需支持）：

```jsonc
{
  "target_object": "可乐",
  "action": "pick",
  "position": [0.30, -0.05, 0.30],       // 抓取点（base 系, 米）
  "orientation": [0.0, 0.0, 0.0, 1.0],   // 抓取时末端姿态（xyzw, base 系）
  "grasp_pose": [0.30, -0.05, 0.30, 0.0, 0.0, 0.0, 1.0]  // 可选：冗余/完整抓取姿态
}
```

> `orientation` 与 `grasp_pose` 二选一即可，建议 `position`+`orientation` 为主，`grasp_pose` 作为带接近轴/冗余信息的增强字段（Stage 2 再启用）。

---

## 8. Stage 1 实现目标与约束

Stage 1 的目的是**把整条「本地↔云↔控制侧接 orientation」的链路打通**，云端暂用 `mask + 几何位姿`，**不依赖 FoundationPose**：

- 云端：`Grounding-DINO + SAM2` → 目标实例 mask →（用 mask 点云 + 内参）RANSAC/圆柱拟合推出**中心 + 瓶轴 axis** + 一个基准四元数（瓶轴对齐到 Z 的旋转）。输出 §5 schema。
- 本地：perception_node 发云、收位姿；tf2 完整位姿；`TaskCommand` 带 `orientation`；控制侧 `pick_place.py` 先用四元数（正交化后）调用 `move_cartesian`（仍可先竖抓，但已能承载任意姿态）。

### 8.1 非功能要求
- **模型常驻（warm）**：云端一次加载 Grounding-DINO + SAM2，进程常驻；`GET /health` 返回已加载模型。避免每请求冷启动。
- **延迟预算**：单次 `estimate_pose` 目标 < 2s（GPU）；本地 `timeout` 设 15s 给足余量。
- **一次性 vs 持续**：当前抓取流程是「LLM 指令→检测→抓取」一次性，单次往返足够。若未来需要**夹持时重估位姿**（替代 MuJoCo GT 验证），请保证接口每次独立无状态（本契约已是无状态，可直接复用）。

---

## 9. 暂缓项（TBD，标记到 Stage 2/3，不影响本契约 v1）

1. **FoundationPose 物体参考**（`reference_object_id` / mesh 或参考图）：Stage 3 引入，本契约预留 `reference_object_id` 字段与 `/v1/objects/{id}/register` 端点（见附录 A）。
2. **物体系抓取点**：`ObjectConfig` 当前只有 `radius/height`；需新增「物体坐标系下的抓取点 + 接近轴」，用于 Stage 2 由 6D 位姿推导世界抓取点。本契约不规定此点，属本地控制侧。
3. **夹持时位姿重估 / 抓取验证替代 GT**：`pick_place.py` 现在靠 MuJoCo `free_joint_state_publisher` 的 ground truth 做位移/微抬校验；真实机械臂没有 GT，需用指尖力/触觉 + 力力矩，或**在夹持时再调一次云服务重估位姿**替代。本契约已无状态、可直接支持后者。

---

## 附录 A：物体参考注册端点（Stage 3 预留）

`POST /v1/objects/{object_id}/register`（body 传 mesh 二进制或参考图 base64）→ 存服务端，供 FoundationPose 用。Stage 1 不需要。**字段/格式在决定 mesh vs 参考图后补全**（见 §9.1）。

---

## 附录 B：JSON Schema（机器可读，供生成客户端/校验）

（下节为可选的机器可读 schema，供 FastAPI pydantic 或 TS/CLI 生成）

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "EstimatePoseRequest",
  "type": "object",
  "required": ["version","image","camera_intrinsics","camera_frame_id","prompt","object_id"],
  "properties": {
    "version": {"const":"v1"},
    "image": {
      "type":"object",
      "required":["encoding","width","height","data_b64"],
      "properties":{
        "encoding":{"const":"rgb8"},
        "width":{"type":"integer","minimum":1},
        "height":{"type":"integer","minimum":1},
        "data_b64":{"type":"string"}
      }
    },
    "depth": {
      "type":"object",
      "properties":{
        "encoding":{"const":"32FC1"},
        "width":{"type":"integer","minimum":1},
        "height":{"type":"integer","minimum":1},
        "data_b64":{"type":"string"}
      }
    },
    "camera_intrinsics": {
      "type":"object",
      "required":["fx","fy","cx","cy","width","height"],
      "properties":{
        "fx":{"type":"number","exclusiveMinimum":0},
        "fy":{"type":"number","exclusiveMinimum":0},
        "cx":{"type":"number"},
        "cy":{"type":"number"},
        "width":{"type":"integer","minimum":1},
        "height":{"type":"integer","minimum":1},
        "distortion_k":{"type":"array","items":{"type":"number"},"minItems":5,"maxItems":5}
      }
    },
    "camera_frame_id":{"type":"string"},
    "prompt":{"type":"string"},
    "object_id":{"type":"string"},
    "return_mask":{"type":"boolean"},
    "max_detections":{"type":"integer","minimum":1},
    "reference_object_id":{"type":["string","null"]}
  },
  "additionalProperties": false
}
```
