"""感知位姿云服务：契约 + Stage 1 几何位姿 + HTTP 客户端 + mock 服务。

不依赖 ROS，可独立单测；perception_node 通过 cloud_client 调用，
mock_server 与真实云端服务都要遵守 contract 里的 wire 契约。
"""
