# 单个仿真目标（target_motion_simulator_node 发布)
# 仅用于仿真链路:携带一个地面运动目标的类别 + NWU 位置
uint32              id         # 稳定目标 id(YAML 配置顺序,0 起)
string              label      # 类别,如 "person"/"car"/"truck"(与 YOLO label 语义一致)
geometry_msgs/Point position   # 目标位置(NWU,米;z 一般为 0=地面)
