# 任务流:从地面站下发到机载的 skill 序列
# 仅描述 ROS 内部路由格式,不替 starling_bridge 业务帧
string   flow_id
uint64   issued_at_us
uint8    device_id
Skill[]  skills
