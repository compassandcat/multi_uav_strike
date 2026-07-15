# 一帧内的多条 YOLO 检测 (gimbal_simulator_node 发布 detection/yolo_results)
# 多目标场景:一帧图像里可能有多个目标同时在 FOV 内,各自一条 YoloDetection。
# target_estimator_node 遍历 detections 逐条做 LOS 反投影 + 聚类。
YoloDetection[] detections
