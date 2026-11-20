# 原候选：光学系可见内点重心（米）、法向四元数、YOLO 类名与置信度。
vision_msgs/Detection3D detection

# 模型原始类别索引；当前 mix4 为 0=block、1=top，-1 表示未知。
int32 class_id -1

# 固定颜色编号；0 表示未知，不能当成已确定的颜色使用。
uint8 COLOR_UNKNOWN=0
uint8 COLOR_RED=1
uint8 COLOR_BLUE=2
uint8 COLOR_MIXED=3
uint8 color_id 0
