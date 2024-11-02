from ultralytics import YOLO
import cv2

# 1. 加载预训练的 YOLOv8 模型
model = YOLO("yolov8n.pt")  # 使用 YOLOv8n 轻量级模型

# 2. 指定待推理的图像路径
img_path = "./image.jpg"  # 请替换为您自己的图像路径

# 3. 进行推理
results = model(img_path)

# 4. 显示推理结果
for result in results:
    # 提取推理的图像和预测框
    annotated_image = result.plot()  # 在图像上绘制检测框
    cv2.imshow("YOLOv8 Inference", annotated_image)  # 使用 OpenCV 显示图像
    cv2.waitKey(0)  # 等待按键
    cv2.destroyAllWindows()  # 关闭所有窗口

# 5. 保存推理结果（可选）
result.save("./predicted_image.jpg")  # 请替换为您想保存的路径
