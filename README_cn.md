# 功能介绍

基于深度学习的方法识别小球和底座，使用模型为YOLOv5s

# 使用方法

## 准备工作

1. 具备TonyPi机器人，包含相机及RDK套件，并且能够正常运行。
2. 具备小球等相关道具

## 编译与运行

**1.编译**

启动机器人后，通过终端SSH或者VNC连接机器人，打开终端拉取相应代码并编译安装

```bash
# 拉取目标检测代码与图像矫正代码
mkdir -p ~/tonypi_ws/src && cd ~/tonypi_ws/src

# RDK X5
git clone https://github.com/wunuo1/tonypi_obj_detection.git -b feature-humble-x5
git clone https://github.com/wunuo1/tonypi_image_correction.git -b feature-humble-x5
# RDK X3
git clone https://github.com/wunuo1/tonypi_obj_detection.git -b feature-foxy-x3
git clone https://github.com/wunuo1/tonypi_image_correction.git -b feature-foxy-x3

# 编译
cd ..
source /opt/tros/setup.bash
colcon build
```

**2.运行物体检测功能**

```shell
source /opt/tros/local_setup.bash
cp -r /opt/tros/lib/tonypi_obj_detection/config/ .

# web端可视化障碍物（启动功能后在浏览器打开 ip:8000）
export WEB_SHOW=TRUE

ros2 launch tonypi_obj_detection target_detection.launch.py

```

# 原理简介

RDK X3通过摄像头获取机器人前方环境数据，图像数据通过训练好的YOLO模型进行推理得到物体的图像坐标值并发布。

# 接口说明

## 话题

### Pub话题

| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /robot_target_detection    | ai_msgs/msg/PerceptionTargets             | 发布物体信息                |

### Sub话题
| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /hb_image       | hbm_img_msgs/msg/HbmMsg1080P      | 接收相机发布的图片消息（640x480）                   |

## 参数

| 参数名                | 类型        | 说明                                                                                                                                 |
| --------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| sub_img_topic       | string |     接收的图片话题名称，请根据实际接收到的话题名称配置，默认值为/hb_image |
| config_file | string | 配置文件读取路径，请根据识别情况配置，默认值为config/TonyPi_yolov5sconfig.json |

# 注意
该功能包提供特定的实际场景中识别物体的模型，若自行采集数据集进行训练，请注意替换。