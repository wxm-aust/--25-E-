# 2025 全国大学生电子设计竞赛 C题：基于单目视觉的目标物测量装置

## 项目概述

本项目基于 ROS 框架，运行于 Orange Pi 开发板，通过单目摄像头实现对 A4 纸张上目标图形的自动检测与尺寸测量。系统支持多种形状识别（正方形、三角形、圆形、重叠正方形、数字正方形等），并利用透视变换和 PnP 算法计算目标物到摄像头的距离及目标物的实际物理尺寸。

## 硬件平台

- **主控**: Orange Pi 4
- **摄像头**: 海康MV-CE060
- **功率采集器**: STM32 

## 软件架构

项目基于 ROS Melodic/Noetic，包含三个 ROS 功能包：

```
25电赛/
├── src/
│   ├── cv/                  # 核心视觉检测包
│   ├── serial_sender/       # 串口通信包
│   └── qt/                  # GUI 控制面板包
├── C题_基于单目视觉的目标物测量装置.pdf
└── VID_20250801_210319.mp4  # 演示视频
```

### 节点通信拓扑

```
[摄像头] ──> detect.py ──> /dx_dy ──> serial_sender_node ──> [MCU]
                │
                ├──> /d_value (距离)
                ├──> /x_value (尺寸)
                │
[MCU] ──> serial_receiver_node ──> /received_data ──> detect.py
                                     │
                                     └──> menu.py (GUI显示)
               
[menu.py] ──> /button_clicks ──> detect.py (切换检测模式)
```

## 功能包详解

### 1. cv 包 (核心视觉检测)

#### 主要文件

| 文件 | 功能 |
|------|------|
| `detect.py` | 主检测节点，摄像头采集、形状检测调度、距离/尺寸计算与发布 |
| `hk_detect.py` | 海康威视工业相机 SDK 封装，支持 USB/GigE 相机枚举与采集 |
| `Get_rect.py` | 矩形/多边形轮廓检测，含灰度、彩色(HSV)、OTSU 三种预处理方式 |
| `Get_cir.py` | 圆形检测、透视变换(`transform_to_face`)、点绘制等工具函数 |
| `digital.py` | OCR 数字识别，基于 ddddocr 库 |
| `yjm_pnp.py` | PnP 位姿估计，相机内参/畸变系数定义，距离反算 |
| `meansere.py` | 单应性矩阵分解(Homography Decomposition)、世界坐标计算 |
| `get_hsv_thr.py` | HSV 阈值调试工具，通过滑块实时调节颜色阈值 |
| `PIDConfig.cfg` | Dynamic Reconfigure 动态参数配置 |

#### 支持的检测模式

| 按键编号 | 模式 | 说明 |
|----------|------|------|
| 11 | 正方形(square) | 检测 A4 纸上的单个正方形 |
| 12 | 三角形(triangle) | 检测单个三角形 |
| 13 | 圆形(circle) | 检测单个圆形 |
| 14 | 多正方形(muti_square) | 检测多个正方形，返回最小者 |
| 15 | 重叠正方形(fold_square) | 检测有重叠的正方形 |
| 16 | 数字正方形(number_square) | 检测带数字的正方形，配合数字键 0-9 选择目标 |
| 17 | 倾斜+正方形(angle_square) | 透视校正后检测正方形 |
| 18 | 倾斜+重叠正方形(test1) | 透视校正后检测重叠正方形 |
| 19 | 倾斜+数字正方形(test2) | 透视校正后检测数字正方形 |
| 20 | 倾斜+多正方形(btn20) | 透视校正后检测多个正方形 |

#### 核心算法流程

1. **图像采集**: 从 USB 摄像头捕获 1920×1080 帧，裁剪为 480×480 中心区域
2. **外框检测**: 使用 OTSU 二值化 + 轮廓检测定位 A4 纸的四个角点
3. **ROI 提取**: 通过透视变换将 A4 纸区域校正为正面视图
4. **形状检测**: 在 ROI 区域内根据当前模式选择对应的检测算法
5. **距离计算**: 
   - 正面测距：`D = 88845.79 × 0.01 / side_sum`
   - 倾斜测距：`D = 91251.92 × 0.01 / side_sum`
6. **尺寸计算**:
   - 矩形/正方形：`X = x_pix × D × 0.08080`
   - 三角形/圆形/重叠正方形：`X = x_pix × D × 0.099`
7. **结果发布**: 通过 ROS Topic 发布距离(`d_value`)和尺寸(`x_value`)

#### 相机参数

- 内参矩阵:
  ```
  fx=1557.07, fy=1488.26, cx=241.42, cy=322.75
  ```
- 畸变系数: `[0.516, 13.425, 0.017, 0.0068, -573.23]`

#### 依赖库

- OpenCV (图像处理)
- ddddocr (OCR 数字识别)
- PyYAML (配置文件解析)
- NumPy (数值计算)

### 2. serial_sender 包 (串口通信)

#### 主要文件

| 文件 | 功能 |
|------|------|
| `serial_sender_node.cpp` | 发送节点，将 dx/dy 数据打包发送至 MCU |
| `serial_receiver_node.cpp` | 接收节点，从 MCU 接收电压/电流数据 |

#### 通信协议

**发送协议** (→ MCU, `/dev/ttyS3`, 115200 bps, 100Hz):
```
[A5] [d1_high] [d1_low] [d2_high] [d2_low] [checksum]
```
- d1: Roll 角度 (蓝色舵机, 对应 dy)
- d2: Pitch 角度 (黑色舵机, 对应 dx)
- 校验和: 前 5 字节累加取低 8 位

**接收协议** (← MCU, `/dev/ttyS4`, 9600 bps):
```
[A5] [data_byte1] [data_byte2] [data_byte3] [data_byte4] [checksum]
```
- 数据为 32 位大端序，转换为电压值后发布到 `/received_data` 话题

### 3. qt 包 (GUI 控制面板)

基于 PyQt5 构建的嵌入式控制面板，主要功能：

- **20 个按钮**: 数字键 0-9 用于选择目标数字，功能键用于切换 8 种检测模式
- **实时监测显示**: 显示电流(A)、功率(W)、距离(D)、目标尺寸(x)
- **ROS 集成**: 通过 `/button_clicks` 话题发布按钮事件，订阅 `/current_value`、`/power_value`、`/d_value`、`/x_value` 话题

## 启动方式

### 主启动 (视觉检测 + GUI + 串口接收)

```bash
roslaunch cv detect.launch
```

### 串口发送启动 (单独启动舵机控制)

```bash
roslaunch serial_sender wxm.launch
```

## 测量原理

系统利用 A4 纸的已知尺寸 (210mm × 297mm) 作为参照物：

1. 检测 A4 纸在图像中的四个角点
2. 根据角点像素坐标计算纸张的像素周长/边长
3. 利用比例关系反算距离：`D = K / perimeter_pixels`（K 为标定系数）
4. 在已知距离 D 后，通过目标形状的像素尺寸换算实际物理尺寸

对于倾斜的纸张，系统先进行透视变换校正，再执行上述测量流程。

## 文件清单

```
src/
├── cv/
│   ├── cfg/PIDConfig.cfg                  # 动态参数配置
│   ├── launch/detect.launch               # 主启动文件
│   ├── scripts/
│   │   ├── detect.py                      # ★ 主检测节点
│   │   ├── hk_detect.py                   # 海康相机驱动
│   │   ├── Get_rect.py                    # 矩形/多边形检测
│   │   ├── Get_cir.py                     # 圆形检测与透视变换
│   │   ├── digital.py                     # OCR 数字识别
│   │   ├── yjm_pnp.py                     # PnP 位姿估计
│   │   ├── meansere.py                    # 单应性矩阵分解
│   │   ├── get_hsv_thr.py                 # HSV 阈值调试工具
│   │   ├── pid_config.yaml                # PID 参数
│   │   ├── code.png / code0.png           # 标定图像
│   │   ├── 图像变换.png / 找框.png / 预处理.png  # 算法示意图
│   │   └── __pycache__/                   # Python 字节码缓存
│   ├── test_imgs/                         # 0-9 数字测试图像
│   ├── CMakeLists.txt
│   └── package.xml
├── serial_sender/
│   ├── launch/wxm.launch                  # 串口启动文件
│   ├── src/
│   │   ├── serial_sender_node.cpp         # 串口发送节点
│   │   └── serial_receiver_node.cpp       # 串口接收节点
│   ├── CMakeLists.txt
│   └── package.xml
└── qt/
    ├── scripts/menu.py                    # ★ GUI 控制面板
    ├── CMakeLists.txt
    └── package.xml
```

## 开发环境

- **操作系统**: Ubuntu (ARM, Orange Pi)
- **ROS 版本**: Melodic / Noetic
- **Python 版本**: 3.8
- **构建工具**: catkin_make
- **GUI 框架**: PyQt5
- **视觉库**: OpenCV 4.x
- **OCR 引擎**: ddddocr
