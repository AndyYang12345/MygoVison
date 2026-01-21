# Pentagon Simulator with Target Tracking

## 概述

`pentagon_simulator_app` 是一个集成了 `PentagonSimulator` 和 `TargetTracker` 的演示应用程序，用于虚拟仿真五角形靶子的旋转投影，并实时进行目标识别和跟踪。

## 功能特性

### 1. 3D 投影模拟
- 生成五角形靶子的3D旋转帧
- 支持可配置的相机参数（焦距、分辨率等）
- 支持可交互的相机控制（俯仰角、偏航角）

### 2. 实时目标追踪
- 使用 `TargetTracker` 识别目标色块
- 计算标靶板中心和目标色块中心
- 实时计算距离和角度

### 3. 可视化展示
- 在图像上绘制标靶板中心（黄色圆点）
- 在图像上绘制目标色块中心（绿色圆点）
- 绘制板中心到目标中心的连接线
- 显示距离和角度信息

## 编译与运行

### 编译
```bash
cd /home/harekasa/Mygo
mkdir -p build
cd build
cmake ..
make
```

### 运行
```bash
./bin/pentagon_simulator_app
```

## 交互控制

运行程序后，可以使用以下按键进行交互：

| 按键 | 功能 |
|------|------|
| `W/w` | 镜头向上旋转（减小俯仰角）|
| `S/s` | 镜头向下旋转（增加俯仰角）|
| `A/a` | 镜头向左旋转（增加偏航角）|
| `D/d` | 镜头向右旋转（减小偏航角）|
| `P/p` | 暂停/继续动画 |
| `Q/q` 或 `ESC` | 退出程序 |

## 输出信息

### 实时显示
- **Frame**: 当前帧编号
- **Pitch**: 相机俯仰角（度）
- **Yaw**: 相机偏航角（度）
- **状态**: PLAYING 或 PAUSED
- **Distance**: 标靶板中心到目标色块的距离（像素）
- **Angle**: 连接线与水平线的夹角（度）

### 统计信息（程序退出时）
- 总帧数
- 成功追踪帧数
- 追踪成功率
- 详细统计信息

## 配置参数

### PentagonSimulator 配置
```cpp
PentagonSimulator::CameraConfig config;
config.width = 640;              // 图像宽度
config.height = 640;             // 图像高度
config.fps = 30.0f;              // 帧率
config.fx = 381.625f;            // 焦距（x方向）
config.fy = 381.625f;            // 焦距（y方向）
config.cx = 320.0f;              // 主点（x坐标）
config.cy = 320.0f;              // 主点（y坐标）
config.position = cv::Point3f(0, -100, 1000);  // 相机位置
config.pitch = 0.35f;            // 初始俯仰角（弧度）
config.yaw = 0.0f;               // 初始偏航角（弧度）
```

### TargetTracker 配置
```cpp
TrackerConfig tracker_config;
tracker_config.min_blob_area = 100;          // 最小色块面积
tracker_config.max_blob_area = 5000;         // 最大色块面积
tracker_config.min_circularity = 0.5f;       // 最小圆形度
tracker_config.show_debug_windows = false;   // 是否显示调试窗口
tracker_config.print_debug_info = false;     // 是否打印调试信息
```

## 绘制说明

程序会在输出图像上绘制以下元素：

1. **标靶板中心** - 黄色圆点，标记标靶板的中心位置
2. **目标色块中心** - 绿色圆点，标记识别到的目标色块位置
3. **连接线** - 青色线段，连接标靶板中心和目标色块中心
4. **文字标注** - 显示各点位置的标签和距离/角度信息

## 用途

这个应用程序可用于：
- 训练目标识别算法
- 测试不同的追踪参数
- 生成虚拟训练数据
- 验证追踪算法的性能
- 调试相机参数

## 依赖项

- OpenCV (cv::Mat, cv::Point, cv::circle, etc.)
- PentagonSimulator（五角形模拟器类）
- TargetTracker（目标追踪类）
- C++17 或更高版本

## 注意事项

- 程序运行时会持续消耗CPU资源，请在需要时按'Q'键退出
- 追踪器配置参数可能需要根据具体的靶子颜色进行调整
- 图像分辨率越高，计算量越大，帧率可能会降低
