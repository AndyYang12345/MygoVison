# SimulationCamera - 仿真相机类使用指南

## 概述

`SimulationCamera` 是一个功能完整的3D空间仿真相机类，能够：

1. **平面投影**：将靶平面从2D投影到3D空间
2. **透视变换**：根据相机位置和姿态进行透视变换
3. **相机模型**：实现完整的相机内外参数模型
4. **光学效果**：支持运动模糊、焦点效果、镜头畸变等
5. **与仿真流集成**：与 `TrainingFrameGenerator` 无缝集成

---

## 核心类结构

### 1. CameraIntrinsics (相机内参数)

```cpp
struct CameraIntrinsics {
    float fx, fy;      // 焦距（像素）
    float cx, cy;      // 主点坐标
    float k1, k2;      // 径向畸变系数
    float p1, p2;      // 切向畸变系数
    float sensor_width;    // 传感器宽度(mm)
    float sensor_height;   // 传感器高度(mm)
};
```

**说明**：
- `fx, fy`: 相机焦距（通常由相机标定得到）
- `cx, cy`: 图像的主点（通常为图像中心）
- `k1, k2`: 镜头畸变参数
- 传感器尺寸：用于计算视场角(FOV)

### 2. CameraPose (相机位置和姿态)

```cpp
struct CameraPose {
    cv::Point3f position;              // 相机位置(mm)
    cv::Point3f rotation;              // 欧拉角(pitch, yaw, roll)
    cv::Point3f target_plane_normal;   // 靶平面法向量
    float target_plane_distance;       // 靶平面到原点的距离
};
```

**说明**：
- `position`: 相机在世界坐标系中的位置
- `rotation`: 相机的旋转（欧拉角，单位：弧度）
- `target_plane_normal`: 定义靶平面的法向量
- `target_plane_distance`: 靶平面与原点的距离

### 3. ProjectionResult (投影结果)

```cpp
struct ProjectionResult {
    cv::Mat projected_frame;           // 投影后的图像
    std::vector<cv::Point2f> projected_positions;  // 投影位置(像素)
    std::vector<cv::Point3f> world_positions;      // 世界坐标
    bool is_in_view;                   // 是否在视野内
    float distance_to_camera;          // 到相机的距离(mm)
};
```

---

## 使用示例

### 基本使用流程

```cpp
#include "TargetSim/SimulationCamera.hpp"

// 1. 创建相机对象 (800x600分辨率, 30FPS)
SimulationCamera camera(800, 600, 30.0f);

// 2. 设置相机内参数
SimulationCamera::CameraIntrinsics intrinsics;
intrinsics.fx = 400.0f;  // 焦距
intrinsics.fy = 400.0f;
intrinsics.cx = 400.0f;  // 主点
intrinsics.cy = 300.0f;
camera.set_intrinsics(intrinsics);

// 3. 设置相机位置和姿态
SimulationCamera::CameraPose pose;
pose.position = cv::Point3f(0, 0, 500);      // 相机在Z=500mm处
pose.rotation = cv::Point3f(0, 0, 0);        // 无旋转
pose.target_plane_distance = 500;             // 靶平面距离
camera.set_pose(pose);

// 4. 创建仿真帧生成器
TrainingFrameGenerator generator(800, 600, 30.0f);
generator.set_circular_motion(cv::Point2f(400, 300), 150.0f, 1.0f);

// 5. 获取投影结果
auto result = camera.project_training_frame(generator);

// 6. 使用投影结果
if (result.is_in_view) {
    cv::imshow("Projected Frame", result.projected_frame);
    std::cout << "Target position: " << result.projected_positions[0] << std::endl;
    std::cout << "World position: " << result.world_positions[0] << std::endl;
}
```

---

## 关键函数说明

### 坐标转换函数

#### `world_to_image()` - 世界坐标转图像坐标

```cpp
cv::Point2f image_pos = camera.world_to_image(cv::Point3f(100, 0, 500));
```

- **输入**：世界坐标系中的点(单位：mm)
- **输出**：相应的图像坐标(像素)
- **用途**：将3D点投影到2D图像

#### `image_to_world()` - 图像坐标转世界坐标

```cpp
cv::Point3f world_pos = camera.image_to_world(cv::Point2f(400, 300));
```

- **输入**：图像坐标(像素)
- **输出**：对应的世界坐标(单位：mm)
- **用途**：从2D图像反投影到3D空间(基于靶平面)

#### `is_in_view()` - 检查点是否在视野内

```cpp
bool visible = camera.is_in_view(world_point);
```

---

### 投影函数

#### `project_training_frame()` - 投影训练帧

```cpp
auto result = camera.project_training_frame(generator);
```

- 从 `TrainingFrameGenerator` 获取帧
- 执行透视变换
- 返回完整的投影结果

#### `project_target_position()` - 投影单个目标位置

```cpp
auto result = camera.project_target_position(
    cv::Point2f(400, 300), 
    original_frame
);
```

---

### 相机操作函数

#### `update_pose()` - 更新相机位置和姿态

```cpp
camera.update_pose(
    cv::Point3f(10, 0, 0),      // 位置增量(mm)
    cv::Point3f(0, 0.05f, 0)    // 旋转增量(rad)
);
```

#### `get_target_plane_projection()` - 获取靶平面投影

```cpp
std::vector<cv::Point2f> corners = camera.get_target_plane_projection();
```

- 返回靶平面四个角的像素坐标
- 用于绘制靶平面的可视化边框

---

### 光学效果函数

#### `apply_motion_blur()` - 运动模糊

```cpp
cv::Mat blurred = camera.apply_motion_blur(
    frame,
    cv::Point2f(10, 5),  // 速度向量(像素/帧)
    3                    // 模糊强度(1-5)
);
```

#### `apply_focus_effect()` - 焦点效果(景深)

```cpp
cv::Mat focused = camera.apply_focus_effect(
    frame,
    400,    // 焦距距离(mm)
    2.0f    // 光圈大小
);
```

#### `apply_lens_distortion()` - 镜头畸变

```cpp
cv::Mat distorted = camera.apply_lens_distortion(frame);
```

---

## 工作流程示意

```
┌─────────────────────────────────────────────────────────────┐
│                      仿真相机工作流程                          │
└─────────────────────────────────────────────────────────────┘

1. 初始化阶段
   ├─ 创建 SimulationCamera
   ├─ 设置相机内参(CameraIntrinsics)
   └─ 设置相机位置/姿态(CameraPose)

2. 帧生成阶段
   ├─ 创建 TrainingFrameGenerator
   ├─ 设置运动模式(圆周、线性、参数方程等)
   └─ 生成 TrainingFrame

3. 投影阶段
   ├─ 获取原始2D帧
   ├─ 计算透视变换矩阵
   └─ 生成投影帧

4. 后处理阶段(可选)
   ├─ 应用运动模糊
   ├─ 应用焦点效果
   └─ 应用镜头畸变

5. 结果输出
   ├─ 投影帧(cv::Mat)
   ├─ 投影位置(像素)
   ├─ 世界坐标(mm)
   └─ 可见性标志
```

---

## 坐标系定义

### 世界坐标系

```
    Y
    ^
    |     Z
    |    /
    |   /
    +------ X
```

- **原点**：通常在靶平面中心
- **单位**：毫米(mm)
- **靶平面**：垂直于Z轴，位于Z = target_plane_distance处

### 相机坐标系

```
    Y'
    ^
    |     Z'(光轴)
    |    /
    |   /
    +------ X'
```

- **原点**：相机光心
- **Z轴**：光轴方向(沿焦距)
- **X'Y'平面**：图像平面之前

### 图像坐标系

```
(0,0) ─────── X(像素) ─────────> (width-1, 0)
  |
  |
  Y(像素)
  |
  v
(0, height-1) ─────────────────> (width-1, height-1)
```

---

## 高级特性

### 1. 动态相机运动

```cpp
// 模拟相机绕目标运动
for (int frame = 0; frame < 100; frame++) {
    float angle = frame * 0.1f;  // 每帧旋转0.1弧度
    
    // 更新相机位置(圆周运动)
    float radius = 500;
    cv::Point3f pos(
        radius * std::cos(angle),
        radius * std::sin(angle),
        300
    );
    
    SimulationCamera::CameraPose pose = camera.get_pose();
    pose.position = pos;
    camera.set_pose(pose);
    
    auto result = camera.project_training_frame(generator);
    // 处理投影结果...
}
```

### 2. 多相机仿真

```cpp
// 创建多个相机
SimulationCamera camera1(800, 600, 30.0f);
SimulationCamera camera2(800, 600, 30.0f);

// 设置不同的位置和姿态
// camera1: 正视图
// camera2: 侧视图

// 同时获取多个视图的投影
auto result1 = camera1.project_training_frame(generator);
auto result2 = camera2.project_training_frame(generator);
```

### 3. 相机标定模拟

```cpp
// 标定相机参数
for (int test = 0; test < num_calibration_points; test++) {
    // 获取已知世界坐标
    cv::Point3f world_pos(100, 100, 500);
    
    // 投影到图像
    cv::Point2f image_pos = camera.world_to_image(world_pos);
    
    // 验证反向投影
    cv::Point3f world_back = camera.image_to_world(image_pos);
    
    // 计算误差
    float error = cv::norm(world_pos - world_back);
}
```

---

## 测试和验证

### 运行测试程序

```bash
cd /home/harekasa/Mygo/build/bin
./test_simulation_camera
```

### 生成的测试文件

- `projected_frame_1.jpg`, `projected_frame_2.jpg`, `projected_frame_3.jpg`：投影帧
- `motion_blur_test.jpg`：运动模糊效果
- `focus_effect_test.jpg`：焦点效果

---

## 常见问题

### Q: 为什么目标看起来不在视野内？

A: 检查以下几点：
1. 相机位置(position)是否正确
2. 相机旋转(rotation)是否正确
3. 靶平面距离(target_plane_distance)是否合理
4. 图像分辨率和焦距是否匹配

### Q: 如何实现特定的相机运动？

A: 使用 `update_pose()` 函数逐帧更新相机位置和姿态。

### Q: 投影结果看起来很大或很小？

A: 调整相机内参(特别是焦距fx, fy)或相机到目标的距离。

---

## 后续应用

这个 `SimulationCamera` 类为以下应用奠定了基础：

1. **3D空间仿真**：完整的相机和目标运动模型
2. **深度学习训练**：生成多视角训练数据
3. **性能评估**：在不同视角和距离下评估识别性能
4. **轨迹预测**：基于相机和目标运动的轨迹预测
5. **增强现实**：虚拟物体的3D投影

---

## 相关文件

- 头文件：`include/TargetSim/SimulationCamera.hpp`
- 源文件：`src/TargetSim/SimulationCamera.cpp`
- 测试程序：`src/apps/test_simulation_camera.cpp`
