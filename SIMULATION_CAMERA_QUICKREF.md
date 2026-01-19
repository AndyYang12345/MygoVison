# SimulationCamera 快速参考卡

## 📦 包含头文件

```cpp
#include "TargetSim/SimulationCamera.hpp"
#include "TargetSim/TrainingFrameGenerator.hpp"
```

---

## ⚡ 快速开始（30秒）

```cpp
// 1. 创建相机
SimulationCamera camera(800, 600, 30.0f);

// 2. 创建帧生成器
TrainingFrameGenerator gen(800, 600, 30.0f);
gen.set_circular_motion(cv::Point2f(400, 300), 150, 1.0f);

// 3. 投影帧
auto result = camera.project_training_frame(gen);

// 4. 使用结果
cv::imshow("Result", result.projected_frame);
std::cout << "Position: " << result.projected_positions[0] << std::endl;
```

---

## 🎛️ 常用操作

### 设置相机内参
```cpp
SimulationCamera::CameraIntrinsics intrinsics;
intrinsics.fx = 400.0f;      // X焦距
intrinsics.fy = 400.0f;      // Y焦距
intrinsics.cx = 400.0f;      // X主点
intrinsics.cy = 300.0f;      // Y主点
camera.set_intrinsics(intrinsics);
```

### 设置相机位置/姿态
```cpp
SimulationCamera::CameraPose pose;
pose.position = cv::Point3f(x, y, z);        // 位置(mm)
pose.rotation = cv::Point3f(pitch, yaw, roll);  // 欧拉角(rad)
pose.target_plane_distance = 500;            // 靶平面距离
camera.set_pose(pose);
```

### 更新相机位置
```cpp
camera.update_pose(
    cv::Point3f(10, 0, 0),     // 位置增量
    cv::Point3f(0, 0.1f, 0)    // 旋转增量
);
```

### 坐标转换
```cpp
// 世界 → 图像
cv::Point2f image_pos = camera.world_to_image(world_point);

// 图像 → 世界
cv::Point3f world_pos = camera.image_to_world(image_point);

// 检查可见性
if (camera.is_in_view(world_point)) { /* ... */ }
```

### 投影和获取结果
```cpp
// 投影训练帧
auto result = camera.project_training_frame(generator);

// 检查结果
if (result.is_in_view) {
    std::cout << "投影位置: " << result.projected_positions[0] << std::endl;
    std::cout << "世界坐标: " << result.world_positions[0] << std::endl;
    std::cout << "到相机距离: " << result.distance_to_camera << "mm" << std::endl;
}
```

---

## 🎨 光学效果

### 运动模糊
```cpp
cv::Mat blurred = camera.apply_motion_blur(
    frame, 
    cv::Point2f(velocity_x, velocity_y),  // 速度向量
    blur_strength                         // 1-5
);
```

### 焦点效果（景深）
```cpp
cv::Mat focused = camera.apply_focus_effect(
    frame,
    focus_distance,  // 焦距(mm)
    aperture         // 光圈大小，越大越模糊
);
```

### 镜头畸变
```cpp
cv::Mat distorted = camera.apply_lens_distortion(frame);
```

---

## 📊 数据结构

### ProjectionResult
```cpp
struct ProjectionResult {
    cv::Mat projected_frame;              // 投影后的图像
    std::vector<cv::Point2f> projected_positions;  // 像素坐标
    std::vector<cv::Point3f> world_positions;      // 世界坐标
    bool is_in_view;                      // 是否在视野内
    float distance_to_camera;             // 距离(mm)
};
```

---

## 🔧 配置示例

### 标准相机（焦距400px）
```cpp
SimulationCamera::CameraIntrinsics standard;
standard.fx = 400.0f;
standard.fy = 400.0f;
standard.cx = 400.0f;
standard.cy = 300.0f;
```

### 广角相机（焦距200px）
```cpp
SimulationCamera::CameraIntrinsics wide;
wide.fx = 200.0f;
wide.fy = 200.0f;
```

### 长焦相机（焦距800px）
```cpp
SimulationCamera::CameraIntrinsics tele;
tele.fx = 800.0f;
tele.fy = 800.0f;
```

---

## 📐 坐标系

### 世界坐标系 (mm)
- 单位: 毫米
- 原点: 通常在靶平面中心
- Z轴: 从相机指向靶

### 相机坐标系
- 原点: 光心
- Z轴: 光轴
- X, Y轴: 图像平面

### 图像坐标系 (px)
- 单位: 像素
- 原点: 左上角(0,0)
- X轴: 向右
- Y轴: 向下

---

## 💡 技巧

### 多相机仿真
```cpp
std::vector<SimulationCamera> cameras;
for (int i = 0; i < 3; i++) {
    SimulationCamera cam(800, 600, 30.0f);
    // 配置相机...
    cameras.push_back(cam);
}

// 同时投影
for (auto& cam : cameras) {
    auto result = cam.project_training_frame(generator);
}
```

### 动态相机运动
```cpp
for (int t = 0; t < frames; t++) {
    float angle = t * 0.1f;
    camera.update_pose(
        cv::Point3f(600*cos(angle), 600*sin(angle), 400),
        cv::Point3f(0, angle, 0)
    );
    auto result = camera.project_training_frame(generator);
}
```

### 获取靶平面投影
```cpp
std::vector<cv::Point2f> corners = camera.get_target_plane_projection();
// corners[0-3]: 靶平面四个角的像素坐标
cv::polylines(image, corners, true, cv::Scalar(0, 255, 0), 2);
```

---

## 🚀 实用代码片段

### 相机标定验证
```cpp
std::vector<cv::Point3f> world_points = {...};
std::vector<cv::Point2f> image_points;

for (auto& wp : world_points) {
    image_points.push_back(camera.world_to_image(wp));
}

// 计算重投影误差
float error = 0;
for (size_t i = 0; i < world_points.size(); i++) {
    auto wp_back = camera.image_to_world(image_points[i]);
    error += cv::norm(world_points[i] - wp_back);
}
```

### 绘制靶平面边界
```cpp
auto corners = camera.get_target_plane_projection();
cv::polylines(frame, corners, true, cv::Scalar(0, 255, 0), 2);
```

### 相机位置约束
```cpp
// 保持距离固定，绕Z轴旋转
float radius = 600;  // mm
float angular_velocity = 1.0;  // rad/s

for (float t = 0; t < 10; t += 0.033) {
    SimulationCamera::CameraPose pose;
    pose.position = cv::Point3f(
        radius * cos(angular_velocity * t),
        radius * sin(angular_velocity * t),
        500
    );
    pose.rotation = cv::Point3f(0, angular_velocity * t, 0);
    camera.set_pose(pose);
}
```

---

## 📁 文件位置

| 文件 | 路径 |
|------|------|
| 头文件 | `include/TargetSim/SimulationCamera.hpp` |
| 实现 | `src/TargetSim/SimulationCamera.cpp` |
| 基础测试 | `src/apps/test_simulation_camera.cpp` |
| 高级测试 | `src/apps/advanced_simulation_camera.cpp` |
| 完整指南 | `SimulationCamera_Guide.md` |

---

## ✅ 检查清单

使用前检查:
- [ ] 已包含头文件
- [ ] 已链接 target_sim 库
- [ ] TrainingFrameGenerator 已初始化
- [ ] 相机内参已设置
- [ ] 相机位置已设置

---

## 🆘 常见问题

**Q: 投影位置为负或超出范围？**  
A: 检查相机焦距(fx, fy)是否过小，或相机到靶的距离是否过近。

**Q: 所有点都显示不在视野内？**  
A: 检查靶平面距离(target_plane_distance)设置，或调整相机位置。

**Q: 如何实现自由视角？**  
A: 使用 `update_pose()` 逐帧更新相机位置和旋转。

---

## 📞 技术支持

遇到问题？查看：
1. `SimulationCamera_Guide.md` - 完整文档
2. `test_simulation_camera.cpp` - 基础示例
3. `advanced_simulation_camera.cpp` - 高级示例

---

**最后更新**: 2026年1月19日  
**版本**: 1.0  
**状态**: 生产级别 ✅
