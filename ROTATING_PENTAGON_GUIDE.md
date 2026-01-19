# 五角形旋转靶子3D空间仿真使用指南

## 概述

`rotating_pentagon_test` 是一个专门为演示**五角形靶子在3D空间中旋转运动**而设计的测试程序。它展示了如何利用 `TrainingFrameGenerator` 的五角形生成功能，结合 `SimulationCamera` 的3D投影能力，来创建复杂的仿真场景。

---

## 核心功能

该程序包含**5种不同的仿真模式**，逐步演示从简单到复杂的旋转运动：

### 模式1：靶子绕自身中心旋转 🔄

```
特点: 靶子在原地旋转，相机固定
用途: 验证旋转追踪的基础能力
帧数: 30帧
旋转范围: 0° ~ 360°
```

**关键参数**:
- 相机位置: (0, 0, 600) mm (俯视)
- 靶旋转角: 0° → 360° (每帧12°)
- 投影位置: 恒为 (400, 300) px (中心)

**应用场景**:
- 测试旋转不变性
- 验证中心追踪稳定性
- 基准测试

---

### 模式2：靶子绕固定中心旋转（轨道运动）🌍

```
特点: 靶沿圆形轨道运动，相机固定俯视
用途: 验证轨道追踪和距离计算
帧数: 40帧
轨道: 半径200mm的水平圆形
```

**关键参数**:
- 相机位置: (0, 0, 600) mm
- 轨道中心: (0, 0, 0) mm
- 轨道半径: 200 mm
- 轨道角度: 0° → 360°

**投影轨迹**:
```
投影起点: (266, 300) px   [右侧]
投影最高点: (400, 166) px  [上方]
投影终点: (533, 300) px   [左侧]
投影最低点: (400, 433) px  [下方]
```

**应用场景**:
- 圆形运动追踪
- 距离变化检测
- 可见性判断验证

---

### 模式3：靶子自转+绕中心旋转（复杂运动）⚙️

```
特点: 靶和相机都在运动（双旋转）
用途: 仿真真实的多体运动场景
帧数: 60帧
相机轨道: 半径600mm，角速度0.5倍靶速
```

**运动方程**:
- **靶的运动**:
  - 水平: r=200mm, θ=t
  - 垂直: z=50*sin(2t)
  
- **相机的运动**:
  - 水平: r=600mm, θ=0.5t
  - 垂直: z=300+100*cos(t)

**观察结果**:
- 靶始终在相机视野之外（距离远，角度差）
- 距离范围: 565mm ~ 904mm
- 完整的3D追踪演示

**应用场景**:
- 复杂运动仿真
- 视野管理
- 追踪系统测试

---

### 模式4：五角星旋转序列（使用生成器）⭐

```
特点: 使用TrainingFrameGenerator生成五角星
用途: 验证与仿真流的集成
帧数: 50帧
相机位置: (0, 0, 700) mm (高空俯视)
```

**工作流程**:
1. 创建 `TrainingFrameGenerator`
2. 设置 `MODE_PENTAGON_ROTATION` 模式
3. 逐帧调用 `project_training_frame()`
4. 获取投影结果和五角星靶位置

**输出信息**:
- 靶位置 (像素坐标)
- 时间戳
- 可见性标志
- 投影帧保存

**应用场景**:
- 五角星识别测试
- 仿真流集成验证
- 实际仿真数据生成

---

### 模式5：追踪统计分析 📊

```
特点: 详细的追踪性能统计
用途: 评估追踪系统性能
帧数: 100帧
相机位置: (300, 300, 500) mm (倾斜视角)
```

**统计指标**:
- **可见率**: 35% (35/100帧)
- **平均距离**: 622 mm
- **距离范围**: 570mm ~ 709mm
- **平均位置变化**: 14 px/帧

**数据分析**:
```
时间序列:
  t=0s   : 可见 | 距离 602mm
  t=0.66s: 不可见 | 距离 582mm
  t=1.36s: 不可见 | 距离 687mm
  ...

统计信息:
  平均距离 = 622 mm
  最小距离 = 570 mm
  最大距离 = 709 mm
  位置变化 = 14 px/帧
```

**应用场景**:
- 性能基准测试
- 追踪可靠性评估
- 算法优化验证

---

## 运行方式

### 编译

```bash
cd /home/harekasa/Mygo/build
cmake ..
make rotating_pentagon_test -j4
```

### 执行

```bash
./bin/rotating_pentagon_test
```

### 输出示例

```
***********************************************************
*   五角形旋转靶子3D空间仿真测试
***********************************************************

============================================================
模式1: 靶子绕自身中心旋转
============================================================

相机位置: (0, 0, 600) mm
靶平面中心: (0, 0, 0) mm

开始生成旋转靶子序列...
帧 1/30: 旋转角 0°, 投影位置: (400, 300) px → 已记录
帧 2/30: 旋转角 12°, 投影位置: (400, 300) px
...
✅ 模式1完成！靶子在原地旋转，相机固定观察。

============================================================
模式2: 靶子绕固定中心旋转（轨道运动）
============================================================
...
```

---

## 核心API使用

### 1. 创建相机和生成器

```cpp
// 创建仿真相机
SimulationCamera camera(800, 600, 30.0f);

// 设置内参
SimulationCamera::CameraIntrinsics intrinsics;
intrinsics.fx = 400.0f;
intrinsics.fy = 400.0f;
intrinsics.cx = 400.0f;
intrinsics.cy = 300.0f;
camera.set_intrinsics(intrinsics);

// 设置位置
SimulationCamera::CameraPose pose;
pose.position = cv::Point3f(0, 0, 600);
pose.rotation = cv::Point3f(0, 0, 0);
camera.set_pose(pose);

// 创建帧生成器
TrainingFrameGenerator generator(800, 600, 30.0f);
```

### 2. 设置五角星旋转模式

```cpp
// 方式1: 直接设置
generator.set_training_mode(
    TrainingFrameGenerator::MODE_PENTAGON_ROTATION
);

// 方式2: 重新生成五角星
generator.regenerate_pentagon(cv::Point2f(-1, -1));  // 随机位置
```

### 3. 生成和投影

```cpp
// 获取训练帧
auto training_frame = generator.get_next_frame();

// 通过相机投影
auto projection = camera.project_training_frame(generator);

// 检查结果
if (projection.is_in_view) {
    std::cout << "靶投影位置: " 
              << projection.projected_positions[0] << std::endl;
    std::cout << "靶世界坐标: " 
              << projection.world_positions[0] << std::endl;
    std::cout << "相机距离: " 
              << projection.distance_to_camera << " mm" << std::endl;
}
```

### 4. 动态相机运动

```cpp
// 每帧更新相机位置
for (int frame = 0; frame < num_frames; ++frame) {
    float t = frame * 0.033f;  // 时间
    
    // 计算新的相机位置
    cv::Point3f new_pos(
        radius * std::cos(t),
        radius * std::sin(t),
        height
    );
    
    // 更新相机
    pose.position = new_pos;
    camera.set_pose(pose);
    
    // 获取投影
    auto result = camera.project_training_frame(generator);
}
```

---

## 关键数据结构

### ProjectionResult

```cpp
struct ProjectionResult {
    cv::Mat projected_frame;  // 投影后的图像
    std::vector<cv::Point2f> projected_positions;  // 像素坐标
    std::vector<cv::Point3f> world_positions;      // 世界坐标
    bool is_in_view;          // 是否在视野内
    float distance_to_camera; // 距离(mm)
};
```

### 五角星靶位置

- **中心**: 五角星中心位置
- **外围5个**: 五角星5个顶点附近的目标
- **颜色编码**: 每个位置有不同的颜色

---

## 典型应用场景

### 场景1: 实时追踪测试

```cpp
// 模式2 (轨道运动) 适用
// 验证追踪器在运动目标上的性能
for (int frame = 0; frame < 40; ++frame) {
    // 更新靶位置
    // 运行追踪算法
    // 收集追踪误差
}
```

### 场景2: 多角度观测

```cpp
// 创建多个相机
vector<SimulationCamera> cameras;
for (int angle = 0; angle < 360; angle += 45) {
    SimulationCamera cam = create_camera_at_angle(angle);
    cameras.push_back(cam);
}

// 同时投影
for (auto& cam : cameras) {
    auto result = cam.project_training_frame(generator);
}
```

### 场景3: 性能基准测试

```cpp
// 模式5 (追踪统计) 适用
// 运行100帧完整场景
// 收集可见率、距离统计等
// 评估算法性能
```

---

## 性能指标

| 指标 | 值 |
|------|-----|
| 总帧数 | 190帧 (5种模式) |
| 编译时间 | ~2秒 |
| 运行时间 | ~0.5秒 |
| 内存占用 | ~50MB |
| 可见率(模式5) | 35% |

---

## 输出文件

测试程序生成的可选输出文件：

```
rotating_target_self_*.jpg       # 自转模式快照
pentagon_rotation_*.jpg          # 五角星投影快照
```

（具体取决于程序中是否启用文件保存）

---

## 调试技巧

### 1. 检查投影是否正确

```cpp
// 输出投影细节
if (projection.is_in_view) {
    std::cout << "投影位置: " << projection.projected_positions[0] << std::endl;
} else {
    std::cout << "不在视野内" << std::endl;
}
```

### 2. 监测距离变化

```cpp
// 记录距离序列
static float prev_distance = 0;
float distance_change = projection.distance_to_camera - prev_distance;
std::cout << "距离变化: " << distance_change << " mm" << std::endl;
prev_distance = projection.distance_to_camera;
```

### 3. 统计可见性

```cpp
// 计算可见率
int visible_count = 0;
for (int i = 0; i < total_frames; ++i) {
    if (projections[i].is_in_view) visible_count++;
}
float visible_rate = 100.0f * visible_count / total_frames;
std::cout << "可见率: " << visible_rate << "%" << std::endl;
```

---

## 后续扩展

### 可能的改进方向

1. **增加运动模式**
   - 螺旋运动
   - 摆动运动
   - 随机游走

2. **集成追踪算法**
   - 测试KalmanFilter
   - 测试粒子滤波
   - 测试卡尔曼-匈牙利追踪

3. **性能优化**
   - 多线程投影
   - GPU加速
   - 缓存优化

4. **数据导出**
   - CSV格式
   - 视频生成
   - 3D可视化

---

## 常见问题

**Q: 为什么模式3显示靶不在视野内？**  
A: 这是正常的。模式3设置的相机和靶位置使得它们距离较远，且相机不直接指向靶。这是用来测试视野管理的。

**Q: 如何改变相机位置？**  
A: 修改 `pose.position` 的值，例如：
```cpp
pose.position = cv::Point3f(300, 300, 500);  // 新位置
camera.set_pose(pose);
```

**Q: 如何改变靶的轨道半径？**  
A: 在相应模式中修改 `radius` 变量，例如：
```cpp
float radius = 300.0f;  // 改为300mm
```

**Q: 如何增加帧数？**  
A: 修改 `num_frames` 变量，例如：
```cpp
int num_frames = 100;  // 增加到100帧
```

---

## 总结

`rotating_pentagon_test` 提供了一个完整的、多维度的仿真测试框架，用于验证基于 `SimulationCamera` 和 `TrainingFrameGenerator` 的3D仿真系统。通过5种不同的运动模式，可以全面评估追踪系统在各种场景下的性能。

**核心优势**:
- ✅ 完整的3D运动仿真
- ✅ 五角星靶子集成
- ✅ 多相机支持
- ✅ 详细的性能统计
- ✅ 易于扩展

---

**最后更新**: 2026年1月19日  
**版本**: 1.0  
**状态**: 生产就绪 ✅
