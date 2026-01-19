# 五角形靶子 3D 旋转投影实时显示

## 程序说明

这是一个实时显示五角形靶子3D旋转投影的程序。

### 配置参数

**靶子位置**：
- 中心坐标：原点 (0, 0, 0)
- 方向：面朝 X 轴正方向（沿X轴方向）

**相机位置**：
- 位置：(0, -100, 300) - 靶子前方 30cm，向下 10cm
- 视角：向下约 20° 的角度
- 这样可以从前上方斜视角观察靶子

**运动**：
- 靶子绕 Y 轴连续旋转
- 使用 TrainingFrameGenerator 生成真实的五角星靶子
- 投影到 3D 空间并从观察相机位置呈现

### 控制按键

| 按键 | 功能 |
|------|------|
| `q` 或 `ESC` | 退出程序 |
| `p` | 暂停/继续旋转 |
| `+` 或 `=` | 增加旋转速度 |
| `-` | 减少旋转速度 |

### 技术细节

**投影管道**：
1. 从 TrainingFrameGenerator 获取生成的靶子 Mat 图像
2. 每个像素反投影到 3D 世界坐标
3. 在 3D 空间中应用旋转变换（绕Y轴）
4. 使用观察相机的投影矩阵投影回2D图像
5. 使用 OpenCV imshow 实时显示

**关键函数**：
- `rotate_point_3d()` - 在3D空间中旋转点
- `reproject_image_3d()` - 完整的投影流程

### 编译和运行

```bash
cd /home/harekasa/Mygo/build
make pentagon_3d_perspective -j4
./bin/pentagon_3d_perspective
```

### 所需库

- OpenCV 4.7.0
- TargetSim (靶子生成)
- TrainingFrameGenerator (帧生成)
- SimulationCamera (相机投影)

## 观察效果

程序运行时会显示：
- 五角形靶子从不同角度旋转
- 显示实时帧数、旋转角度、播放状态
- 由于视角略微向下，可以观察到靶子的深度感和透视效果
- 靶子面朝 X 轴，旋转时会展示不同的侧面

## 文件位置

- 源文件：`/home/harekasa/Mygo/src/apps/pentagon_3d_perspective.cpp`
- 可执行文件：`/home/harekasa/Mygo/build/bin/pentagon_3d_perspective`
