#ifndef TARGET3DGENERATOR_HPP
#define TARGET3DGENERATOR_HPP


#include "TargetSim/SimulationCamera.hpp"


/**
 * @class Target3DGenerator
 * @brief 使用TrainingFrameGenerator生成随机圆点靶并投影到3D空间
 */
class Target3DGenerator {
public:
    explicit Target3DGenerator(int width = 800, int height = 600, float fps = 30.0f);

    void set_random_interval(float seconds);

    void set_camera_intrinsics(const SimulationCamera::CameraIntrinsics& intrinsics);
    void set_camera_pose(const SimulationCamera::CameraPose& pose);
    void set_target_plane(const cv::Point3f& plane_normal, float distance);

    SimulationCamera::ProjectionResult generate_projected_frame();

    TrainingFrameGenerator& generator() { return _generator; }
    const TrainingFrameGenerator& generator() const { return _generator; }
    SimulationCamera& camera() { return _camera; }
    const SimulationCamera& camera() const { return _camera; }

private:
    TrainingFrameGenerator _generator;
    SimulationCamera _camera;
    float _random_interval;
};

#endif // TARGET3DGENERATOR_HPP
