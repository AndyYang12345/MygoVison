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
    void set_laser_model(bool enabled,
                         float offset_left_mm = 0.0f,
                         float dot_radius_px = 4.0f,
                         float offset_up_mm = 20.0f);

    SimulationCamera::ProjectionResult generate_projected_frame();
    bool has_last_laser_point() const { return _has_last_laser; }
    cv::Point2f get_last_laser_point() const { return _last_laser_projected; }

    TrainingFrameGenerator& generator() { return _generator; }
    const TrainingFrameGenerator& generator() const { return _generator; }
    SimulationCamera& camera() { return _camera; }
    const SimulationCamera& camera() const { return _camera; }

private:
    TrainingFrameGenerator _generator;
    SimulationCamera _camera;
    float _random_interval;
    bool _laser_enabled;
    float _laser_offset_left_mm;
    float _laser_offset_up_mm;
    float _laser_dot_radius_px;
    bool _has_last_laser;
    cv::Point2f _last_laser_projected;
};

#endif // TARGET3DGENERATOR_HPP
