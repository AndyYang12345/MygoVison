#include "TargetSim/Target3DGenerator.hpp"

Target3DGenerator::Target3DGenerator(int width, int height, float fps)
    : _generator(width, height, fps),
      _camera(width, height, fps),
      _random_interval(0.6f) {
    _generator.set_training_mode(TrainingFrameGenerator::MODE_RANDOM_APPEARANCE, _random_interval);
}

void Target3DGenerator::set_random_interval(float seconds) {
    _random_interval = seconds;
    _generator.set_training_mode(TrainingFrameGenerator::MODE_RANDOM_APPEARANCE, _random_interval);
}

void Target3DGenerator::set_camera_intrinsics(const SimulationCamera::CameraIntrinsics& intrinsics) {
    _camera.set_intrinsics(intrinsics);
}

void Target3DGenerator::set_camera_pose(const SimulationCamera::CameraPose& pose) {
    _camera.set_pose(pose);
}

void Target3DGenerator::set_target_plane(const cv::Point3f& plane_normal, float distance) {
    _camera.set_target_plane(plane_normal, distance);
}

SimulationCamera::ProjectionResult Target3DGenerator::generate_projected_frame() {
    return _camera.project_training_frame(_generator);
}
