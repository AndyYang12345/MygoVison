#include "TargetSim/SimulationCamera.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

struct Plane {
    cv::Point3f center;
    cv::Point3f normal;
    float width;
    float height;
    cv::Point3f axis_right;
    cv::Point3f axis_up;
};

struct AppState {
    SimulationCamera* camera = nullptr;
    Plane* plane = nullptr;
    std::vector<cv::Point3f>* clicks = nullptr;
    cv::Point2f last_click_pixel = cv::Point2f(-1.0f, -1.0f);
    cv::Mat homography_inv;
    cv::Size canvas_size;
    bool homography_valid = false;
    float target_pitch = 0.0f;
    float target_yaw = 0.0f;
    bool has_target = false;
    float base_pitch = 0.0f;
    float base_yaw = 0.0f;
    float max_pitch = 0.0f;
    float max_yaw = 0.0f;
};

cv::Point3f add(const cv::Point3f& a, const cv::Point3f& b) {
    return cv::Point3f(a.x + b.x, a.y + b.y, a.z + b.z);
}

cv::Point3f sub(const cv::Point3f& a, const cv::Point3f& b) {
    return cv::Point3f(a.x - b.x, a.y - b.y, a.z - b.z);
}

cv::Point3f scale(const cv::Point3f& v, float s) {
    return cv::Point3f(v.x * s, v.y * s, v.z * s);
}

float dot(const cv::Point3f& a, const cv::Point3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

cv::Point3f cross(const cv::Point3f& a, const cv::Point3f& b) {
    return cv::Point3f(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

float norm(const cv::Point3f& v) {
    return std::sqrt(dot(v, v));
}

cv::Point3f normalize(const cv::Point3f& v) {
    float n = norm(v);
    if (n <= 1e-6f) return cv::Point3f(0, 0, 0);
    return scale(v, 1.0f / n);
}

cv::Point3f look_at_direction(const cv::Point3f& from, const cv::Point3f& to);
void direction_to_pitch_yaw(const cv::Point3f& dir, float& pitch, float& yaw);
float wrap_angle(float angle);

cv::Mat rotation_matrix(float pitch, float yaw, float roll) {
    float cy = std::cos(yaw);
    float sy = std::sin(yaw);
    float cp = std::cos(pitch);
    float sp = std::sin(pitch);
    float cr = std::cos(roll);
    float sr = std::sin(roll);

    cv::Mat r(3, 3, CV_32F);
    r.at<float>(0, 0) = cy * cr + sy * sp * sr;
    r.at<float>(0, 1) = -cy * sr + sy * sp * cr;
    r.at<float>(0, 2) = sy * cp;
    r.at<float>(1, 0) = cp * sr;
    r.at<float>(1, 1) = cp * cr;
    r.at<float>(1, 2) = -sp;
    r.at<float>(2, 0) = -sy * cr + cy * sp * sr;
    r.at<float>(2, 1) = sy * sr + cy * sp * cr;
    r.at<float>(2, 2) = cy * cp;
    return r;
}

void update_plane_axes(Plane& plane) {
    cv::Point3f up(0.0f, 1.0f, 0.0f);
    if (std::abs(dot(plane.normal, up)) > 0.95f) {
        up = cv::Point3f(0.0f, 0.0f, 1.0f);
    }
    plane.axis_right = normalize(cross(plane.normal, up));
    plane.axis_up = normalize(cross(plane.axis_right, plane.normal));
}

std::vector<cv::Point3f> plane_corners_world(const Plane& plane) {
    float half_w = plane.width * 0.5f;
    float half_h = plane.height * 0.5f;
    std::vector<cv::Point3f> corners;
    corners.push_back(add(plane.center, add(scale(plane.axis_right, -half_w), scale(plane.axis_up, -half_h))));
    corners.push_back(add(plane.center, add(scale(plane.axis_right, half_w), scale(plane.axis_up, -half_h))));
    corners.push_back(add(plane.center, add(scale(plane.axis_right, half_w), scale(plane.axis_up, half_h))));
    corners.push_back(add(plane.center, add(scale(plane.axis_right, -half_w), scale(plane.axis_up, half_h))));
    return corners;
}

cv::Point2f world_to_image(SimulationCamera& cam, const cv::Point3f& world) {
    return cam.world_to_image(world);
}

bool world_to_camera_coords(SimulationCamera& cam, const cv::Point3f& world, cv::Point3f& camera_out) {
    const auto& pose = cam.get_pose();
    cv::Mat r = cam.get_rotation_matrix();
    cv::Mat world_vec(3, 1, CV_32F);
    world_vec.at<float>(0) = world.x - pose.position.x;
    world_vec.at<float>(1) = world.y - pose.position.y;
    world_vec.at<float>(2) = world.z - pose.position.z;
    cv::Mat cam_vec = r * world_vec;
    camera_out = cv::Point3f(cam_vec.at<float>(0), cam_vec.at<float>(1), cam_vec.at<float>(2));
    return camera_out.z > 1e-3f;
}

bool pixel_to_world_ray(SimulationCamera& cam, const cv::Point2f& pixel,
                        cv::Point3f& ray_origin, cv::Point3f& ray_dir) {
    const auto& intr = cam.get_intrinsics();
    const auto& pose = cam.get_pose();

    float x = (pixel.x - intr.cx) / intr.fx;
    float y = (pixel.y - intr.cy) / intr.fy;
    cv::Point3f ray_cam(x, y, 1.0f);
    ray_cam = normalize(ray_cam);

    cv::Mat r = cam.get_rotation_matrix();
    cv::Mat r_inv = r.t();
    cv::Mat dir_mat = r_inv * cv::Mat(3, 1, CV_32F, (void*)&ray_cam.x);
    ray_dir = cv::Point3f(dir_mat.at<float>(0), dir_mat.at<float>(1), dir_mat.at<float>(2));
    ray_dir = normalize(ray_dir);
    ray_origin = pose.position;
    return true;
}

bool intersect_ray_plane(const cv::Point3f& ray_origin, const cv::Point3f& ray_dir,
                         const Plane& plane, cv::Point3f& hit_point, float& u, float& v) {
    float denom = dot(plane.normal, ray_dir);
    if (std::abs(denom) < 1e-6f) {
        return false;
    }
    float t = dot(plane.normal, sub(plane.center, ray_origin)) / denom;
    if (t <= 0.0f) {
        return false;
    }
    hit_point = add(ray_origin, scale(ray_dir, t));
    cv::Point3f d = sub(hit_point, plane.center);
    u = dot(d, plane.axis_right);
    v = dot(d, plane.axis_up);
    return true;
}

cv::Point3f forward_direction_world(SimulationCamera& cam) {
    cv::Mat r = cam.get_rotation_matrix();
    cv::Mat r_inv = r.t();
    cv::Point3f forward_cam(0.0f, 0.0f, 1.0f);
    cv::Mat dir_mat = r_inv * cv::Mat(3, 1, CV_32F, (void*)&forward_cam.x);
    cv::Point3f dir(dir_mat.at<float>(0), dir_mat.at<float>(1), dir_mat.at<float>(2));
    return normalize(dir);
}

void mouse_callback(int event, int x, int y, int, void* userdata) {
    if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }

    auto* state = reinterpret_cast<AppState*>(userdata);
    if (!state || !state->camera || !state->plane) {
        return;
    }
    cv::Point3f ray_origin;
    cv::Point3f ray_dir;
    pixel_to_world_ray(*state->camera, cv::Point2f(static_cast<float>(x), static_cast<float>(y)),
                       ray_origin, ray_dir);

    float u = 0.0f;
    float v = 0.0f;
    cv::Point3f hit;
    if (!intersect_ray_plane(ray_origin, ray_dir, *state->plane, hit, u, v)) {
        return;
    }
    float half_w = state->plane->width * 0.5f;
    float half_h = state->plane->height * 0.5f;
    if (std::abs(u) > half_w || std::abs(v) > half_h) {
        return;
    }

    state->last_click_pixel = cv::Point2f(static_cast<float>(x), static_cast<float>(y));
    state->clicks->push_back(hit);

    direction_to_pitch_yaw(ray_dir, state->target_pitch, state->target_yaw);
    state->has_target = true;
}

void draw_text_shadow(cv::Mat& frame, const std::string& text, const cv::Point& org,
                      double scale, const cv::Scalar& color) {
    cv::putText(frame, text, org + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX,
                scale, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame, text, org, cv::FONT_HERSHEY_SIMPLEX,
                scale, color, 2);
}

cv::Point3f look_at_direction(const cv::Point3f& from, const cv::Point3f& to) {
    return normalize(sub(to, from));
}

void direction_to_pitch_yaw(const cv::Point3f& dir, float& pitch, float& yaw) {
    yaw = std::atan2(dir.x, dir.z);
    float horiz = std::sqrt(dir.x * dir.x + dir.z * dir.z);
    pitch = -std::atan2(dir.y, horiz);
}

float wrap_angle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

int main() {
    const int cam_width = 640;
    const int cam_height = 480;
    SimulationCamera sim_cam(cam_width, cam_height, 30.0f);

    SimulationCamera::CameraIntrinsics cam_intr;
    cam_intr.fx = cam_width * 0.9f;
    cam_intr.fy = cam_width * 0.9f;
    cam_intr.cx = cam_width * 0.5f;
    cam_intr.cy = cam_height * 0.5f;
    sim_cam.set_intrinsics(cam_intr);

    cv::Point3f camera_pos(0.0f, 0.0f, -80.0f);
    cv::Point3f plane_center(0.0f, 0.0f, 0.0f);

    Plane plane;
    plane.center = plane_center;
    plane.normal = normalize(sub(camera_pos, plane_center));
    plane.width = 40.0f;
    plane.height = 40.0f;
    update_plane_axes(plane);

    cv::Point3f dir = look_at_direction(camera_pos, plane_center);
    float base_pitch = 0.0f;
    float base_yaw = 0.0f;
    direction_to_pitch_yaw(dir, base_pitch, base_yaw);

    SimulationCamera::CameraPose cam_pose;
    cam_pose.position = camera_pos;
    cam_pose.rotation = cv::Point3f(-base_pitch, -base_yaw, 0.0f);
    sim_cam.set_pose(cam_pose);

    cv::Mat canvas_img(360, 360, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::rectangle(canvas_img, cv::Rect(6, 6, 348, 348), cv::Scalar(210, 210, 210), 2);
    const int stripe = 24;
    for (int x = 0; x < canvas_img.cols; x += stripe) {
        if ((x / stripe) % 2 == 0) {
            cv::rectangle(canvas_img, cv::Rect(x, 0, stripe / 2, canvas_img.rows),
                          cv::Scalar(235, 235, 235), -1);
        }
    }
    for (int y = 0; y < canvas_img.rows; y += stripe) {
        if ((y / stripe) % 2 == 0) {
            cv::rectangle(canvas_img, cv::Rect(0, y, canvas_img.cols, stripe / 2),
                          cv::Scalar(245, 245, 245), -1);
        }
    }
    const cv::Point axis_center(canvas_img.cols / 2, canvas_img.rows / 2);
    cv::arrowedLine(canvas_img, axis_center,
                    cv::Point(canvas_img.cols - 12, axis_center.y),
                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.05);
    cv::arrowedLine(canvas_img, axis_center,
                    cv::Point(axis_center.x, 12),
                    cv::Scalar(0, 180, 0), 2, cv::LINE_AA, 0, 0.05);
    cv::putText(canvas_img, "X", cv::Point(canvas_img.cols - 24, axis_center.y - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 200), 2);
    cv::putText(canvas_img, "Y", cv::Point(axis_center.x + 8, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 150, 0), 2);

    std::vector<cv::Point3f> clicked_points;

    AppState state;
    state.camera = &sim_cam;
    state.plane = &plane;
    state.clicks = &clicked_points;
    state.base_pitch = base_pitch;
    state.base_yaw = base_yaw;
    state.max_pitch = 10.0f * static_cast<float>(CV_PI) / 180.0f;
    state.max_yaw = 15.0f * static_cast<float>(CV_PI) / 180.0f;
    state.canvas_size = canvas_img.size();

    cv::namedWindow("Camera View", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Camera View", mouse_callback, &state);

    bool rotating = false;
    float current_pitch = base_pitch;
    float current_yaw = base_yaw;
    const float rotate_step = 0.005f;
    const float manual_step = 0.02f;

    std::cout << "Left click on Camera View to mark a point." << std::endl;
    std::cout << "Press SPACE to rotate camera to target angle. Press R to reset." << std::endl;
    std::cout << "Press Q or ESC to quit." << std::endl;

    while (true) {
        if (rotating && state.has_target) {
            float dp = wrap_angle(state.target_pitch - current_pitch);
            float dy = wrap_angle(state.target_yaw - current_yaw);

            if (std::abs(dp) < rotate_step && std::abs(dy) < rotate_step) {
                current_pitch = state.target_pitch;
                current_yaw = state.target_yaw;
                rotating = false;
            } else {
                current_pitch += std::max(std::min(dp, rotate_step), -rotate_step);
                current_yaw += std::max(std::min(dy, rotate_step), -rotate_step);
            }
            SimulationCamera::CameraPose pose = sim_cam.get_pose();
            pose.rotation = cv::Point3f(-current_pitch, -current_yaw, 0.0f);
            sim_cam.set_pose(pose);
        }

        cv::Mat camera_view(cam_height, cam_width, CV_8UC3, cv::Scalar(70, 70, 70));

        std::vector<cv::Point3f> corners_world = plane_corners_world(plane);
        std::vector<cv::Point2f> corners_cam;
        bool all_in_front = true;
        for (const auto& corner : corners_world) {
            cv::Point3f cam_pt;
            if (!world_to_camera_coords(sim_cam, corner, cam_pt)) {
                all_in_front = false;
            }
            corners_cam.push_back(world_to_image(sim_cam, corner));
        }

        if (all_in_front) {
            std::vector<cv::Point2f> src_pts = {
                cv::Point2f(0.0f, 0.0f),
                cv::Point2f(static_cast<float>(canvas_img.cols - 1), 0.0f),
                cv::Point2f(static_cast<float>(canvas_img.cols - 1), static_cast<float>(canvas_img.rows - 1)),
                cv::Point2f(0.0f, static_cast<float>(canvas_img.rows - 1))
            };
            cv::Mat M = cv::getPerspectiveTransform(src_pts, corners_cam);
            cv::Mat inv_M;
            if (cv::invert(M, inv_M)) {
                state.homography_inv = inv_M;
                state.homography_valid = true;
            } else {
                state.homography_valid = false;
            }
            cv::Mat warped;
            cv::warpPerspective(canvas_img, warped, M, camera_view.size(), cv::INTER_LINEAR,
                                cv::BORDER_CONSTANT, cv::Scalar(70, 70, 70));
            cv::Mat mask(canvas_img.size(), CV_8UC1, cv::Scalar(255));
            cv::Mat warped_mask;
            cv::warpPerspective(mask, warped_mask, M, camera_view.size());
            warped.copyTo(camera_view, warped_mask);
        } else {
            state.homography_valid = false;
        }

        for (const auto& world_pt : clicked_points) {
            cv::Point2f img_pt = world_to_image(sim_cam, world_pt);
            cv::circle(camera_view, img_pt, 5, cv::Scalar(0, 0, 255), -1);
        }

        if (state.last_click_pixel.x >= 0.0f) {
            std::string click_text = "Click: (" +
                                     std::to_string(static_cast<int>(state.last_click_pixel.x)) +
                                     ", " +
                                     std::to_string(static_cast<int>(state.last_click_pixel.y)) + ")";
            draw_text_shadow(camera_view, click_text, cv::Point(15, 30), 0.6, cv::Scalar(255, 255, 255));
            if (state.has_target) {
                std::string mapped_text = "Click->pitch/yaw: " +
                                          std::to_string(static_cast<int>(state.target_pitch * 180.0f / CV_PI)) +
                                          ", " +
                                          std::to_string(static_cast<int>(state.target_yaw * 180.0f / CV_PI));
                draw_text_shadow(camera_view, mapped_text, cv::Point(15, 52), 0.55, cv::Scalar(220, 255, 220));
            }
        }

        {
            std::string angle_text = "Current pitch/yaw: " +
                                     std::to_string(static_cast<int>(current_pitch * 180.0f / CV_PI)) +
                                     ", " +
                                     std::to_string(static_cast<int>(current_yaw * 180.0f / CV_PI));
            draw_text_shadow(camera_view, angle_text, cv::Point(15, 76), 0.5, cv::Scalar(200, 220, 255));
        }
        if (state.has_target) {
            std::string angle_text = "Target pitch/yaw: " +
                                     std::to_string(static_cast<int>(state.target_pitch * 180.0f / CV_PI)) +
                                     ", " +
                                     std::to_string(static_cast<int>(state.target_yaw * 180.0f / CV_PI));
            draw_text_shadow(camera_view, angle_text, cv::Point(15, 98), 0.5, cv::Scalar(200, 255, 200));

            float raw_dp = state.target_pitch - current_pitch;
            float raw_dy = state.target_yaw - current_yaw;
            std::string delta_text = "Delta pitch/yaw: " +
                                     std::to_string(static_cast<int>(raw_dp * 180.0f / CV_PI)) +
                                     ", " +
                                     std::to_string(static_cast<int>(raw_dy * 180.0f / CV_PI));
            draw_text_shadow(camera_view, delta_text, cv::Point(15, 120), 0.5, cv::Scalar(255, 220, 180));
        }

        cv::Point3f forward_world = forward_direction_world(sim_cam);
        cv::Point3f hit_point;
        float hit_u = 0.0f;
        float hit_v = 0.0f;
        if (intersect_ray_plane(camera_pos, forward_world, plane, hit_point, hit_u, hit_v)) {
            float half_w = plane.width * 0.5f;
            float half_h = plane.height * 0.5f;
            if (std::abs(hit_u) <= half_w && std::abs(hit_v) <= half_h) {
                cv::Point2f hit_cam = world_to_image(sim_cam, hit_point);
                cv::circle(camera_view, hit_cam, 4, cv::Scalar(0, 255, 0), -1);
            }
        }

        cv::imshow("Camera View", camera_view);

        int key = cv::waitKey(16);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
        if (key == ' ') {
            if (state.has_target) {
                rotating = true;
            }
        }
        if (key == 'w' || key == 'W') {
            current_pitch -= manual_step;
            SimulationCamera::CameraPose pose = sim_cam.get_pose();
            pose.rotation = cv::Point3f(-current_pitch, -current_yaw, 0.0f);
            sim_cam.set_pose(pose);
            rotating = false;
        }
        if (key == 's' || key == 'S') {
            current_pitch += manual_step;
            SimulationCamera::CameraPose pose = sim_cam.get_pose();
            pose.rotation = cv::Point3f(-current_pitch, -current_yaw, 0.0f);
            sim_cam.set_pose(pose);
            rotating = false;
        }
        if (key == 'a' || key == 'A') {
            current_yaw += manual_step;
            SimulationCamera::CameraPose pose = sim_cam.get_pose();
            pose.rotation = cv::Point3f(-current_pitch, -current_yaw, 0.0f);
            sim_cam.set_pose(pose);
            rotating = false;
        }
        if (key == 'd' || key == 'D') {
            current_yaw -= manual_step;
            SimulationCamera::CameraPose pose = sim_cam.get_pose();
            pose.rotation = cv::Point3f(-current_pitch, -current_yaw, 0.0f);
            sim_cam.set_pose(pose);
            rotating = false;
        }
        if (key == 'r' || key == 'R') {
            current_pitch = base_pitch;
            current_yaw = base_yaw;
            SimulationCamera::CameraPose pose = sim_cam.get_pose();
            pose.rotation = cv::Point3f(-base_pitch, -base_yaw, 0.0f);
            sim_cam.set_pose(pose);
            rotating = false;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
