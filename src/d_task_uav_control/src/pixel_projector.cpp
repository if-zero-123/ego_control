#include "d_task_uav_control/pixel_projector.h"

#include <cmath>

namespace d_task_uav_control {

PixelProjector::PixelProjector(const CameraIntrinsics& intrinsics,
                               const Eigen::Matrix3d& body_r_camera,
                               const Eigen::Vector3d& body_t_camera)
    : intrinsics_(intrinsics),
      body_r_camera_(body_r_camera),
      body_t_camera_(body_t_camera) {}

bool PixelProjector::project(double u, double v,
                             const Eigen::Vector3d& world_t_body,
                             const Eigen::Quaterniond& world_q_body,
                             double plane_z,
                             Eigen::Vector3d& world_point) const {
    if (intrinsics_.fx <= 0.0 || intrinsics_.fy <= 0.0) {
        return false;
    }
    Eigen::Vector3d camera_ray(
        (u - intrinsics_.cx) / intrinsics_.fx,
        (v - intrinsics_.cy) / intrinsics_.fy,
        1.0);
    const Eigen::Quaterniond normalised_q = world_q_body.normalized();
    const Eigen::Vector3d world_origin =
        world_t_body + normalised_q * body_t_camera_;
    const Eigen::Vector3d world_ray =
        normalised_q * (body_r_camera_ * camera_ray);
    if (!world_ray.allFinite() || world_ray.z() >= -1e-6) {
        return false;
    }
    const double scale = (plane_z - world_origin.z()) / world_ray.z();
    if (!std::isfinite(scale) || scale <= 0.0) {
        return false;
    }
    world_point = world_origin + scale * world_ray;
    world_point.z() = plane_z;
    return world_point.allFinite();
}

}  // namespace d_task_uav_control
