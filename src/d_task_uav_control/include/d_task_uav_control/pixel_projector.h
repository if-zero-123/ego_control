#ifndef D_TASK_UAV_CONTROL_PIXEL_PROJECTOR_H_
#define D_TASK_UAV_CONTROL_PIXEL_PROJECTOR_H_

#include <Eigen/Geometry>

namespace d_task_uav_control {

struct CameraIntrinsics {
    double fx = 1.0;
    double fy = 1.0;
    double cx = 0.0;
    double cy = 0.0;
};

class PixelProjector {
public:
    PixelProjector(const CameraIntrinsics& intrinsics,
                   const Eigen::Matrix3d& body_r_camera,
                   const Eigen::Vector3d& body_t_camera);

    bool project(double u, double v,
                 const Eigen::Vector3d& world_t_body,
                 const Eigen::Quaterniond& world_q_body,
                 double plane_z,
                 Eigen::Vector3d& world_point) const;

private:
    CameraIntrinsics intrinsics_;
    Eigen::Matrix3d body_r_camera_;
    Eigen::Vector3d body_t_camera_;
};

}  // namespace d_task_uav_control

#endif  // D_TASK_UAV_CONTROL_PIXEL_PROJECTOR_H_
