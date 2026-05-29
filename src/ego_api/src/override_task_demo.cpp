#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "ego_api/ego_api.h"

namespace {

// D435 深度图穿框 override 节点：
// 主流程触发 task 1/3 后，本节点临时接管速度控制，完成“找门洞 -> 对准 -> 靠近 -> 穿越”。
// 二维码和气球任务当前只保留函数入口，方便后续替换成真实视觉/执行机构逻辑。
template <typename T>
T clampValue(T v, T lo, T hi) {
    return std::max(lo, std::min(v, hi));
}

double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

class DepthFramePasser {
public:
    DepthFramePasser(ros::NodeHandle& nh, EgoApi& api)
        : nh_(nh), pnh_("~"), it_(nh), api_(api) {
        // 默认使用仿真中的 D435 深度图和内参话题；实机只需要在 launch 中改参数。
        pnh_.param<std::string>("depth_topic", depth_topic_, "/d435/depth/image_raw");
        pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/d435/depth/camera_info");
        pnh_.param<double>("flight_z", flight_z_, 1.1);
        pnh_.param<double>("depth_min", depth_min_, 0.35);
        pnh_.param<double>("depth_near_threshold", depth_near_threshold_, 1.8);
        pnh_.param<double>("depth_max", depth_max_, 3.0);
        pnh_.param<double>("align_threshold_m", align_threshold_m_, 0.06);
        pnh_.param<double>("align_hold_time", align_hold_time_, 0.5);
        pnh_.param<double>("search_timeout", search_timeout_, 15.0);
        pnh_.param<double>("align_timeout", align_timeout_, 20.0);
        pnh_.param<double>("approach_distance", approach_distance_, 0.60);
        pnh_.param<double>("approach_timeout", approach_timeout_, 15.0);
        pnh_.param<double>("pass_distance", pass_distance_, 1.35);
        pnh_.param<double>("pass_timeout", pass_timeout_, 12.0);
        pnh_.param<double>("approach_vx", approach_vx_, 0.12);
        pnh_.param<double>("pass_vx", pass_vx_, 0.18);
        pnh_.param<double>("k_y", k_y_, 0.8);
        pnh_.param<double>("k_z", k_z_, 0.8);
        pnh_.param<double>("k_yaw", k_yaw_, 0.5);
        pnh_.param<double>("max_vy", max_vy_, 0.25);
        pnh_.param<double>("max_vz", max_vz_, 0.18);
        pnh_.param<double>("max_yaw_rate", max_yaw_rate_, 0.25);
        pnh_.param<int>("min_open_width_px", min_open_width_px_, 70);
        pnh_.param<int>("min_open_height_px", min_open_height_px_, 70);

        depth_sub_ = it_.subscribe(depth_topic_, 1, &DepthFramePasser::depthCb, this);
        info_sub_ = nh_.subscribe(camera_info_topic_, 1, &DepthFramePasser::infoCb, this);
        ROS_INFO("[frame_pass] depth=%s camera_info=%s", depth_topic_.c_str(), camera_info_topic_.c_str());
    }

    bool passFrame(const std::string& label) {
        ROS_INFO("[frame_pass] ===== %s: DETECT =====", label.c_str());
        // 先看到稳定门洞再进入对准，避免盲目前进。
        if (!waitForDetection(search_timeout_)) {
            ROS_ERROR("[frame_pass] %s: no depth gate detection", label.c_str());
            hold(1.0);
            return false;
        }

        ROS_INFO("[frame_pass] ===== %s: ALIGN =====", label.c_str());
        // 对准阶段只修正 lateral / height / yaw，不向前冲。
        if (!align(label)) {
            ROS_ERROR("[frame_pass] %s: align failed", label.c_str());
            hold(1.0);
            return false;
        }

        ROS_INFO("[frame_pass] ===== %s: APPROACH =====", label.c_str());
        approach(label);

        ROS_INFO("[frame_pass] ===== %s: PASS =====", label.c_str());
        passThrough(label);

        hold(0.5);
        ROS_INFO("[frame_pass] %s: done", label.c_str());
        return true;
    }

private:
    struct Detection {
        bool found = false;
        // err_y/err_z 是门洞中心相对相机光轴的米级偏差，直接用于速度闭环。
        double err_y = 0.0;
        double err_z = 0.0;
        double dist = std::numeric_limits<double>::quiet_NaN();
        double yaw_err = 0.0;
        ros::Time stamp;
    };

    void infoCb(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        // 深度像素误差需要相机内参才能换算成实际横向/高度偏差。
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        have_info_ = fx_ > 1.0 && fy_ > 1.0;
    }

    void depthCb(const sensor_msgs::ImageConstPtr& msg) {
        if (!have_info_) return;

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg);
        } catch (const cv_bridge::Exception& e) {
            ROS_WARN_THROTTLE(1.0, "[frame_pass] cv_bridge error: %s", e.what());
            return;
        }

        Detection det;
        det.stamp = msg->header.stamp;
        if (processDepth(cv_ptr->image, det)) {
            latest_ = det;
            have_detection_ = true;
        } else {
            latest_.found = false;
            latest_.stamp = msg->header.stamp;
        }
    }

    bool processDepth(const cv::Mat& depth, Detection& det) const {
        if (depth.empty()) return false;

        // 只在画面中央大区域搜索门洞，避开边缘畸变和机体/地面干扰。
        const int width = depth.cols;
        const int height = depth.rows;
        const int x0 = width / 5;
        const int x1 = width - x0;
        const int y0 = height / 5;
        const int y1 = height - y0 / 2;

        int best_area = 0;
        int best_u0 = 0;
        int best_u1 = 0;
        int best_v0 = 0;
        int best_v1 = 0;

        // 逐行找最大“空区”：门洞内部通常是远距离或无效深度，门框两侧是近距离深度。
        for (int v = y0; v < y1; ++v) {
            int run_start = -1;
            for (int u = x0; u <= x1; ++u) {
                const bool at_end = (u == x1);
                const bool open = !at_end && isOpenDepth(depth, u, v);
                if (open && run_start < 0) {
                    run_start = u;
                }
                if ((!open || at_end) && run_start >= 0) {
                    const int run_end = u - 1;
                    const int run_width = run_end - run_start + 1;
                    if (run_width >= min_open_width_px_) {
                        int vv0 = v;
                        int vv1 = v;
                        growVertical(depth, run_start, run_end, y0, y1, vv0, vv1);
                        const int run_height = vv1 - vv0 + 1;
                        const int area = run_width * run_height;
                        if (run_height >= min_open_height_px_ && area > best_area) {
                            best_area = area;
                            best_u0 = run_start;
                            best_u1 = run_end;
                            best_v0 = vv0;
                            best_v1 = vv1;
                        }
                    }
                    run_start = -1;
                }
            }
        }

        if (best_area <= 0) return false;

        const double u_center = 0.5 * (best_u0 + best_u1);
        const double v_center = 0.5 * (best_v0 + best_v1);
        const double dist = estimateFrameDistance(depth, best_u0, best_u1, best_v0, best_v1);
        if (!std::isfinite(dist)) return false;

        // 用针孔模型把门洞中心像素误差换成米级控制误差。
        det.err_y = (u_center - cx_) / fx_ * dist;
        det.err_z = -(v_center - cy_) / fy_ * dist;
        det.dist = dist;
        det.yaw_err = 0.0;
        det.found = true;
        return true;
    }

    bool isOpenDepth(const cv::Mat& depth, int u, int v) const {
        const double d = readDepthMeters(depth, u, v);
        // D435 穿透门洞时，洞内可能读到远处背景，也可能直接无深度，两者都认为是 open。
        if (!std::isfinite(d)) return true;
        return d > depth_near_threshold_ && d < depth_max_;
    }

    double readDepthMeters(const cv::Mat& depth, int u, int v) const {
        if (u < 0 || v < 0 || u >= depth.cols || v >= depth.rows) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (depth.type() == CV_16UC1) {
            const uint16_t raw = depth.at<uint16_t>(v, u);
            if (raw == 0) return std::numeric_limits<double>::quiet_NaN();
            return static_cast<double>(raw) * 0.001;
        }
        if (depth.type() == CV_32FC1) {
            const float raw = depth.at<float>(v, u);
            if (!std::isfinite(raw) || raw <= 0.0f) return std::numeric_limits<double>::quiet_NaN();
            return static_cast<double>(raw);
        }
        return std::numeric_limits<double>::quiet_NaN();
    }

    void growVertical(const cv::Mat& depth,
                      int u0,
                      int u1,
                      int y_min,
                      int y_max,
                      int& v0,
                      int& v1) const {
        const int step = std::max(1, (u1 - u0 + 1) / 12);
        // 竖向扩展时只抽样若干列，减少每帧计算量。
        auto rowOpenRatio = [&](int v) {
            int open = 0;
            int total = 0;
            for (int u = u0; u <= u1; u += step) {
                ++total;
                if (isOpenDepth(depth, u, v)) ++open;
            }
            return total > 0 ? static_cast<double>(open) / total : 0.0;
        };

        while (v0 > y_min && rowOpenRatio(v0 - 1) > 0.65) --v0;
        while (v1 < y_max - 1 && rowOpenRatio(v1 + 1) > 0.65) ++v1;
    }

    double estimateFrameDistance(const cv::Mat& depth,
                                 int u0,
                                 int u1,
                                 int v0,
                                 int v1) const {
        std::vector<double> samples;
        const int margin = 8;
        const int step = 4;
        // 距离取门洞左右边缘附近的近深度中位数，比直接取洞内深度更稳定。
        const int left0 = std::max(0, u0 - margin * 3);
        const int left1 = std::max(0, u0 - margin);
        const int right0 = std::min(depth.cols - 1, u1 + margin);
        const int right1 = std::min(depth.cols - 1, u1 + margin * 3);
        for (int v = v0; v <= v1; v += step) {
            for (int u = left0; u <= left1; u += step) collectNearDepth(depth, u, v, samples);
            for (int u = right0; u <= right1; u += step) collectNearDepth(depth, u, v, samples);
        }
        if (samples.empty()) return std::numeric_limits<double>::quiet_NaN();
        std::nth_element(samples.begin(), samples.begin() + samples.size() / 2, samples.end());
        return samples[samples.size() / 2];
    }

    void collectNearDepth(const cv::Mat& depth, int u, int v, std::vector<double>& samples) const {
        const double d = readDepthMeters(depth, u, v);
        if (std::isfinite(d) && d > depth_min_ && d < depth_near_threshold_) {
            samples.push_back(d);
        }
    }

    bool waitForDetection(double timeout) {
        ros::Rate rate(30);
        const ros::Time start = ros::Time::now();
        while (ros::ok()) {
            ros::spinOnce();
            if (freshDetection()) return true;
            if ((ros::Time::now() - start).toSec() > timeout) return false;
            rate.sleep();
        }
        return false;
    }

    bool freshDetection() const {
        return have_detection_ && latest_.found && (ros::Time::now() - latest_.stamp).toSec() < 0.5;
    }

    void sendServo(double vx, const Detection& det) {
        // 速度闭环只给小速度，穿框阶段以鲁棒和可中断为优先。
        const double vy = clampValue(-k_y_ * det.err_y, -max_vy_, max_vy_);
        const double vz = clampValue(k_z_ * det.err_z, -max_vz_, max_vz_);
        const double yaw_rate = clampValue(-k_yaw_ * det.yaw_err, -max_yaw_rate_, max_yaw_rate_);
        api_.sendVelocityCmd(vx, vy, vz, yaw_rate);
    }

    bool align(const std::string& label) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time stable_since;
        bool stable_started = false;

        while (ros::ok()) {
            ros::spinOnce();
            if (!freshDetection()) {
                // 丢失门洞时立即悬停，不继续累积“已经对准”的时间。
                api_.holdPosition();
                stable_started = false;
            } else {
                sendServo(0.0, latest_);
                const bool centered = std::abs(latest_.err_y) < align_threshold_m_ &&
                                      std::abs(latest_.err_z) < align_threshold_m_;
                if (centered && !stable_started) {
                    stable_started = true;
                    stable_since = ros::Time::now();
                } else if (!centered) {
                    stable_started = false;
                }
                if (stable_started && (ros::Time::now() - stable_since).toSec() > align_hold_time_) {
                    ROS_INFO("[frame_pass] %s centered err_y=%.3f err_z=%.3f dist=%.3f",
                             label.c_str(), latest_.err_y, latest_.err_z, latest_.dist);
                    return true;
                }
            }

            if ((ros::Time::now() - start).toSec() > align_timeout_) return false;
            rate.sleep();
        }
        return false;
    }

    void approach(const std::string&) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        while (ros::ok()) {
            ros::spinOnce();
            if (freshDetection()) {
                // 靠近阶段仍持续修正门洞中心，直到距离足够近。
                sendServo(approach_vx_, latest_);
                if (latest_.dist < approach_distance_) return;
            } else {
                api_.sendVelocityCmd(0.0, 0.0, 0.0, 0.0);
            }
            if ((ros::Time::now() - start).toSec() > approach_timeout_) return;
            rate.sleep();
        }
    }

    void passThrough(const std::string&) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        const Eigen::Vector3d start_pos = api_.getOdomPosition();
        const double yaw = api_.getOdomYaw();
        const Eigen::Vector2d dir(std::cos(yaw), std::sin(yaw));

        while (ros::ok()) {
            ros::spinOnce();
            if (freshDetection()) {
                // 穿越中能看到门洞就继续闭环修正，看不到时保持低速直行完成最后一段。
                sendServo(pass_vx_, latest_);
            } else {
                api_.sendVelocityCmd(pass_vx_, 0.0, 0.0, 0.0);
            }

            const Eigen::Vector3d now = api_.getOdomPosition();
            const Eigen::Vector2d delta(now.x() - start_pos.x(), now.y() - start_pos.y());
            if (delta.dot(dir) > pass_distance_) return;
            if ((ros::Time::now() - start).toSec() > pass_timeout_) return;
            rate.sleep();
        }
    }

    void hold(double seconds) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < seconds) {
            ros::spinOnce();
            api_.holdPosition();
            rate.sleep();
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    EgoApi& api_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber info_sub_;

    std::string depth_topic_;
    std::string camera_info_topic_;
    double flight_z_ = 1.1;
    double depth_min_ = 0.35;
    double depth_near_threshold_ = 1.8;
    double depth_max_ = 3.0;
    double align_threshold_m_ = 0.06;
    double align_hold_time_ = 0.5;
    double search_timeout_ = 15.0;
    double align_timeout_ = 20.0;
    double approach_distance_ = 0.60;
    double approach_timeout_ = 15.0;
    double pass_distance_ = 1.35;
    double pass_timeout_ = 12.0;
    double approach_vx_ = 0.12;
    double pass_vx_ = 0.18;
    double k_y_ = 0.8;
    double k_z_ = 0.8;
    double k_yaw_ = 0.5;
    double max_vy_ = 0.25;
    double max_vz_ = 0.18;
    double max_yaw_rate_ = 0.25;
    int min_open_width_px_ = 70;
    int min_open_height_px_ = 70;

    bool have_info_ = false;
    bool have_detection_ = false;
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    Detection latest_;
};

void holdPlaceholder(EgoApi& api, const std::string& label, const std::string& message) {
    // 预留任务入口：先悬停并打印日志，后续把这里替换成二维码/气球真实任务。
    ROS_WARN("[%s] %s", label.c_str(), message.c_str());
    ros::Rate rate(30);
    const ros::Time start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start).toSec() < 3.0) {
        ros::spinOnce();
        api.holdPosition();
        rate.sleep();
    }
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "override_task_demo");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string bridge_ns;
    pnh.param<std::string>("bridge_ns", bridge_ns, "/ego_bridge");

    EgoApi api(nh, bridge_ns);
    DepthFramePasser frame_passer(nh, api);

    ROS_INFO("[override] CRAIC override node started, waiting for trigger...");

    while (ros::ok()) {
        const int task_id = api.waitForOverrideTrigger(0);
        if (task_id < 0) continue;

        ROS_INFO("[override] ===== Received task %d =====", task_id);
        if (!api.enableOverride()) {
            ROS_ERROR("[override] Failed to enable override, skip task %d", task_id);
            continue;
        }

        // task 分配：
        // 1 去程 D435 穿框；2 二维码占位；3 返程 D435 穿框；4 气球占位。
        switch (task_id) {
            case 1:
                frame_passer.passFrame("task1_pass_frame_forward");
                break;
            case 2:
                holdPlaceholder(api, "task2_qr_placeholder",
                                "QR recognition placeholder: connect downward camera and OpenCV QR later.");
                break;
            case 3:
                frame_passer.passFrame("task3_pass_frame_backward");
                break;
            case 4:
                holdPlaceholder(api, "task4_balloon_placeholder",
                                "Balloon attack placeholder: connect red balloon detector and puncture servo later.");
                break;
            default:
                holdPlaceholder(api, "task_reserve", "Reserved task placeholder.");
                break;
        }

        api.disableOverride();
        ROS_INFO("[override] Task %d complete, control returned.\n", task_id);
    }

    return 0;
}
