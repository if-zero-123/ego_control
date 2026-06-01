#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "ego_api/ego_api.h"

namespace {

// 将 yaw 规整到 [-pi, pi]，避免返程转向时角度跳变过大。
double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

double clampValue(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

Eigen::Vector3d posePosition(const geometry_msgs::PoseStamped& pose) {
    return Eigen::Vector3d(pose.pose.position.x,
                           pose.pose.position.y,
                           pose.pose.position.z);
}

// 从 ROS 位姿四元数中提取平面 yaw，EGO-Planner 目标点只需要平面朝向。
double yawFromPose(const geometry_msgs::PoseStamped& pose) {
    const auto& q = pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool validFrameStatus(const std::string& status) {
    return status == "valid_full" ||
           status == "valid_weak_pair" ||
           status == "valid_split_frame" ||
           status == "valid_partial_left" ||
           status == "valid_partial_right";
}

bool preferredFrameStatus(const std::string& status) {
    return status == "valid_full";
}

// CRAIC 第一版主流程：
// 1. 起飞并记录起飞后悬停点作为击打区返回点。
// 2. 等待 MID360/FAST-LIO 柱子识别节点给出稳定 pre/post goal。
// 3. 用 EGO-Planner 真实飞过柱子通道。
// 4. 等待 MID360 窗框识别锁定一次 pre/post，用 EGO 直接穿框并在返程复用记录点。
// 5. 二维码和气球任务先保留入口，后续替换成真实识别和执行逻辑。
class CraicCompetitionDemo {
public:
    CraicCompetitionDemo()
        : nh_(), pnh_("~"), api_(nh_, getBridgeNs()) {
        pnh_.param<double>("flight_z", flight_z_, 0.9);
        pnh_.param<double>("takeoff_timeout", takeoff_timeout_, 30.0);
        pnh_.param<double>("goal_timeout", goal_timeout_, 60.0);
        pnh_.param<double>("override_timeout", override_timeout_, 120.0);
        pnh_.param<double>("pillar_detect_timeout", pillar_detect_timeout_, 30.0);
        pnh_.param<double>("override_climb_timeout", override_climb_timeout_, 10.0);
        pnh_.param<double>("override_climb_speed", override_climb_speed_, 0.18);
        pnh_.param<double>("override_climb_threshold", override_climb_threshold_, 0.05);
        pnh_.param<double>("pillar_valid_max_age", pillar_valid_max_age_, 0.5);
        pnh_.param<int>("pillar_valid_count_required", pillar_valid_count_required_, 5);
        pnh_.param<double>("pillar_align_timeout", pillar_align_timeout_, 25.0);
        pnh_.param<double>("pillar_align_threshold", pillar_align_threshold_, 0.12);
        pnh_.param<double>("pillar_align_speed", pillar_align_speed_, 0.10);
        pnh_.param<double>("pillar_align_kp", pillar_align_kp_, 0.65);
        pnh_.param<double>("pillar_align_z_kp", pillar_align_z_kp_, 0.9);
        pnh_.param<double>("pillar_align_max_vz", pillar_align_max_vz_, 0.12);
        pnh_.param<double>("pillar_align_yaw_kp", pillar_align_yaw_kp_, 0.6);
        pnh_.param<double>("pillar_align_max_yaw_rate", pillar_align_max_yaw_rate_, 0.25);
        pnh_.param<double>("pillar_align_yaw_threshold", pillar_align_yaw_threshold_, 0.20);
        pnh_.param<double>("pillar_align_hold_time", pillar_align_hold_time_, 0.8);
        pnh_.param<double>("pillar_frame_switch_after_distance", pillar_frame_switch_after_distance_, 0.60);
        pnh_.param<bool>("enable_pillar_active_scan", enable_pillar_active_scan_, true);
        pnh_.param<double>("pillar_active_scan_delay", pillar_active_scan_delay_, 3.0);
        pnh_.param<double>("pillar_scan_yaw_rate", pillar_scan_yaw_rate_, 0.35);
        pnh_.param<double>("pillar_scan_period", pillar_scan_period_, 8.0);
        pnh_.param<double>("pillar_scan_lateral_speed", pillar_scan_lateral_speed_, 0.06);
        pnh_.param<double>("pillar_scan_lateral_after", pillar_scan_lateral_after_, 8.0);
        pnh_.param<double>("pillar_scan_lateral_period", pillar_scan_lateral_period_, 10.0);
        pnh_.param<double>("pillar_scan_max_vz", pillar_scan_max_vz_, 0.12);
        pnh_.param<double>("frame_detect_timeout", frame_detect_timeout_, 20.0);
        pnh_.param<double>("frame_valid_max_age", frame_valid_max_age_, 0.6);
        pnh_.param<int>("frame_valid_count_required", frame_valid_count_required_, 5);
        pnh_.param<double>("frame_hold_at_pre", frame_hold_at_pre_, 1.0);
        pnh_.param<double>("frame_max_goal_distance", frame_max_goal_distance_, 5.0);
        pnh_.param<double>("frame_min_goal_z", frame_min_goal_z_, 0.35);
        pnh_.param<double>("frame_max_goal_z", frame_max_goal_z_, 1.8);
        pnh_.param<double>("frame_goal_fixed_z", frame_goal_fixed_z_, 0.9);
        pnh_.param<std::string>("goal_frame_id", goal_frame_id_, "world");
        pnh_.param<double>("ego_goal_z_limit", ego_goal_z_limit_, 2.0);
        pnh_.param<double>("ego_goal_z_fallback", ego_goal_z_fallback_, 0.9);
        pnh_.param<double>("frame_pre_direct_timeout", frame_pre_direct_timeout_, 30.0);
        pnh_.param<double>("frame_pre_direct_threshold", frame_pre_direct_threshold_, 0.20);
        pnh_.param<bool>("enable_frame_opportunistic_lock", enable_frame_opportunistic_lock_, true);
        pnh_.param<bool>("prefer_full_frame_lock", prefer_full_frame_lock_, true);
        pnh_.param<double>("frame_full_prefer_timeout", frame_full_prefer_timeout_, 4.0);
        pnh_.param<bool>("climb_to_flight_z_before_detect", climb_to_flight_z_before_detect_, true);
        pnh_.param<bool>("land_after_finish", land_after_finish_, true);

        pre_sub_ = nh_.subscribe("/craic/pillar_pre_goal", 1, &CraicCompetitionDemo::preGoalCb, this);
        post_sub_ = nh_.subscribe("/craic/pillar_post_goal", 1, &CraicCompetitionDemo::postGoalCb, this);
        status_sub_ = nh_.subscribe("/craic/pillar_status", 1, &CraicCompetitionDemo::statusCb, this);
        frame_pre_sub_ = nh_.subscribe("/craic/frame_pre_goal", 1, &CraicCompetitionDemo::framePreGoalCb, this);
        frame_post_sub_ = nh_.subscribe("/craic/frame_post_goal", 1, &CraicCompetitionDemo::framePostGoalCb, this);
        frame_status_sub_ = nh_.subscribe("/craic/frame_status", 1, &CraicCompetitionDemo::frameStatusCb, this);
    }

    int run() {
        if (!waitForBridge()) return 1;

        ROS_INFO("[craic_demo] ===== TAKEOFF =====");
        if (!api_.takeoff(takeoff_timeout_)) {
            ROS_ERROR("[craic_demo] Takeoff failed. Abort.");
            return 1;
        }

        // home_hover 是“击打区/起点区”的返回锚点：后面从任务区回来时不硬编码坐标。
        home_hover_ = api_.getOdomPosition();
        if (home_hover_.z() < 0.5) home_hover_.z() = flight_z_;
        ROS_INFO("[craic_demo] home_hover=(%.2f, %.2f, %.2f)",
                 home_hover_.x(), home_hover_.y(), home_hover_.z());

        if (climb_to_flight_z_before_detect_ && api_.getOdomPosition().z() < flight_z_ - 0.08) {
            ROS_INFO("[craic_demo] ===== CLIMB_TO_DETECT_HEIGHT =====");
            // 起飞后近处点云容易把自身/地面残点刷进 EGO 地图，这里不用 EGO 爬高，
            // 只用 OVERRIDE 给一个很小的竖直速度，把无人机送到任务高度。
            if (!overrideClimbToFlightZ()) {
                ROS_ERROR("[craic_demo] Override climb failed. Abort.");
                safeLand();
                return 1;
            }
            home_hover_.z() = flight_z_;
        }

        ROS_INFO("[craic_demo] ===== DETECT_PILLARS =====");
        // 必须等 /craic/pillar_status=valid，并且 pre/post 两个目标都收到，才允许继续飞。
        if (!waitForPillars()) {
            ROS_ERROR("[craic_demo] Pillar detection timeout. Abort.");
            safeLand();
            return 1;
        }

        // 去程朝向来自柱子识别节点发布的目标姿态；返程直接反向加 pi。
        double yaw_forward = yawFromPose(pillar_pre_);

        ROS_INFO("[craic_demo] ===== PASS_PILLARS_OUTBOUND =====");
        // 第一段不再把 pillar_pre 交给 EGO：先用 OVERRIDE 横向挪到两柱中心线，
        // 避免 EGO 从起点直接规划到贴近柱子的 pre_goal 时进入障碍膨胀区。
        if (!overrideAlignToPillarCenter(yaw_forward)) {
            ROS_ERROR("[craic_demo] Pillar lateral alignment failed. Abort.");
            safeLand();
            return 1;
        }
        yaw_forward = yawFromPose(pillar_pre_);
        const double yaw_back = normalizeYaw(yaw_forward + M_PI);
        if (enable_frame_opportunistic_lock_) {
            allow_frame_lock_ = true;
            frame_lock_start_ = ros::Time::now();
            ROS_INFO("[craic_demo] Frame opportunistic lock enabled during pillar outbound pass.");
        }
        // 对中完成后，只给 EGO 一个柱后目标点，让规划轨迹自然穿过两柱中线。
        if (!flyPillarPostUntilCrossOrReached(yaw_forward)) {
            ROS_ERROR("[craic_demo] Pillar outbound pass failed. Abort.");
            safeLand();
            return 1;
        }

        ROS_INFO("[craic_demo] ===== DETECT_FRAME_ONCE =====");
        if (!waitForFrameOnce()) {
            ROS_ERROR("[craic_demo] Frame detection timeout. Abort.");
            safeLand();
            return 1;
        }

        ROS_INFO("[craic_demo] ===== PASS_FRAME_OUTBOUND =====");
        if (!passFrameOutbound()) {
            ROS_ERROR("[craic_demo] Frame outbound pass failed. Abort.");
            safeLand();
            return 1;
        }

        ROS_INFO("[craic_demo] ===== QR_PLACEHOLDER =====");
        runOverrideTask(2, "qr_placeholder");

        ROS_INFO("[craic_demo] ===== PASS_FRAME_BACKWARD =====");
        if (!passFrameBackward()) {
            ROS_ERROR("[craic_demo] Frame backward pass failed. Abort.");
            safeLand();
            return 1;
        }

        ROS_INFO("[craic_demo] ===== PASS_PILLARS_BACKWARD =====");
        // 返程复用同一组柱子通道点，但 yaw 反向，避免依赖额外的固定返程航点。
        flyGoal("pillar_post_return", pillar_post_, yaw_back);
        flyGoal("pillar_pre_return", pillar_pre_, yaw_back);

        ROS_INFO("[craic_demo] ===== RETURN_ATTACK_ZONE =====");
        geometry_msgs::PoseStamped home = makePose(home_hover_, yaw_forward);
        flyGoal("home_attack_zone", home, yaw_forward);

        ROS_INFO("[craic_demo] ===== BALLOON_PLACEHOLDER =====");
        runOverrideTask(4, "balloon_placeholder");

        ROS_INFO("[craic_demo] ===== LANDING =====");
        if (land_after_finish_) {
            if (!api_.land(30.0)) {
                ROS_WARN("[craic_demo] Landing timeout.");
            }
        } else {
            ROS_WARN("[craic_demo] land_after_finish=false, holding after mission.");
        }

        ROS_INFO("[craic_demo] ===== DONE =====");
        return 0;
    }

private:
    std::string getBridgeNs() {
        std::string bridge_ns;
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("bridge_ns", bridge_ns, "/ego_bridge");
        return bridge_ns;
    }

    bool waitForBridge() {
        ROS_INFO("[craic_demo] Waiting for ego_bridge...");
        ros::Rate rate(10);
        while (ros::ok() && !api_.isConnected()) {
            ros::spinOnce();
            rate.sleep();
        }
        if (!ros::ok()) return false;
        ROS_INFO("[craic_demo] Connected. flight_state=%s", api_.getFlightState().c_str());
        return true;
    }

    bool waitForPillars() {
        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        bool scan_active = false;
        ros::Time scan_start;
        int valid_count = 0;
        while (ros::ok()) {
            ros::spinOnce();
            // 状态和目标点分开订阅，防止只收到 status 却还没拿到最新 goal。
            if (freshPillars()) {
                ++valid_count;
                if (valid_count >= pillar_valid_count_required_) {
                    if (scan_active) {
                        stopPillarActiveScan();
                    }
                    ROS_INFO("[craic_demo] Pillars stable. pre=(%.2f %.2f %.2f), post=(%.2f %.2f %.2f)",
                             pillar_pre_.pose.position.x, pillar_pre_.pose.position.y, pillar_pre_.pose.position.z,
                             pillar_post_.pose.position.x, pillar_post_.pose.position.y, pillar_post_.pose.position.z);
                    return true;
                }
            } else {
                valid_count = 0;
            }
            const double elapsed = (ros::Time::now() - start).toSec();
            if (enable_pillar_active_scan_ && !scan_active && elapsed > pillar_active_scan_delay_) {
                ROS_WARN("[craic_demo] Pillars not valid yet, start OVERRIDE active scan.");
                if (api_.enableOverride()) {
                    scan_active = true;
                    scan_start = ros::Time::now();
                } else {
                    ROS_WARN("[craic_demo] Active scan cannot enter OVERRIDE, keep waiting.");
                }
            }
            if (scan_active) {
                sendPillarActiveScanCmd((ros::Time::now() - scan_start).toSec());
            }
            if ((ros::Time::now() - start).toSec() > pillar_detect_timeout_) {
                if (scan_active) {
                    stopPillarActiveScan();
                }
                ROS_ERROR("[craic_demo] Pillar status=%s have_pre=%d have_post=%d",
                          pillar_status_.c_str(), have_pre_, have_post_);
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool freshPillars() const {
        const ros::Time now = ros::Time::now();
        return pillar_status_ == "valid" &&
               have_pre_ &&
               have_post_ &&
               pillar_status_stamp_.isValid() &&
               pillar_goal_stamp_.isValid() &&
               (now - pillar_status_stamp_).toSec() < pillar_valid_max_age_ &&
               (now - pillar_goal_stamp_).toSec() < pillar_valid_max_age_;
    }

    bool waitForFrameOnce() {
        if (have_locked_frame_) {
            ROS_INFO("[craic_demo] Frame already locked before explicit wait; reuse locked goals.");
            return true;
        }
        allow_frame_lock_ = true;
        if (!frame_lock_start_.isValid()) {
            frame_lock_start_ = ros::Time::now();
        }
        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        while (ros::ok()) {
            ros::spinOnce();
            updateFrameLock();
            if (have_locked_frame_) return true;

            ROS_INFO_THROTTLE(0.8,
                              "[craic_demo] waiting frame status=%s have_pre=%d have_post=%d valid_count=%d",
                              frame_status_.c_str(), have_frame_pre_, have_frame_post_, frame_valid_count_);
            if ((ros::Time::now() - start).toSec() > frame_detect_timeout_) {
                ROS_ERROR("[craic_demo] Frame status=%s have_pre=%d have_post=%d",
                          frame_status_.c_str(), have_frame_pre_, have_frame_post_);
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    void updateFrameLock() {
        if (!allow_frame_lock_ || have_locked_frame_) return;

        if (!freshFrame()) {
            frame_valid_count_ = 0;
            last_lock_candidate_status_.clear();
            return;
        }

        if (prefer_full_frame_lock_ &&
            !preferredFrameStatus(frame_status_) &&
            withinFullFramePreferWindow()) {
            frame_valid_count_ = 0;
            ROS_INFO_THROTTLE(0.8,
                              "[craic_demo] frame status=%s valid but waiting for valid_full prefer window %.1fs",
                              frame_status_.c_str(), frame_full_prefer_timeout_);
            return;
        }

        if (frame_status_ != last_lock_candidate_status_) {
            frame_valid_count_ = 0;
            last_lock_candidate_status_ = frame_status_;
        }

        ++frame_valid_count_;
        if (frame_valid_count_ < frame_valid_count_required_) return;

        geometry_msgs::PoseStamped pre = normalizeWorldGoal(frame_pre_);
        geometry_msgs::PoseStamped post = normalizeWorldGoal(frame_post_);
        if (!validateFrameGoal(pre, "frame_pre") ||
            !validateFrameGoal(post, "frame_post")) {
            frame_valid_count_ = 0;
            return;
        }

        locked_frame_pre_ = pre;
        locked_frame_post_ = post;
        locked_frame_yaw_forward_ = yawFromPose(locked_frame_pre_);
        locked_frame_yaw_back_ = normalizeYaw(locked_frame_yaw_forward_ + M_PI);
        have_locked_frame_ = true;
        logFrameGoal("locked_frame_pre", locked_frame_pre_);
        logFrameGoal("locked_frame_post", locked_frame_post_);
        ROS_INFO("[craic_demo] Frame locked once. status=%s valid_count=%d yaw_forward=%.2f yaw_back=%.2f",
                 frame_status_.c_str(), frame_valid_count_,
                 locked_frame_yaw_forward_, locked_frame_yaw_back_);
    }

    bool withinFullFramePreferWindow() const {
        if (!frame_lock_start_.isValid()) return true;
        return (ros::Time::now() - frame_lock_start_).toSec() < frame_full_prefer_timeout_;
    }

    bool freshFrame() const {
        const ros::Time now = ros::Time::now();
        return validFrameStatus(frame_status_) &&
               have_frame_pre_ &&
               have_frame_post_ &&
               frame_status_stamp_.isValid() &&
               frame_pre_stamp_.isValid() &&
               frame_post_stamp_.isValid() &&
               (now - frame_status_stamp_).toSec() < frame_valid_max_age_ &&
               (now - frame_pre_stamp_).toSec() < frame_valid_max_age_ &&
               (now - frame_post_stamp_).toSec() < frame_valid_max_age_;
    }

    geometry_msgs::PoseStamped normalizeWorldGoal(const geometry_msgs::PoseStamped& goal) const {
        geometry_msgs::PoseStamped out = goal;
        out.header.frame_id = goal_frame_id_;
        return out;
    }

    bool validateFrameGoal(const geometry_msgs::PoseStamped& pose,
                           const std::string& label) const {
        const Eigen::Vector3d target = posePosition(pose);
        const Eigen::Vector3d odom = api_.getOdomPosition();
        const double dist = (target - odom).norm();
        if (!std::isfinite(target.x()) ||
            !std::isfinite(target.y()) ||
            !std::isfinite(target.z())) {
            ROS_ERROR("[craic_demo] %s has non-finite position.", label.c_str());
            return false;
        }
        if (dist > frame_max_goal_distance_) {
            ROS_ERROR("[craic_demo] %s too far dist=%.2f max=%.2f target=(%.2f %.2f %.2f) odom=(%.2f %.2f %.2f)",
                      label.c_str(), dist, frame_max_goal_distance_,
                      target.x(), target.y(), target.z(),
                      odom.x(), odom.y(), odom.z());
            return false;
        }
        if (target.z() < frame_min_goal_z_ || target.z() > frame_max_goal_z_) {
            ROS_ERROR("[craic_demo] %s z out of range z=%.2f allowed=[%.2f %.2f]",
                      label.c_str(), target.z(), frame_min_goal_z_, frame_max_goal_z_);
            return false;
        }
        return true;
    }

    void logFrameGoal(const std::string& label,
                      const geometry_msgs::PoseStamped& pose) const {
        const Eigen::Vector3d target = posePosition(pose);
        const Eigen::Vector3d odom = api_.getOdomPosition();
        const double yaw = api_.getOdomYaw();
        const double dx = target.x() - odom.x();
        const double dy = target.y() - odom.y();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const double forward = c * dx + s * dy;
        const double lateral = -s * dx + c * dy;
        ROS_INFO("[craic_demo] %s frame=%s world=(%.2f %.2f %.2f) body_rel=(%.2f %.2f %.2f) yaw=%.2f",
                 label.c_str(), pose.header.frame_id.c_str(),
                 target.x(), target.y(), target.z(),
                 forward, lateral, target.z() - odom.z(), yawFromPose(pose));
    }

    bool passFrameOutbound() {
        if (!have_locked_frame_) {
            ROS_ERROR("[craic_demo] No locked frame goals for outbound pass.");
            return false;
        }
        logFrameGoal("outbound_frame_pre", locked_frame_pre_);
        logFrameGoal("outbound_frame_post", locked_frame_post_);
        if (!moveToFramePreEgo()) return false;
        ros::Duration(frame_hold_at_pre_).sleep();
        return flyFrameGoalFixedZ("frame_post", locked_frame_post_, locked_frame_yaw_forward_);
    }

    bool moveToFramePreEgo() {
        return flyFrameGoalFixedZ("frame_pre", locked_frame_pre_, api_.getOdomYaw());
    }

    bool passFrameBackward() {
        if (!have_locked_frame_) {
            ROS_ERROR("[craic_demo] No locked frame goals for backward pass.");
            return false;
        }
        ROS_INFO("[craic_demo] Reusing locked frame goals for backward pass; no frame re-detect.");
        logFrameGoal("backward_frame_post_prepare", locked_frame_post_);
        logFrameGoal("backward_frame_pre", locked_frame_pre_);
        if (!flyFrameGoalFixedZ("frame_post_return_prepare", locked_frame_post_, locked_frame_yaw_back_)) return false;
        ros::Duration(frame_hold_at_pre_).sleep();
        return flyFrameGoalFixedZ("frame_pre_return", locked_frame_pre_, locked_frame_yaw_back_);
    }

    void sendPillarActiveScanCmd(double elapsed) {
        const Eigen::Vector3d pos = api_.getOdomPosition();
        const double z_err = flight_z_ - pos.z();
        const double vz = clampValue(0.8 * z_err, -pillar_scan_max_vz_, pillar_scan_max_vz_);
        const double yaw_rate = pillar_scan_yaw_rate_ *
            std::sin(2.0 * M_PI * elapsed / std::max(1.0, pillar_scan_period_));

        double lateral_speed = 0.0;
        if (elapsed > pillar_scan_lateral_after_) {
            lateral_speed = pillar_scan_lateral_speed_ *
                std::sin(2.0 * M_PI * (elapsed - pillar_scan_lateral_after_) /
                         std::max(1.0, pillar_scan_lateral_period_));
        }
        const double yaw = api_.getOdomYaw();
        const double vx = -std::sin(yaw) * lateral_speed;
        const double vy = std::cos(yaw) * lateral_speed;
        api_.sendVelocityCmd(vx, vy, vz, yaw_rate);

        ROS_INFO_THROTTLE(1.0,
                          "[craic_demo] active pillar scan status=%s yaw_rate=%.2f lateral=%.2f z_err=%.2f",
                          pillar_status_.c_str(), yaw_rate, lateral_speed, z_err);
    }

    void stopPillarActiveScan() {
        holdOverride(0.25);
        api_.disableOverride();
    }

    bool flyGoal(const std::string& label, const geometry_msgs::PoseStamped& pose, double yaw) {
        const double z = sanitizeEgoGoalZ(pose.pose.position.z, label);
        ROS_INFO("[craic_demo] Goal %s: (%.2f, %.2f, %.2f), yaw=%.2f",
                 label.c_str(),
                 pose.pose.position.x, pose.pose.position.y, z, yaw);
        const bool reached = api_.sendGoalWithYaw(pose.pose.position.x,
                                                  pose.pose.position.y,
                                                  z,
                                                  yaw,
                                                  goal_timeout_);
        if (!reached) {
            ROS_WARN("[craic_demo] Goal %s timeout.", label.c_str());
        }
        return reached;
    }

    bool flyFrameGoalFixedZ(const std::string& label, const geometry_msgs::PoseStamped& pose, double yaw) {
        geometry_msgs::PoseStamped fixed = pose;
        fixed.pose.position.z = frame_goal_fixed_z_;
        return flyGoal(label, fixed, yaw);
    }

    bool flyPillarPostUntilCrossOrReached(double yaw) {
        const Eigen::Vector3d pre = posePosition(pillar_pre_);
        const Eigen::Vector3d post = posePosition(pillar_post_);
        const Eigen::Vector2d pre_xy(pre.x(), pre.y());
        const Eigen::Vector2d post_xy(post.x(), post.y());
        Eigen::Vector2d forward = post_xy - pre_xy;
        if (forward.norm() < 1e-3) {
            ROS_ERROR("[craic_demo] invalid pillar pre/post, cannot monitor pillar outbound progress.");
            return false;
        }
        forward.normalize();

        const Eigen::Vector2d corridor_xy = 0.5 * (pre_xy + post_xy);
        const double z = sanitizeEgoGoalZ(post.z(), "pillar_post");
        ROS_INFO("[craic_demo] Goal pillar_post monitored: (%.2f, %.2f, %.2f), yaw=%.2f switch_after=%.2f",
                 post.x(), post.y(), z, yaw, pillar_frame_switch_after_distance_);
        api_.publishGoalOnly(post.x(), post.y(), z, yaw);

        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        bool saw_reach_reset = api_.getReachStatus() == 0;
        while (ros::ok()) {
            ros::spinOnce();
            updateFrameLock();

            if (api_.getReachStatus() == 0) {
                saw_reach_reset = true;
            } else if (saw_reach_reset && api_.getReachStatus() == 1) {
                ROS_INFO("[craic_demo] pillar_post reached by ego_bridge.");
                return true;
            }

            const Eigen::Vector3d pos = api_.getOdomPosition();
            const Eigen::Vector2d pos_xy(pos.x(), pos.y());
            const double progress = (pos_xy - corridor_xy).dot(forward);
            const double lateral = std::abs((pos_xy - corridor_xy).dot(Eigen::Vector2d(-forward.y(), forward.x())));
            if (progress >= pillar_frame_switch_after_distance_ && have_locked_frame_) {
                ROS_INFO("[craic_demo] pillar crossed %.2fm after corridor with locked frame, switch to frame stage. lateral=%.2f",
                         progress, lateral);
                return true;
            }

            ROS_INFO_THROTTLE(0.5,
                              "[craic_demo] pillar outbound progress=%.2f/%.2f lateral=%.2f frame_locked=%d reach=%u",
                              progress, pillar_frame_switch_after_distance_, lateral,
                              have_locked_frame_, static_cast<unsigned int>(api_.getReachStatus()));

            if ((ros::Time::now() - start).toSec() > goal_timeout_) {
                ROS_WARN("[craic_demo] pillar outbound timeout progress=%.2f frame_locked=%d",
                         progress, have_locked_frame_);
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    double sanitizeEgoGoalZ(double z, const std::string& label) const {
        if (std::isfinite(z) && z <= ego_goal_z_limit_) {
            return z;
        }
        ROS_WARN("[craic_demo] %s z=%.2f exceeds ego limit %.2f, send fallback z=%.2f",
                 label.c_str(), z, ego_goal_z_limit_, ego_goal_z_fallback_);
        return ego_goal_z_fallback_;
    }

    bool overrideClimbToFlightZ() {
        if (!api_.enableOverride()) return false;

        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time stable_since;
        bool stable_started = false;
        bool reached = false;

        while (ros::ok()) {
            ros::spinOnce();
            const Eigen::Vector3d pos = api_.getOdomPosition();
            const double z_err = flight_z_ - pos.z();
            double vz = clampValue(1.0 * z_err, -override_climb_speed_, override_climb_speed_);
            if (std::abs(z_err) < override_climb_threshold_) vz = 0.0;

            api_.sendVelocityCmd(0.0, 0.0, vz, 0.0);

            if (std::abs(z_err) < override_climb_threshold_) {
                if (!stable_started) {
                    stable_started = true;
                    stable_since = ros::Time::now();
                } else if ((ros::Time::now() - stable_since).toSec() > 0.35) {
                    reached = true;
                    break;
                }
            } else {
                stable_started = false;
            }

            ROS_INFO_THROTTLE(0.5,
                              "[craic_demo] override climb z=%.2f target=%.2f err=%.2f vz=%.2f",
                              pos.z(), flight_z_, z_err, vz);

            if ((ros::Time::now() - start).toSec() > override_climb_timeout_) {
                ROS_WARN("[craic_demo] override climb timeout z_err=%.2f", z_err);
                break;
            }
            rate.sleep();
        }

        holdOverride(0.25);
        const bool disabled = api_.disableOverride();
        return reached && disabled;
    }

    bool overrideAlignToPillarCenter(double yaw_forward) {
        if (!api_.enableOverride()) return false;

        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time stable_since;
        bool stable_started = false;
        bool reached = false;

        while (ros::ok()) {
            ros::spinOnce();
            const Eigen::Vector3d pos = api_.getOdomPosition();
            const double z_err = flight_z_ - pos.z();
            double vz = clampValue(pillar_align_z_kp_ * z_err,
                                   -pillar_align_max_vz_, pillar_align_max_vz_);
            if (std::abs(z_err) < 0.04) vz = 0.0;

            if (!freshPillars()) {
                api_.sendVelocityCmd(0.0, 0.0, vz, 0.0);
                stable_started = false;
                ROS_WARN_THROTTLE(0.5,
                                  "[craic_demo] Pillar detection stale while aligning. status=%s",
                                  pillar_status_.c_str());
            } else {
                const Eigen::Vector3d pre = posePosition(pillar_pre_);
                const Eigen::Vector3d post = posePosition(pillar_post_);
                Eigen::Vector2d forward(post.x() - pre.x(), post.y() - pre.y());
                if (forward.norm() < 1e-3) {
                    ROS_ERROR("[craic_demo] invalid pillar pre/post, cannot compute forward direction.");
                    break;
                }
                forward.normalize();

                // lateral 是两柱连线方向；只沿这个方向移动，就能从起点横向挪到通道中心线。
                const Eigen::Vector2d lateral(-forward.y(), forward.x());
                const Eigen::Vector2d corridor_xy(0.5 * (pre.x() + post.x()),
                                                  0.5 * (pre.y() + post.y()));
                const Eigen::Vector2d pos_xy(pos.x(), pos.y());
                const Eigen::Vector2d to_center = corridor_xy - pos_xy;

                const double lateral_err = to_center.dot(lateral);
                const double along_err = to_center.dot(forward);
                yaw_forward = yawFromPose(pillar_pre_);
                const double yaw_err = normalizeYaw(yaw_forward - api_.getOdomYaw());

                double v_lat = clampValue(pillar_align_kp_ * lateral_err,
                                          -pillar_align_speed_, pillar_align_speed_);
                if (std::abs(lateral_err) < pillar_align_threshold_) v_lat = 0.0;

                double yaw_rate = clampValue(pillar_align_yaw_kp_ * yaw_err,
                                             -pillar_align_max_yaw_rate_, pillar_align_max_yaw_rate_);
                if (std::abs(yaw_err) < pillar_align_yaw_threshold_) yaw_rate = 0.0;

                api_.sendVelocityCmd(v_lat * lateral.x(),
                                     v_lat * lateral.y(),
                                     vz,
                                     yaw_rate);

                // 这里的“对中”只看横向误差和高度误差；yaw 只顺手慢速修正。
                const bool centered = std::abs(lateral_err) < pillar_align_threshold_ &&
                                      std::abs(z_err) < 0.12;
                if (centered) {
                    if (!stable_started) {
                        stable_started = true;
                        stable_since = ros::Time::now();
                    } else if ((ros::Time::now() - stable_since).toSec() > pillar_align_hold_time_) {
                        reached = true;
                        break;
                    }
                } else {
                    stable_started = false;
                }

                ROS_INFO_THROTTLE(0.4,
                                  "[craic_demo] pillar align pos=(%.2f %.2f %.2f) lat_err=%.3f along_err=%.3f z_err=%.3f yaw_err=%.3f v_lat=%.3f",
                                  pos.x(), pos.y(), pos.z(),
                                  lateral_err, along_err, z_err, yaw_err, v_lat);
            }

            if ((ros::Time::now() - start).toSec() > pillar_align_timeout_) {
                ROS_WARN("[craic_demo] pillar align timeout.");
                break;
            }
            rate.sleep();
        }

        holdOverride(0.25);
        const bool disabled = api_.disableOverride();
        return reached && disabled;
    }

    void holdOverride(double seconds) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < seconds) {
            ros::spinOnce();
            api_.holdPosition();
            rate.sleep();
        }
    }

    void runOverrideTask(int task_id, const std::string& label) {
        ROS_INFO("[craic_demo] Trigger override task %d (%s)", task_id, label.c_str());
        // task_id 的具体含义在 override_task_demo.cpp 中集中分配，主流程只负责编排顺序。
        api_.triggerOverrideTask(task_id);
        const bool ok = api_.waitOverrideComplete(override_timeout_);
        if (!ok) {
            ROS_WARN("[craic_demo] Override task %d (%s) timeout/forced return.", task_id, label.c_str());
        }
    }

    void safeLand() {
        if (land_after_finish_) {
            ROS_WARN("[craic_demo] Attempting safe landing after abort.");
            api_.land(30.0);
        }
    }

    geometry_msgs::PoseStamped makePose(const Eigen::Vector3d& p, double yaw) const {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "world";
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = p.z();
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);
        return pose;
    }

    void preGoalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pillar_pre_ = *msg;
        // 高度统一由比赛流程控制，柱子识别只负责给出平面通道位置。
        pillar_pre_.pose.position.z = flight_z_;
        have_pre_ = true;
        pillar_goal_stamp_ = ros::Time::now();
    }

    void postGoalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pillar_post_ = *msg;
        pillar_post_.pose.position.z = flight_z_;
        have_post_ = true;
        pillar_goal_stamp_ = ros::Time::now();
    }

    void statusCb(const std_msgs::String::ConstPtr& msg) {
        pillar_status_ = msg->data;
        pillar_status_stamp_ = ros::Time::now();
    }

    void framePreGoalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        frame_pre_ = *msg;
        have_frame_pre_ = true;
        frame_pre_stamp_ = ros::Time::now();
        updateFrameLock();
    }

    void framePostGoalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        frame_post_ = *msg;
        have_frame_post_ = true;
        frame_post_stamp_ = ros::Time::now();
        updateFrameLock();
    }

    void frameStatusCb(const std_msgs::String::ConstPtr& msg) {
        frame_status_ = msg->data;
        frame_status_stamp_ = ros::Time::now();
        updateFrameLock();
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    EgoApi api_;
    ros::Subscriber pre_sub_;
    ros::Subscriber post_sub_;
    ros::Subscriber status_sub_;
    ros::Subscriber frame_pre_sub_;
    ros::Subscriber frame_post_sub_;
    ros::Subscriber frame_status_sub_;

    double flight_z_ = 0.9;
    double takeoff_timeout_ = 30.0;
    double goal_timeout_ = 60.0;
    double override_timeout_ = 120.0;
    double pillar_detect_timeout_ = 30.0;
    double override_climb_timeout_ = 10.0;
    double override_climb_speed_ = 0.18;
    double override_climb_threshold_ = 0.05;
    double pillar_valid_max_age_ = 0.5;
    int pillar_valid_count_required_ = 5;
    double pillar_align_timeout_ = 25.0;
    double pillar_align_threshold_ = 0.12;
    double pillar_align_speed_ = 0.10;
    double pillar_align_kp_ = 0.65;
    double pillar_align_z_kp_ = 0.9;
    double pillar_align_max_vz_ = 0.12;
    double pillar_align_yaw_kp_ = 0.6;
    double pillar_align_max_yaw_rate_ = 0.25;
    double pillar_align_yaw_threshold_ = 0.20;
    double pillar_align_hold_time_ = 0.8;
    double pillar_frame_switch_after_distance_ = 0.60;
    bool enable_pillar_active_scan_ = true;
    double pillar_active_scan_delay_ = 3.0;
    double pillar_scan_yaw_rate_ = 0.35;
    double pillar_scan_period_ = 8.0;
    double pillar_scan_lateral_speed_ = 0.06;
    double pillar_scan_lateral_after_ = 8.0;
    double pillar_scan_lateral_period_ = 10.0;
    double pillar_scan_max_vz_ = 0.12;
    double frame_detect_timeout_ = 20.0;
    double frame_valid_max_age_ = 0.6;
    int frame_valid_count_required_ = 5;
    double frame_hold_at_pre_ = 1.0;
    double frame_max_goal_distance_ = 5.0;
    double frame_min_goal_z_ = 0.35;
    double frame_max_goal_z_ = 1.8;
    double frame_goal_fixed_z_ = 0.9;
    std::string goal_frame_id_ = "world";
    double ego_goal_z_limit_ = 2.0;
    double ego_goal_z_fallback_ = 0.9;
    double frame_pre_direct_timeout_ = 30.0;
    double frame_pre_direct_threshold_ = 0.20;
    bool enable_frame_opportunistic_lock_ = true;
    bool prefer_full_frame_lock_ = true;
    double frame_full_prefer_timeout_ = 4.0;
    bool land_after_finish_ = true;
    bool climb_to_flight_z_before_detect_ = true;

    bool have_pre_ = false;
    bool have_post_ = false;
    bool have_frame_pre_ = false;
    bool have_frame_post_ = false;
    bool have_locked_frame_ = false;
    bool allow_frame_lock_ = false;
    int frame_valid_count_ = 0;
    std::string last_lock_candidate_status_;
    std::string pillar_status_ = "unknown";
    std::string frame_status_ = "unknown";
    ros::Time pillar_status_stamp_;
    ros::Time pillar_goal_stamp_;
    ros::Time frame_status_stamp_;
    ros::Time frame_pre_stamp_;
    ros::Time frame_post_stamp_;
    ros::Time frame_lock_start_;
    geometry_msgs::PoseStamped pillar_pre_;
    geometry_msgs::PoseStamped pillar_post_;
    geometry_msgs::PoseStamped frame_pre_;
    geometry_msgs::PoseStamped frame_post_;
    geometry_msgs::PoseStamped locked_frame_pre_;
    geometry_msgs::PoseStamped locked_frame_post_;
    double locked_frame_yaw_forward_ = 0.0;
    double locked_frame_yaw_back_ = 0.0;
    Eigen::Vector3d home_hover_ = Eigen::Vector3d::Zero();
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "craic_competition_demo");
    CraicCompetitionDemo demo;
    return demo.run();
}
