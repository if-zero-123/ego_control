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

const char* yesNo(bool value) {
    return value ? "yes" : "no";
}

struct FrameHeightInfo {
    double raw_pre_z = 0.0;
    double raw_post_z = 0.0;
    double used_z = 0.0;
    std::string reason = "detector_post_z";
};

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
        pnh_.param<double>("mission_log_period", mission_log_period_, 1.0);
        pnh_.param<bool>("simple_logs", simple_logs_, true);
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
        pnh_.param<double>("pillar_return_switch_after_distance", pillar_return_switch_after_distance_, 0.45);
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
        pnh_.param<double>("frame_auto_goal_z_min", frame_auto_goal_z_min_, 0.9);
        pnh_.param<double>("frame_auto_goal_z_max", frame_auto_goal_z_max_, 1.65);
        pnh_.param<double>("frame_auto_goal_z_fallback", frame_auto_goal_z_fallback_, 1.1);
        pnh_.param<std::string>("goal_frame_id", goal_frame_id_, "world");
        pnh_.param<double>("ego_goal_z_limit", ego_goal_z_limit_, 2.0);
        pnh_.param<double>("ego_goal_z_fallback", ego_goal_z_fallback_, 0.9);
        pnh_.param<double>("frame_pre_direct_timeout", frame_pre_direct_timeout_, 30.0);
        pnh_.param<double>("frame_pre_direct_threshold", frame_pre_direct_threshold_, 0.20);
        pnh_.param<bool>("enable_frame_opportunistic_lock", enable_frame_opportunistic_lock_, true);
        pnh_.param<bool>("prefer_full_frame_lock", prefer_full_frame_lock_, true);
        pnh_.param<double>("frame_full_prefer_timeout", frame_full_prefer_timeout_, 4.0);
        pnh_.param<bool>("frame_use_override_direct_pass", frame_use_override_direct_pass_, false);
        pnh_.param<double>("frame_override_timeout", frame_override_timeout_, 20.0);
        pnh_.param<double>("frame_override_speed", frame_override_speed_, 0.18);
        pnh_.param<double>("frame_override_lateral_kp", frame_override_lateral_kp_, 0.8);
        pnh_.param<double>("frame_override_max_lateral_speed", frame_override_max_lateral_speed_, 0.12);
        pnh_.param<double>("frame_override_z_kp", frame_override_z_kp_, 0.8);
        pnh_.param<double>("frame_override_max_vz", frame_override_max_vz_, 0.10);
        pnh_.param<double>("frame_override_yaw_kp", frame_override_yaw_kp_, 0.7);
        pnh_.param<double>("frame_override_max_yaw_rate", frame_override_max_yaw_rate_, 0.25);
        pnh_.param<double>("frame_override_lateral_threshold", frame_override_lateral_threshold_, 0.15);
        pnh_.param<double>("frame_override_z_threshold", frame_override_z_threshold_, 0.12);
        pnh_.param<double>("frame_override_yaw_threshold", frame_override_yaw_threshold_, 0.25);
        pnh_.param<bool>("enable_frame_yaw_align_override", enable_frame_yaw_align_override_, true);
        pnh_.param<double>("frame_yaw_align_timeout", frame_yaw_align_timeout_, 4.0);
        pnh_.param<double>("frame_yaw_align_threshold", frame_yaw_align_threshold_, 0.12);
        pnh_.param<double>("frame_yaw_align_hold_time", frame_yaw_align_hold_time_, 0.25);
        pnh_.param<double>("frame_pre_offset", frame_pre_offset_, 0.70);
        pnh_.param<double>("frame_post_offset", frame_post_offset_, 0.70);
        pnh_.param<bool>("enable_qr_ego_scan", enable_qr_ego_scan_, true);
        pnh_.param<double>("qr_gate_half_width", qr_gate_half_width_, 0.5);
        pnh_.param<double>("qr_forward_offset", qr_forward_offset_, 1.0);
        pnh_.param<double>("qr_left_offset", qr_left_offset_, 1.0);
        pnh_.param<double>("qr_hold_time", qr_hold_time_, 2.0);
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

        logStage("TAKEOFF", "api.takeoff");
        if (!api_.takeoff(takeoff_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=TAKEOFF reason=takeoff_timeout_or_failed");
            return 1;
        }

        // home_hover 是“击打区/起点区”的返回锚点：后面从任务区回来时不硬编码坐标。
        home_hover_ = api_.getOdomPosition();
        if (home_hover_.z() < 0.5) home_hover_.z() = flight_z_;
        ROS_INFO("[craic_demo] HOME_ANCHOR world=(%.2f %.2f %.2f)",
                 home_hover_.x(), home_hover_.y(), home_hover_.z());

        if (climb_to_flight_z_before_detect_ && api_.getOdomPosition().z() < flight_z_ - 0.08) {
            logStage("CLIMB_TO_DETECT_Z", "override vertical velocity");
            // 起飞后近处点云容易把自身/地面残点刷进 EGO 地图，这里不用 EGO 爬高，
            // 只用 OVERRIDE 给一个很小的竖直速度，把无人机送到任务高度。
            if (!overrideClimbToFlightZ()) {
                ROS_ERROR("[craic_demo] FAIL stage=CLIMB_TO_DETECT_Z reason=override_climb_timeout target_z=%.2f", flight_z_);
                safeLand();
                return 1;
            }
            home_hover_.z() = flight_z_;
        }

        logStage("PILLAR_DETECT", "wait pillar_status=valid and pre/post goals");
        // 必须等 /craic/pillar_status=valid，并且 pre/post 两个目标都收到，才允许继续飞。
        if (!waitForPillars()) {
            ROS_ERROR("[craic_demo] FAIL stage=PILLAR_DETECT reason=timeout");
            safeLand();
            return 1;
        }

        // 去程朝向来自柱子识别节点发布的目标姿态；返程直接反向加 pi。
        double yaw_forward = yawFromPose(pillar_pre_);

        logStage("PILLAR_ALIGN", "override lateral align to pillar corridor");
        // 第一段不再把 pillar_pre 交给 EGO：先用 OVERRIDE 横向挪到两柱中心线，
        // 避免 EGO 从起点直接规划到贴近柱子的 pre_goal 时进入障碍膨胀区。
        if (!overrideAlignToPillarCenter(yaw_forward)) {
            ROS_ERROR("[craic_demo] FAIL stage=PILLAR_ALIGN reason=timeout_or_detection_lost");
            safeLand();
            return 1;
        }
        yaw_forward = yawFromPose(pillar_pre_);
        const double yaw_back = normalizeYaw(yaw_forward + M_PI);
        if (enable_frame_opportunistic_lock_) {
            allow_frame_lock_ = true;
            frame_lock_start_ = ros::Time::now();
            if (!simple_logs_) {
                ROS_INFO("[craic_demo] FRAME_LOCK opportunistic=yes during=PILLAR_PASS");
            }
        }
        // 对中完成后，只给 EGO 一个柱后目标点，让规划轨迹自然穿过两柱中线。
        logStage("PILLAR_PASS_EGO", "publish pillar_post; watch reach_status or centerline progress");
        if (!flyPillarPostUntilCrossOrReached(yaw_forward)) {
            ROS_ERROR("[craic_demo] FAIL stage=PILLAR_PASS_EGO reason=pass_condition_not_met");
            safeLand();
            return 1;
        }

        logStage("FRAME_LOCK", "wait one stable frame pre/post goal");
        if (!waitForFrameOnce()) {
            ROS_ERROR("[craic_demo] FAIL stage=FRAME_LOCK reason=timeout");
            safeLand();
            return 1;
        }

        logStage(frame_use_override_direct_pass_ ? "FRAME_PASS_OUT_OVERRIDE" : "FRAME_PASS_OUT_EGO",
                 frame_use_override_direct_pass_ ? "override direct frame pass" : "fly frame_pre then frame_post");
        if (!passFrameOutbound()) {
            ROS_ERROR("[craic_demo] FAIL stage=%s reason=goal_timeout_or_invalid",
                      frame_use_override_direct_pass_ ? "FRAME_PASS_OUT_OVERRIDE" : "FRAME_PASS_OUT_EGO");
            safeLand();
            return 1;
        }

        if (enable_qr_ego_scan_) {
            logStage("QR_SCAN_EGO", "fly to qr area, hover, return frame_post");
            if (!flyQrScanAfterFrame()) {
                ROS_ERROR("[craic_demo] FAIL stage=QR_SCAN_EGO reason=goal_timeout_or_invalid");
                safeLand();
                return 1;
            }
        } else {
            logStage("QR_SCAN_SKIP", "enable_qr_ego_scan=false");
        }

        logStage(frame_use_override_direct_pass_ ? "FRAME_PASS_BACK_OVERRIDE" : "FRAME_PASS_BACK_EGO",
                 frame_use_override_direct_pass_ ? "override direct frame return" : "reuse locked frame_pre in reverse");
        if (!passFrameBackward()) {
            ROS_ERROR("[craic_demo] FAIL stage=%s reason=goal_timeout_or_invalid",
                      frame_use_override_direct_pass_ ? "FRAME_PASS_BACK_OVERRIDE" : "FRAME_PASS_BACK_EGO");
            safeLand();
            return 1;
        }

        logStage("PILLAR_RETURN_EGO", "pillar_post entry, switch home after crossing centerline");
        // 返程先回柱后入口点，再用柱前点作为穿柱导向；穿过中心线一段距离后直接切回 home，
        // 避免在柱前目标点多停/多冲一段。
        flyGoal("pillar_post_return", pillar_post_, yaw_back);
        flyPillarPreUntilCrossOrReached(yaw_back);

        logStage("RETURN_HOME", "fly to takeoff hover anchor");
        geometry_msgs::PoseStamped home = makePose(home_hover_, yaw_forward);
        flyGoal("home_attack_zone", home, yaw_forward);

        logStage("BALLOON_PLACEHOLDER", "trigger override task 4");
        runOverrideTask(4, "balloon_placeholder");

        logStage("LAND", land_after_finish_ ? "api.land" : "land_after_finish=false");
        if (land_after_finish_) {
            if (!api_.land(30.0)) {
                ROS_WARN("[craic_demo] LAND result=timeout flight_state=%s", api_.getFlightState().c_str());
            }
        } else {
            ROS_WARN("[craic_demo] LAND skipped land_after_finish=false");
        }

        ROS_INFO("[craic_demo] MISSION_DONE");
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
        ROS_INFO("[craic_demo] CONNECT waiting_for=ego_bridge");
        ros::Rate rate(10);
        while (ros::ok() && !api_.isConnected()) {
            ros::spinOnce();
            rate.sleep();
        }
        if (!ros::ok()) return false;
        ROS_INFO("[craic_demo] CONNECT ok flight_state=%s", api_.getFlightState().c_str());
        return true;
    }

    void logStage(const std::string& stage, const std::string& action) const {
        if (simple_logs_) {
            ROS_INFO("[craic_demo] TASK step=%s", stage.c_str());
            return;
        }
        const Eigen::Vector3d pos = api_.getOdomPosition();
        ROS_INFO("[craic_demo] STAGE name=%s action=\"%s\" odom=(%.2f %.2f %.2f) yaw=%.2f mode=%u reach=%u",
                 stage.c_str(), action.c_str(),
                 pos.x(), pos.y(), pos.z(), api_.getOdomYaw(),
                 static_cast<unsigned int>(api_.getControlMode()),
                 static_cast<unsigned int>(api_.getReachStatus()));
    }

    Eigen::Vector3d targetBodyRel(const Eigen::Vector3d& target) const {
        const Eigen::Vector3d odom = api_.getOdomPosition();
        const double yaw = api_.getOdomYaw();
        const double dx = target.x() - odom.x();
        const double dy = target.y() - odom.y();
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        return Eigen::Vector3d(c * dx + s * dy,
                               -s * dx + c * dy,
                               target.z() - odom.z());
    }

    void logEgoTarget(const std::string& label,
                      const Eigen::Vector3d& target,
                      double yaw,
                      double timeout,
                      const std::string& note) const {
        if (simple_logs_) {
            ROS_INFO("[craic_demo] EGO_GOAL name=%s world=(%.2f %.2f %.2f) yaw=%.2f",
                     label.c_str(), target.x(), target.y(), target.z(), yaw);
            return;
        }
        const Eigen::Vector3d rel = targetBodyRel(target);
        ROS_INFO("[craic_demo] EGO_TARGET name=%s world=(%.2f %.2f %.2f) body=(front=%.2f left=%.2f up=%.2f) yaw=%.2f timeout=%.1f%s%s",
                 label.c_str(),
                 target.x(), target.y(), target.z(),
                 rel.x(), rel.y(), rel.z(),
                 yaw, timeout,
                 note.empty() ? "" : " | ",
                 note.c_str());
    }

    void logPoseTarget(const std::string& label,
                       const geometry_msgs::PoseStamped& pose,
                       double yaw,
                       const std::string& note) const {
        logEgoTarget(label, posePosition(pose), yaw, goal_timeout_, note);
    }

    void logDetectedPillar(int valid_count) const {
        const Eigen::Vector3d pre = posePosition(pillar_pre_);
        const Eigen::Vector3d post = posePosition(pillar_post_);
        const Eigen::Vector3d center = 0.5 * (pre + post);
        const Eigen::Vector2d center_xy(center.x(), center.y());
        const Eigen::Vector2d pre_xy(pre.x(), pre.y());
        const Eigen::Vector2d post_xy(post.x(), post.y());
        ROS_INFO("[craic_demo] DETECT pillar status=%s stable=%d/%d center=(%.2f %.2f %.2f) pre_goal=(%.2f %.2f %.2f) post_goal=(%.2f %.2f %.2f) pre_before_center_m=%.2f post_after_center_m=%.2f yaw=%.2f",
                 pillar_status_.c_str(), valid_count, pillar_valid_count_required_,
                 center.x(), center.y(), center.z(),
                 pre.x(), pre.y(), pre.z(),
                 post.x(), post.y(), post.z(),
                 (pre_xy - center_xy).norm(), (post_xy - center_xy).norm(),
                 yawFromPose(pillar_pre_));
    }

    void logDetectedFrame(const FrameHeightInfo& height_info) const {
        const Eigen::Vector3d pre = posePosition(locked_frame_pre_);
        const Eigen::Vector3d post = posePosition(locked_frame_post_);
        const Eigen::Vector2d pre_xy(pre.x(), pre.y());
        const Eigen::Vector2d post_xy(post.x(), post.y());
        Eigen::Vector2d forward = post_xy - pre_xy;
        if (forward.norm() > 1e-3) {
            forward.normalize();
        } else {
            forward = Eigen::Vector2d(std::cos(locked_frame_yaw_forward_),
                                      std::sin(locked_frame_yaw_forward_));
        }
        const Eigen::Vector2d center_xy = pre_xy + frame_pre_offset_ * forward;
        ROS_INFO("[craic_demo] DETECT frame status=%s stable=%d/%d center=(%.2f %.2f %.2f) pre_goal=(%.2f %.2f %.2f) post_goal=(%.2f %.2f %.2f) pre_before_frame_m=%.2f post_after_frame_m=%.2f raw_z=(pre=%.2f post=%.2f) protected_z=%.2f z_rule=%s yaw=%.2f",
                 frame_status_.c_str(), frame_valid_count_, frame_valid_count_required_,
                 center_xy.x(), center_xy.y(), height_info.used_z,
                 pre.x(), pre.y(), pre.z(),
                 post.x(), post.y(), post.z(),
                 (pre_xy - center_xy).norm(), (post_xy - center_xy).norm(),
                 height_info.raw_pre_z, height_info.raw_post_z,
                 height_info.used_z, height_info.reason.c_str(),
                 locked_frame_yaw_forward_);
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
                    logDetectedPillar(valid_count);
                    return true;
                }
            } else {
                valid_count = 0;
            }
            const double elapsed = (ros::Time::now() - start).toSec();
            if (enable_pillar_active_scan_ && !scan_active && elapsed > pillar_active_scan_delay_) {
                ROS_WARN("[craic_demo] PILLAR_DETECT not_stable_for=%.1fs action=start_override_active_scan", elapsed);
                if (api_.enableOverride()) {
                    scan_active = true;
                    scan_start = ros::Time::now();
                } else {
                    ROS_WARN("[craic_demo] PILLAR_DETECT active_scan_failed action=keep_waiting");
                }
            }
            if (scan_active) {
                sendPillarActiveScanCmd((ros::Time::now() - scan_start).toSec());
            }
            if ((ros::Time::now() - start).toSec() > pillar_detect_timeout_) {
                if (scan_active) {
                    stopPillarActiveScan();
                }
                ROS_ERROR("[craic_demo] PILLAR_DETECT timeout status=%s have_pre=%s have_post=%s",
                          pillar_status_.c_str(), yesNo(have_pre_), yesNo(have_post_));
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
            if (!simple_logs_) {
                ROS_INFO("[craic_demo] FRAME_LOCK already_locked reuse=yes");
            }
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

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] FRAME_LOCK waiting status=%s have_pre=%s have_post=%s stable=%d/%d",
                                  frame_status_.c_str(), yesNo(have_frame_pre_), yesNo(have_frame_post_),
                                  frame_valid_count_, frame_valid_count_required_);
            }
            if ((ros::Time::now() - start).toSec() > frame_detect_timeout_) {
                ROS_ERROR("[craic_demo] FRAME_LOCK timeout status=%s have_pre=%s have_post=%s stable=%d/%d",
                          frame_status_.c_str(), yesNo(have_frame_pre_), yesNo(have_frame_post_),
                          frame_valid_count_, frame_valid_count_required_);
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
            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] FRAME_LOCK candidate=%s action=wait_valid_full prefer_window=%.1fs",
                                  frame_status_.c_str(), frame_full_prefer_timeout_);
            }
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
        const FrameHeightInfo height_info = applyFrameHeightProtection(pre, post);
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
        if (simple_logs_) {
            logDetectedFrame(height_info);
        } else {
            logPoseTarget("frame_pre_locked", locked_frame_pre_, locked_frame_yaw_forward_, "locked frame pre");
            logPoseTarget("frame_post_locked", locked_frame_post_, locked_frame_yaw_forward_, "locked frame post");
            ROS_INFO("[craic_demo] FRAME_LOCK locked status=%s stable=%d/%d yaw_forward=%.2f yaw_back=%.2f used_z=%.2f",
                     frame_status_.c_str(), frame_valid_count_,
                     frame_valid_count_required_, locked_frame_yaw_forward_, locked_frame_yaw_back_,
                     locked_frame_post_.pose.position.z);
        }
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
            ROS_ERROR("[craic_demo] FRAME_GOAL invalid label=%s reason=non_finite", label.c_str());
            return false;
        }
        if (dist > frame_max_goal_distance_) {
            ROS_ERROR("[craic_demo] FRAME_GOAL invalid label=%s reason=too_far dist=%.2f max=%.2f target=(%.2f %.2f %.2f) odom=(%.2f %.2f %.2f)",
                      label.c_str(), dist, frame_max_goal_distance_,
                      target.x(), target.y(), target.z(),
                      odom.x(), odom.y(), odom.z());
            return false;
        }
        if (target.z() < frame_min_goal_z_ || target.z() > frame_max_goal_z_) {
            ROS_ERROR("[craic_demo] FRAME_GOAL invalid label=%s reason=z_out_of_range z=%.2f allowed=[%.2f %.2f]",
                      label.c_str(), target.z(), frame_min_goal_z_, frame_max_goal_z_);
            return false;
        }
        return true;
    }

    FrameHeightInfo applyFrameHeightProtection(geometry_msgs::PoseStamped& pre,
                                               geometry_msgs::PoseStamped& post) const {
        FrameHeightInfo info;
        info.raw_pre_z = pre.pose.position.z;
        info.raw_post_z = post.pose.position.z;
        double z = frame_goal_fixed_z_;
        info.reason = "fixed_goal_z";
        if (!std::isfinite(z)) {
            z = frame_auto_goal_z_fallback_;
            info.reason = "fixed_z_invalid_fallback";
        }
        pre.pose.position.z = z;
        post.pose.position.z = z;
        info.used_z = z;
        return info;
    }

    bool passFrameOutbound() {
        if (!have_locked_frame_) {
            ROS_ERROR("[craic_demo] FRAME_PASS_OUT invalid reason=no_locked_frame");
            return false;
        }
        if (frame_use_override_direct_pass_) {
            return overridePassFrame(true);
        }
        if (!moveToFramePreEgo()) return false;
        if (!alignFrameYawOverride("frame_pre_forward", locked_frame_yaw_forward_)) return false;
        ros::Duration(frame_hold_at_pre_).sleep();
        return flyGoal("frame_post", locked_frame_post_, locked_frame_yaw_forward_);
    }

    bool moveToFramePreEgo() {
        return flyGoal("frame_pre", locked_frame_pre_, api_.getOdomYaw());
    }

    bool passFrameBackward() {
        if (!have_locked_frame_) {
            ROS_ERROR("[craic_demo] FRAME_PASS_BACK invalid reason=no_locked_frame");
            return false;
        }
        if (frame_use_override_direct_pass_) {
            return overridePassFrame(false);
        }
        if (!simple_logs_) {
            ROS_INFO("[craic_demo] FRAME_PASS_BACK reuse_locked_goal=frame_pre z=%.2f",
                     locked_frame_pre_.pose.position.z);
        }
        if (!alignFrameYawOverride("frame_post_back", locked_frame_yaw_back_)) return false;
        return flyGoal("frame_pre_return", locked_frame_pre_, locked_frame_yaw_back_);
    }

    bool flyQrScanAfterFrame() {
        if (!have_locked_frame_) {
            ROS_ERROR("[craic_demo] QR_SCAN invalid reason=no_locked_frame");
            return false;
        }

        geometry_msgs::PoseStamped qr_goal;
        if (!makeQrScanGoal(qr_goal)) {
            return false;
        }

        const Eigen::Vector3d qr_pos = posePosition(qr_goal);
        ROS_INFO("[craic_demo] QR_GOAL source=frame_left_side gate_half_width=%.2f offset_forward=%.2f offset_left=%.2f world=(%.2f %.2f %.2f) hold=%.1f",
                 qr_gate_half_width_, qr_forward_offset_, qr_left_offset_,
                 qr_pos.x(), qr_pos.y(), qr_pos.z(), qr_hold_time_);

        if (!flyGoal("qr_scan_goal", qr_goal, locked_frame_yaw_forward_)) {
            return false;
        }

        ros::Duration(qr_hold_time_).sleep();
        ROS_INFO("[craic_demo] RESULT qr_scan_hover done hold=%.1f", qr_hold_time_);

        return flyGoal("qr_return_frame_post", locked_frame_post_, locked_frame_yaw_back_);
    }

    bool makeQrScanGoal(geometry_msgs::PoseStamped& out) const {
        const Eigen::Vector3d pre = posePosition(locked_frame_pre_);
        const Eigen::Vector3d post = posePosition(locked_frame_post_);
        const Eigen::Vector2d pre_xy(pre.x(), pre.y());
        const Eigen::Vector2d post_xy(post.x(), post.y());
        Eigen::Vector2d forward = post_xy - pre_xy;
        if (forward.norm() < 1e-3) {
            ROS_ERROR("[craic_demo] QR_SCAN invalid reason=bad_frame_direction");
            return false;
        }
        forward.normalize();

        const Eigen::Vector2d left(-forward.y(), forward.x());
        const Eigen::Vector2d center_xy = pre_xy + frame_pre_offset_ * forward;
        const Eigen::Vector2d left_side_xy = center_xy + qr_gate_half_width_ * left;
        const Eigen::Vector2d qr_xy = left_side_xy +
                                      qr_forward_offset_ * forward +
                                      qr_left_offset_ * left;
        const Eigen::Vector3d qr_pos(qr_xy.x(), qr_xy.y(), locked_frame_post_.pose.position.z);
        out = makePose(qr_pos, locked_frame_yaw_forward_);
        return validateFrameGoal(out, "qr_scan_goal");
    }

    bool overridePassFrame(bool outbound) {
        const Eigen::Vector3d pre = posePosition(locked_frame_pre_);
        const Eigen::Vector3d post = posePosition(locked_frame_post_);
        const Eigen::Vector2d pre_xy(pre.x(), pre.y());
        const Eigen::Vector2d post_xy(post.x(), post.y());
        Eigen::Vector2d frame_forward = post_xy - pre_xy;
        if (frame_forward.norm() < 1e-3) {
            ROS_ERROR("[craic_demo] FRAME_OVERRIDE invalid reason=bad_pre_post_direction");
            return false;
        }
        frame_forward.normalize();

        const Eigen::Vector2d travel = outbound ? frame_forward : -frame_forward;
        const Eigen::Vector2d lateral(-frame_forward.y(), frame_forward.x());
        const Eigen::Vector2d center_xy = pre_xy + frame_pre_offset_ * frame_forward;
        const double stop_after_center = outbound ? frame_post_offset_ : frame_pre_offset_;
        const double target_z = outbound ? post.z() : pre.z();
        const double target_yaw = outbound ? locked_frame_yaw_forward_ : locked_frame_yaw_back_;
        const std::string label = outbound ? "frame_override_out" : "frame_override_back";

        ROS_INFO("[craic_demo] OVERRIDE_FRAME start name=%s center=(%.2f %.2f %.2f) stop_after_center=%.2f speed=%.2f",
                 label.c_str(), center_xy.x(), center_xy.y(), target_z,
                 stop_after_center, frame_override_speed_);

        if (!api_.enableOverride()) {
            ROS_ERROR("[craic_demo] OVERRIDE_FRAME failed name=%s reason=enable_override_failed", label.c_str());
            return false;
        }

        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        bool reached = false;
        while (ros::ok()) {
            ros::spinOnce();

            const Eigen::Vector3d pos = api_.getOdomPosition();
            const Eigen::Vector2d pos_xy(pos.x(), pos.y());
            const Eigen::Vector2d to_center = center_xy - pos_xy;
            const double progress = (pos_xy - center_xy).dot(travel);
            const double lateral_err = to_center.dot(lateral);
            const double z_err = target_z - pos.z();
            const double yaw_err = normalizeYaw(target_yaw - api_.getOdomYaw());

            const bool aligned = std::abs(lateral_err) < frame_override_lateral_threshold_ &&
                                 std::abs(z_err) < frame_override_z_threshold_;
            const double forward_speed = aligned ? frame_override_speed_ : 0.0;
            const double v_lat = clampValue(frame_override_lateral_kp_ * lateral_err,
                                            -frame_override_max_lateral_speed_,
                                            frame_override_max_lateral_speed_);
            const double vz = clampValue(frame_override_z_kp_ * z_err,
                                         -frame_override_max_vz_,
                                         frame_override_max_vz_);
            const Eigen::Vector2d cmd_xy = forward_speed * travel + v_lat * lateral;
            sendVelocityCmdWithYaw(cmd_xy.x(), cmd_xy.y(), vz, target_yaw);

            if (progress >= stop_after_center) {
                reached = true;
                ROS_INFO("[craic_demo] RESULT %s reached_by=override_progress progress=%.2f target=%.2f lateral_err=%.2f z_err=%.2f",
                         label.c_str(), progress, stop_after_center, lateral_err, z_err);
                break;
            }

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] OVERRIDE_FRAME name=%s progress=%.2f/%.2f lat_err=%.2f z_err=%.2f yaw_err=%.2f cmd_v=(%.2f %.2f %.2f) yaw_rate=%.2f aligned=%s",
                                  label.c_str(), progress, stop_after_center,
                                  lateral_err, z_err, yaw_err,
                                  cmd_xy.x(), cmd_xy.y(), vz, 0.0,
                                  yesNo(aligned));
            }

            if ((ros::Time::now() - start).toSec() > frame_override_timeout_) {
                ROS_WARN("[craic_demo] OVERRIDE_FRAME result=timeout name=%s timeout=%.1f progress=%.2f/%.2f lateral_err=%.2f z_err=%.2f",
                         label.c_str(), frame_override_timeout_, progress, stop_after_center, lateral_err, z_err);
                break;
            }
            rate.sleep();
        }

        holdOverride(0.25);
        const bool disabled = api_.disableOverride();
        return reached && disabled;
    }

    bool alignFrameYawOverride(const std::string& label, double target_yaw) {
        if (!enable_frame_yaw_align_override_) {
            return true;
        }

        ROS_INFO("[craic_demo] YAW_ALIGN start name=%s control=OVERRIDE target_yaw=%.2f current_yaw=%.2f threshold=%.2f",
                 label.c_str(), target_yaw, api_.getOdomYaw(), frame_yaw_align_threshold_);

        if (!api_.enableOverride()) {
            ROS_ERROR("[craic_demo] YAW_ALIGN failed name=%s reason=enable_override_failed", label.c_str());
            return false;
        }

        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time stable_since(0);
        bool reached = false;
        double yaw_err = normalizeYaw(target_yaw - api_.getOdomYaw());

        while (ros::ok()) {
            ros::spinOnce();

            const double current_yaw = api_.getOdomYaw();
            yaw_err = normalizeYaw(target_yaw - current_yaw);
            sendVelocityCmdWithYaw(0.0, 0.0, 0.0, target_yaw);

            if (std::abs(yaw_err) < frame_yaw_align_threshold_) {
                if (stable_since.isZero()) {
                    stable_since = ros::Time::now();
                }
                if ((ros::Time::now() - stable_since).toSec() >= frame_yaw_align_hold_time_) {
                    reached = true;
                    ROS_INFO("[craic_demo] RESULT yaw_align reached name=%s target_yaw=%.2f current_yaw=%.2f err=%.2f",
                             label.c_str(), target_yaw, current_yaw, yaw_err);
                    break;
                }
            } else {
                stable_since = ros::Time(0);
            }

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] YAW_ALIGN name=%s target_yaw=%.2f current_yaw=%.2f err=%.2f",
                                  label.c_str(), target_yaw, current_yaw, yaw_err);
            }

            if ((ros::Time::now() - start).toSec() > frame_yaw_align_timeout_) {
                ROS_WARN("[craic_demo] YAW_ALIGN result=timeout name=%s timeout=%.1f target_yaw=%.2f current_yaw=%.2f err=%.2f",
                         label.c_str(), frame_yaw_align_timeout_, target_yaw, current_yaw, yaw_err);
                break;
            }
            rate.sleep();
        }

        holdOverride(0.15);
        const bool disabled = api_.disableOverride();
        return reached && disabled;
    }

    void sendVelocityCmdWithYaw(double vx, double vy, double vz, double yaw) {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        const Eigen::Vector3d pos = api_.getOdomPosition();
        cmd.position.x = pos.x();
        cmd.position.y = pos.y();
        cmd.position.z = pos.z();
        cmd.velocity.x = vx;
        cmd.velocity.y = vy;
        cmd.velocity.z = vz;
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
        cmd.yaw = yaw;
        cmd.yaw_dot = 0.0;
        cmd.trajectory_id = 0;
        cmd.trajectory_flag = 0;
        api_.sendOverrideCmd(cmd);
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

        if (!simple_logs_) {
            ROS_INFO_THROTTLE(mission_log_period_,
                              "[craic_demo] PILLAR_SCAN status=%s cmd_v=(%.2f %.2f %.2f) yaw_rate=%.2f z_err=%.2f",
                              pillar_status_.c_str(), vx, vy, vz, yaw_rate, z_err);
        }
    }

    void stopPillarActiveScan() {
        holdOverride(0.25);
        api_.disableOverride();
    }

    bool flyGoal(const std::string& label, const geometry_msgs::PoseStamped& pose, double yaw) {
        const double z = sanitizeEgoGoalZ(pose.pose.position.z, label);
        const Eigen::Vector3d target(pose.pose.position.x, pose.pose.position.y, z);
        logEgoTarget(label, target, yaw, goal_timeout_, "sendGoalWithYaw");
        const bool reached = api_.sendGoalWithYaw(pose.pose.position.x,
                                                  pose.pose.position.y,
                                                  z,
                                                  yaw,
                                                  goal_timeout_);
        if (reached) {
            if (simple_logs_) {
                ROS_INFO("[craic_demo] RESULT goal=%s reached", label.c_str());
            } else {
                ROS_INFO("[craic_demo] EGO_RESULT name=%s result=reached reach_status=1", label.c_str());
            }
        } else {
            if (simple_logs_) {
                ROS_WARN("[craic_demo] RESULT goal=%s timeout=%.1f", label.c_str(), goal_timeout_);
            } else {
                ROS_WARN("[craic_demo] EGO_RESULT name=%s result=timeout timeout=%.1f", label.c_str(), goal_timeout_);
            }
        }
        return reached;
    }

    bool flyPillarPostUntilCrossOrReached(double yaw) {
        const Eigen::Vector3d pre = posePosition(pillar_pre_);
        const Eigen::Vector3d post = posePosition(pillar_post_);
        const Eigen::Vector2d pre_xy(pre.x(), pre.y());
        const Eigen::Vector2d post_xy(post.x(), post.y());
        Eigen::Vector2d forward = post_xy - pre_xy;
        if (forward.norm() < 1e-3) {
            ROS_ERROR("[craic_demo] PILLAR_PASS invalid reason=bad_pre_post_direction");
            return false;
        }
        forward.normalize();

        const Eigen::Vector2d corridor_xy = 0.5 * (pre_xy + post_xy);
        const double z = sanitizeEgoGoalZ(post.z(), "pillar_post");
        logEgoTarget("pillar_post",
                     Eigen::Vector3d(post.x(), post.y(), z),
                     yaw,
                     goal_timeout_,
                     "publishGoalOnly; pass_condition=reach_status_or_centerline");
        if (!simple_logs_) {
            ROS_INFO("[craic_demo] PILLAR_PASS switch_condition=centerline_progress threshold=%.2f require_frame_locked=yes",
                     pillar_frame_switch_after_distance_);
        }
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
                ROS_INFO("[craic_demo] RESULT pillar_pass reached_by=reach_status");
                return true;
            }

            const Eigen::Vector3d pos = api_.getOdomPosition();
            const Eigen::Vector2d pos_xy(pos.x(), pos.y());
            const double progress = (pos_xy - corridor_xy).dot(forward);
            const double lateral = std::abs((pos_xy - corridor_xy).dot(Eigen::Vector2d(-forward.y(), forward.x())));
            if (progress >= pillar_frame_switch_after_distance_ && have_locked_frame_) {
                ROS_INFO("[craic_demo] RESULT pillar_pass reached_by=centerline progress=%.2f threshold=%.2f",
                         progress, pillar_frame_switch_after_distance_);
                return true;
            }

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] PILLAR_PASS progress=%.2f/%.2f lateral=%.2f frame_locked=%s reach=%u",
                                  progress, pillar_frame_switch_after_distance_, lateral,
                                  yesNo(have_locked_frame_), static_cast<unsigned int>(api_.getReachStatus()));
            }

            if ((ros::Time::now() - start).toSec() > goal_timeout_) {
                ROS_WARN("[craic_demo] PILLAR_PASS result=timeout timeout=%.1f progress=%.2f/%.2f frame_locked=%s",
                         goal_timeout_, progress, pillar_frame_switch_after_distance_, yesNo(have_locked_frame_));
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool flyPillarPreUntilCrossOrReached(double yaw) {
        const Eigen::Vector3d pre = posePosition(pillar_pre_);
        const Eigen::Vector3d post = posePosition(pillar_post_);
        const Eigen::Vector2d pre_xy(pre.x(), pre.y());
        const Eigen::Vector2d post_xy(post.x(), post.y());
        Eigen::Vector2d return_forward = pre_xy - post_xy;
        if (return_forward.norm() < 1e-3) {
            ROS_ERROR("[craic_demo] PILLAR_RETURN invalid reason=bad_pre_post_direction");
            return false;
        }
        return_forward.normalize();

        const Eigen::Vector2d corridor_xy = 0.5 * (pre_xy + post_xy);
        const double z = sanitizeEgoGoalZ(pre.z(), "pillar_pre_return");
        logEgoTarget("pillar_pre_return",
                     Eigen::Vector3d(pre.x(), pre.y(), z),
                     yaw,
                     goal_timeout_,
                     "publishGoalOnly; switch_home_after_centerline");
        api_.publishGoalOnly(pre.x(), pre.y(), z, yaw);

        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        bool saw_reach_reset = api_.getReachStatus() == 0;
        while (ros::ok()) {
            ros::spinOnce();

            if (api_.getReachStatus() == 0) {
                saw_reach_reset = true;
            } else if (saw_reach_reset && api_.getReachStatus() == 1) {
                ROS_INFO("[craic_demo] RESULT pillar_return reached_by=reach_status");
                return true;
            }

            const Eigen::Vector3d pos = api_.getOdomPosition();
            const Eigen::Vector2d pos_xy(pos.x(), pos.y());
            const double progress = (pos_xy - corridor_xy).dot(return_forward);
            const double lateral = std::abs((pos_xy - corridor_xy).dot(Eigen::Vector2d(-return_forward.y(), return_forward.x())));
            if (progress >= pillar_return_switch_after_distance_) {
                ROS_INFO("[craic_demo] RESULT pillar_return reached_by=centerline_to_home progress=%.2f threshold=%.2f next=home_attack_zone",
                         progress, pillar_return_switch_after_distance_);
                return true;
            }

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] PILLAR_RETURN progress=%.2f/%.2f lateral=%.2f reach=%u",
                                  progress, pillar_return_switch_after_distance_, lateral,
                                  static_cast<unsigned int>(api_.getReachStatus()));
            }

            if ((ros::Time::now() - start).toSec() > goal_timeout_) {
                ROS_WARN("[craic_demo] PILLAR_RETURN result=timeout timeout=%.1f progress=%.2f/%.2f",
                         goal_timeout_, progress, pillar_return_switch_after_distance_);
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
        ROS_WARN("[craic_demo] EGO_TARGET_Z_SANITIZE name=%s raw_z=%.2f limit=%.2f used_z=%.2f",
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

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] OVERRIDE_CLIMB z=%.2f target_z=%.2f z_err=%.2f cmd_v=(0.00 0.00 %.2f)",
                                  pos.z(), flight_z_, z_err, vz);
            }

            if ((ros::Time::now() - start).toSec() > override_climb_timeout_) {
                ROS_WARN("[craic_demo] OVERRIDE_CLIMB result=timeout timeout=%.1f z_err=%.2f",
                         override_climb_timeout_, z_err);
                break;
            }
            rate.sleep();
        }

        holdOverride(0.25);
        const bool disabled = api_.disableOverride();
        if (reached && disabled) {
            if (simple_logs_) {
                ROS_INFO("[craic_demo] RESULT climb_to_detect_z reached z=%.2f", api_.getOdomPosition().z());
            } else {
                ROS_INFO("[craic_demo] OVERRIDE_CLIMB result=done current_z=%.2f", api_.getOdomPosition().z());
            }
        }
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
                if (!simple_logs_) {
                    ROS_WARN_THROTTLE(mission_log_period_,
                                      "[craic_demo] PILLAR_ALIGN detection_stale status=%s action=hold_lateral cmd_v=(0.00 0.00 %.2f)",
                                      pillar_status_.c_str(), vz);
                }
            } else {
                const Eigen::Vector3d pre = posePosition(pillar_pre_);
                const Eigen::Vector3d post = posePosition(pillar_post_);
                Eigen::Vector2d forward(post.x() - pre.x(), post.y() - pre.y());
                if (forward.norm() < 1e-3) {
                    ROS_ERROR("[craic_demo] PILLAR_ALIGN invalid reason=bad_pre_post_direction");
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

                const double cmd_vx = v_lat * lateral.x();
                const double cmd_vy = v_lat * lateral.y();
                api_.sendVelocityCmd(cmd_vx, cmd_vy, vz, yaw_rate);

                // 这里的“对中”只看横向误差和高度误差；yaw 只顺手慢速修正。
                const bool centered = std::abs(lateral_err) < pillar_align_threshold_ &&
                                      std::abs(z_err) < 0.12;
                if (centered) {
                    if (!stable_started) {
                        stable_started = true;
                        stable_since = ros::Time::now();
                    } else if ((ros::Time::now() - stable_since).toSec() > pillar_align_hold_time_) {
                        reached = true;
                        if (simple_logs_) {
                            ROS_INFO("[craic_demo] RESULT pillar_align centered");
                        } else {
                            ROS_INFO("[craic_demo] PILLAR_ALIGN result=done lat_err=%.3f z_err=%.3f yaw_err=%.3f hold=%.1f",
                                     lateral_err, z_err, yaw_err, pillar_align_hold_time_);
                        }
                        break;
                    }
                } else {
                    stable_started = false;
                }

                if (!simple_logs_) {
                    ROS_INFO_THROTTLE(mission_log_period_,
                                      "[craic_demo] PILLAR_ALIGN err_lat=%.3f err_forward=%.3f err_z=%.3f err_yaw=%.3f cmd_v=(%.2f %.2f %.2f) yaw_rate=%.2f",
                                      lateral_err, along_err, z_err, yaw_err,
                                      cmd_vx, cmd_vy, vz, yaw_rate);
                }
            }

            if ((ros::Time::now() - start).toSec() > pillar_align_timeout_) {
                ROS_WARN("[craic_demo] PILLAR_ALIGN result=timeout timeout=%.1f", pillar_align_timeout_);
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
        ROS_INFO("[craic_demo] OVERRIDE_TASK trigger task=%d label=%s", task_id, label.c_str());
        // task_id 的具体含义在 override_task_demo.cpp 中集中分配，主流程只负责编排顺序。
        api_.triggerOverrideTask(task_id);
        const bool ok = api_.waitOverrideComplete(override_timeout_);
        if (!ok) {
            ROS_WARN("[craic_demo] OVERRIDE_TASK result=timeout_or_forced_disable task=%d label=%s", task_id, label.c_str());
        } else {
            ROS_INFO("[craic_demo] OVERRIDE_TASK result=done task=%d label=%s control=EGO", task_id, label.c_str());
        }
    }

    void safeLand() {
        if (land_after_finish_) {
            ROS_WARN("[craic_demo] SAFE_LAND mission_aborted action=land");
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
    double mission_log_period_ = 1.0;
    bool simple_logs_ = true;
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
    double pillar_return_switch_after_distance_ = 0.45;
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
    double frame_auto_goal_z_min_ = 0.9;
    double frame_auto_goal_z_max_ = 1.65;
    double frame_auto_goal_z_fallback_ = 1.1;
    std::string goal_frame_id_ = "world";
    double ego_goal_z_limit_ = 2.0;
    double ego_goal_z_fallback_ = 0.9;
    double frame_pre_direct_timeout_ = 30.0;
    double frame_pre_direct_threshold_ = 0.20;
    bool enable_frame_opportunistic_lock_ = true;
    bool prefer_full_frame_lock_ = true;
    double frame_full_prefer_timeout_ = 4.0;
    bool frame_use_override_direct_pass_ = false;
    double frame_override_timeout_ = 20.0;
    double frame_override_speed_ = 0.18;
    double frame_override_lateral_kp_ = 0.8;
    double frame_override_max_lateral_speed_ = 0.12;
    double frame_override_z_kp_ = 0.8;
    double frame_override_max_vz_ = 0.10;
    double frame_override_yaw_kp_ = 0.7;
    double frame_override_max_yaw_rate_ = 0.25;
    double frame_override_lateral_threshold_ = 0.15;
    double frame_override_z_threshold_ = 0.12;
    double frame_override_yaw_threshold_ = 0.25;
    bool enable_frame_yaw_align_override_ = true;
    double frame_yaw_align_timeout_ = 4.0;
    double frame_yaw_align_threshold_ = 0.12;
    double frame_yaw_align_hold_time_ = 0.25;
    double frame_pre_offset_ = 0.70;
    double frame_post_offset_ = 0.70;
    bool enable_qr_ego_scan_ = true;
    double qr_gate_half_width_ = 0.5;
    double qr_forward_offset_ = 1.0;
    double qr_left_offset_ = 1.0;
    double qr_hold_time_ = 2.0;
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
