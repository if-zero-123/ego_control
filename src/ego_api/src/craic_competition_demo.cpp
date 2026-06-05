#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include "ego_api/ego_api.h"

namespace {

double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

bool finiteVec(const Eigen::Vector3d& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

double clampValue(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

const char* yesNo(bool value) {
    return value ? "yes" : "no";
}

geometry_msgs::PoseStamped makePose(const Eigen::Vector3d& pos, double yaw) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = pos.x();
    pose.pose.position.y = pos.y();
    pose.pose.position.z = pos.z();
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    return pose;
}

bool validFrameStatus(const std::string& status) {
    return status == "valid_frame_cluster" ||
           status == "valid_full" ||
           status == "valid_weak_pair" ||
           status == "valid_split_frame" ||
           status == "valid_partial_left" ||
           status == "valid_partial_right";
}

int frameStatusRank(const std::string& status) {
    if (status == "valid_frame_cluster" || status == "valid_full") return 3;
    if (status == "valid_split_frame" || status == "valid_weak_pair") return 2;
    if (status == "valid_partial_left" || status == "valid_partial_right") return 1;
    return 0;
}

const char* frameRankName(int rank) {
    if (rank >= 3) return "strong";
    if (rank == 2) return "medium";
    if (rank == 1) return "partial";
    return "invalid";
}

struct FrameCenterSample {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    ros::Time stamp;
    std::string status;
    int rank = 0;
    double dist_to_expected = std::numeric_limits<double>::infinity();
};

Eigen::Vector3d medianCenter(const std::vector<FrameCenterSample>& samples) {
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    xs.reserve(samples.size());
    ys.reserve(samples.size());
    zs.reserve(samples.size());
    for (const auto& sample : samples) {
        xs.push_back(sample.center.x());
        ys.push_back(sample.center.y());
        zs.push_back(sample.center.z());
    }

    auto median = [](std::vector<double>& values) {
        const std::size_t mid = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + mid, values.end());
        return values[mid];
    };
    return Eigen::Vector3d(median(xs), median(ys), median(zs));
}

enum class MoveResult {
    Reached,
    QrDetected,
    Timeout,
    RosStopped
};

struct BalloonServo {
    bool found = false;
    double u = 0.0;
    double v = 0.0;
    double err_u = 0.0;
    double err_v = 0.0;
    double area_ratio = 0.0;
    double confidence = 0.0;
    int state = 0;
    double bbox_x = 0.0;
    double bbox_y = 0.0;
    double bbox_w = 0.0;
    double bbox_h = 0.0;
};

struct VerifyRoi {
    double x = 0.0;
    double y = 0.0;
    double w = 0.0;
    double h = 0.0;
    double baseline_area = 0.0;
};

struct VerifyRoiResult {
    bool active = false;
    double area = 0.0;
    double area_ratio = 0.0;
    double baseline_area = 0.0;
};

class CraicCompetitionDemo {
public:
    CraicCompetitionDemo()
        : nh_(), pnh_("~"), api_(nh_, getBridgeNs()) {
        pnh_.param<double>("takeoff_timeout", takeoff_timeout_, 30.0);
        pnh_.param<double>("goal_timeout", goal_timeout_, 60.0);
        pnh_.param<double>("mission_log_period", mission_log_period_, 1.0);
        pnh_.param<bool>("simple_logs", simple_logs_, true);
        pnh_.param<bool>("land_after_finish", land_after_finish_, true);

        pnh_.param<double>("initial_wait_x", initial_wait_.x(), 0.0);
        pnh_.param<double>("initial_wait_y", initial_wait_.y(), -0.85);
        pnh_.param<double>("initial_wait_z", initial_wait_.z(), 1.0);

        pnh_.param<double>("expected_frame_x", expected_frame_center_.x(), 3.2);
        pnh_.param<double>("expected_frame_y", expected_frame_center_.y(), -1.25);
        pnh_.param<double>("expected_frame_z", expected_frame_center_.z(), 1.25);
        pnh_.param<std::string>("frame_center_mode", frame_center_mode_, "auto_detect");
        pnh_.param<double>("frame_center_reject_distance", frame_center_reject_distance_, 0.55);
        pnh_.param<double>("frame_post_x_offset", frame_post_x_offset_, 1.1);
        pnh_.param<double>("frame_pass_guard_x_offset", frame_pass_guard_x_offset_, 0.35);
        pnh_.param<double>("frame_pass_guard_timeout", frame_pass_guard_timeout_, 10.0);
        pnh_.param<double>("frame_detect_timeout", frame_detect_timeout_, 5.0);
        pnh_.param<double>("frame_valid_max_age", frame_valid_max_age_, 0.8);
        pnh_.param<double>("frame_lock_stability_threshold", frame_lock_stability_threshold_, 0.12);
        pnh_.param<int>("frame_strong_lock_frames", frame_strong_lock_frames_, 3);
        pnh_.param<int>("frame_medium_lock_frames", frame_medium_lock_frames_, 4);
        pnh_.param<int>("frame_partial_lock_frames", frame_partial_lock_frames_, 5);
        pnh_.param<double>("frame_medium_min_wait", frame_medium_min_wait_, 0.8);
        pnh_.param<double>("frame_partial_min_wait", frame_partial_min_wait_, 2.0);
        pnh_.param<double>("frame_lock_history_duration", frame_lock_history_duration_, 2.0);

        pnh_.param<double>("qr_goal_x", qr_goal_.x(), 4.0);
        pnh_.param<double>("qr_goal_y", qr_goal_.y(), 0.25);
        pnh_.param<double>("qr_goal_z", qr_goal_.z(), 1.3);
        pnh_.param<double>("qr_initial_wait", qr_initial_wait_, 1.0);
        pnh_.param<double>("qr_search_timeout", qr_search_timeout_, 10.0);
        pnh_.param<double>("qr_search_raise_z", qr_search_raise_z_, 0.2);
        pnh_.param<double>("qr_search_offset", qr_search_offset_, 0.3);
        pnh_.param<double>("qr_search_hold", qr_search_hold_, 0.35);

        pnh_.param<double>("attack_zone_x", attack_zone_.x(), 0.0);
        pnh_.param<double>("attack_zone_y", attack_zone_.y(), -0.8);
        pnh_.param<double>("attack_zone_z", attack_zone_.z(), 1.25);
        pnh_.param<double>("attack_height", attack_height_, 0.3);
        pnh_.param<double>("balloon_wait_timeout", balloon_wait_timeout_, 8.0);
        pnh_.param<double>("balloon_valid_max_age", balloon_valid_max_age_, 0.8);
        pnh_.param<double>("balloon_standoff", balloon_standoff_, 0.70);
        pnh_.param<double>("balloon_puncture_distance", balloon_puncture_distance_, 0.20);
        pnh_.param<double>("balloon_puncture_speed", balloon_puncture_speed_, 0.14);
        pnh_.param<double>("balloon_approach_timeout", balloon_approach_timeout_, 8.0);
        pnh_.param<double>("balloon_align_timeout", balloon_align_timeout_, 6.0);
        pnh_.param<double>("balloon_align_lateral_threshold", balloon_align_lateral_threshold_, 0.06);
        pnh_.param<double>("balloon_align_forward_threshold", balloon_align_forward_threshold_, 0.08);
        pnh_.param<double>("balloon_align_z_threshold", balloon_align_z_threshold_, 0.06);
        pnh_.param<double>("balloon_align_hold_time", balloon_align_hold_time_, 0.35);
        pnh_.param<double>("balloon_center_z_bias", balloon_center_z_bias_, 0.0);
        pnh_.param<double>("balloon_align_z_min", balloon_align_z_min_, 0.25);
        pnh_.param<double>("balloon_align_z_max", balloon_align_z_max_, 0.60);
        pnh_.param<double>("balloon_needle_length", balloon_needle_length_, 0.18);
        pnh_.param<double>("balloon_puncture_extra", balloon_puncture_extra_, 0.04);
        pnh_.param<double>("balloon_servo_timeout", balloon_servo_timeout_, 8.0);
        pnh_.param<double>("balloon_servo_u_threshold", balloon_servo_u_threshold_, 0.08);
        pnh_.param<double>("balloon_servo_v_threshold", balloon_servo_v_threshold_, 0.10);
        pnh_.param<double>("balloon_servo_hold_time", balloon_servo_hold_time_, 0.25);
        pnh_.param<double>("balloon_servo_lateral_gain", balloon_servo_lateral_gain_, 0.18);
        pnh_.param<double>("balloon_servo_z_gain", balloon_servo_z_gain_, 0.12);
        pnh_.param<double>("balloon_servo_max_lateral_speed", balloon_servo_max_lateral_speed_, 0.10);
        pnh_.param<double>("balloon_servo_max_z_speed", balloon_servo_max_z_speed_, 0.08);
        pnh_.param<double>("balloon_fine_forward_speed", balloon_fine_forward_speed_, 0.07);
        pnh_.param<double>("balloon_fine_forward_distance", balloon_fine_forward_distance_, 0.20);
        pnh_.param<double>("balloon_backoff_distance", balloon_backoff_distance_, 0.12);
        pnh_.param<double>("balloon_pop_verify_timeout", balloon_pop_verify_timeout_, 1.0);
        pnh_.param<double>("balloon_pop_area_drop_ratio", balloon_pop_area_drop_ratio_, 0.35);
        pnh_.param<double>("balloon_pop_verify_roi_scale", balloon_pop_verify_roi_scale_, 1.5);
        pnh_.param<double>("balloon_pop_verify_roi_min_px", balloon_pop_verify_roi_min_px_, 120.0);
        pnh_.param<int>("balloon_max_retry", balloon_max_retry_, 1);
        pnh_.param<double>("balloon_return_home_land_z", balloon_return_home_land_z_, 1.0);

        pnh_.param<double>("override_move_timeout", override_move_timeout_, 30.0);
        pnh_.param<double>("override_pos_threshold", override_pos_threshold_, 0.16);
        pnh_.param<double>("override_smooth_speed", override_smooth_speed_, 0.35);
        pnh_.param<double>("override_yaw_timeout", override_yaw_timeout_, 10.0);
        pnh_.param<double>("override_yaw_threshold", override_yaw_threshold_, 0.10);
        pnh_.param<double>("override_yaw_hold_time", override_yaw_hold_time_, 0.25);
        pnh_.param<double>("override_yaw_rate", override_yaw_rate_, 0.5);
        pnh_.param<double>("attack_zone_overrun_x", attack_zone_overrun_x_, 0.25);
        pnh_.param<bool>("robust_land_enable", robust_land_enable_, true);
        pnh_.param<double>("robust_land_soft_z", robust_land_soft_z_, 0.10);
        pnh_.param<double>("robust_land_soft_speed", robust_land_soft_speed_, 0.08);
        pnh_.param<double>("robust_land_z_threshold", robust_land_z_threshold_, 0.04);
        pnh_.param<double>("robust_land_settle_time", robust_land_settle_time_, 0.30);
        pnh_.param<double>("robust_land_press_z", robust_land_press_z_, -0.05);
        pnh_.param<double>("robust_land_disarm_timeout", robust_land_disarm_timeout_, 3.0);
        pnh_.param<double>("robust_land_disarm_retry_period", robust_land_disarm_retry_period_, 0.25);
        pnh_.param<bool>("robust_land_fallback_bridge_land", robust_land_fallback_bridge_land_, true);
        pnh_.param<std::string>("px4_land_mode", px4_land_mode_, "AUTO.LAND");
        pnh_.param<double>("px4_land_mode_timeout", px4_land_mode_timeout_, 5.0);
        pnh_.param<double>("px4_land_retry_period", px4_land_retry_period_, 0.5);

        pnh_.param<std::string>("frame_center_topic", frame_center_topic_, "/craic/frame_center");
        pnh_.param<std::string>("frame_status_topic", frame_status_topic_, "/craic/frame_status");
        pnh_.param<std::string>("qr_ids_topic", qr_ids_topic_, "/usb_camera_vision/aruco_ids");
        pnh_.param<std::string>("balloon_world_topic", balloon_world_topic_, "/balloon/world_point");
        pnh_.param<std::string>("balloon_servo_topic", balloon_servo_topic_, "/balloon/servo");
        pnh_.param<std::string>("balloon_verify_roi_topic", balloon_verify_roi_topic_, "/balloon/verify_roi");
        pnh_.param<std::string>("balloon_verify_roi_result_topic",
                                balloon_verify_roi_result_topic_,
                                "/balloon/verify_roi_result");

        frame_center_sub_ = nh_.subscribe(frame_center_topic_, 1, &CraicCompetitionDemo::frameCenterCb, this);
        frame_status_sub_ = nh_.subscribe(frame_status_topic_, 1, &CraicCompetitionDemo::frameStatusCb, this);
        qr_ids_sub_ = nh_.subscribe(qr_ids_topic_, 1, &CraicCompetitionDemo::qrIdsCb, this);
        balloon_world_sub_ = nh_.subscribe(balloon_world_topic_, 1, &CraicCompetitionDemo::balloonWorldCb, this);
        balloon_servo_sub_ = nh_.subscribe(balloon_servo_topic_, 1, &CraicCompetitionDemo::balloonServoCb, this);
        balloon_verify_result_sub_ = nh_.subscribe(balloon_verify_roi_result_topic_,
                                                  1,
                                                  &CraicCompetitionDemo::balloonVerifyResultCb,
                                                  this);
        balloon_verify_roi_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(balloon_verify_roi_topic_, 1, true);
        mavros_state_sub_ = nh_.subscribe("/mavros/state", 1, &CraicCompetitionDemo::mavrosStateCb, this);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    }

    int run() {
        if (!waitForBridge()) return 1;

        const double initial_yaw = api_.getOdomYaw();
        const double reverse_yaw = normalizeYaw(initial_yaw + M_PI);
        const double attack_yaw = normalizeYaw(initial_yaw - 0.5 * M_PI);
        ROS_INFO("[craic_demo] START initial_yaw=%.3f reverse_yaw=%.3f attack_yaw=%.3f",
                 initial_yaw, reverse_yaw, attack_yaw);

        logStage("TAKEOFF", "ego_bridge takeoff");
        if (!api_.takeoff(takeoff_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=TAKEOFF");
            return 1;
        }

        logStage("INITIAL_WAIT_OVERRIDE", "move to known first wait point");
        if (!overrideMoveTo("initial_wait", initial_wait_, initial_yaw,
                            override_pos_threshold_, override_move_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=INITIAL_WAIT_OVERRIDE");
            safeLand();
            return 1;
        }

        logStage("FRAME_CENTER", "wait frame center with expected fallback");
        const Eigen::Vector3d frame_center = waitFrameCenterWithFallback();
        const Eigen::Vector3d frame_post_control(frame_center.x() + frame_post_x_offset_,
                                                 frame_center.y(),
                                                 frame_center.z());
        ROS_INFO("[craic_demo] FRAME_POST_CONTROL center=(%.2f %.2f %.2f) target=(%.2f %.2f %.2f)",
                 frame_center.x(), frame_center.y(), frame_center.z(),
                 frame_post_control.x(), frame_post_control.y(), frame_post_control.z());

        logStage("FRAME_POST_EGO", "ego goal to frame post control with pass guard");
        if (!flyGoalWatchFramePass("frame_post_control", frame_post_control, initial_yaw, frame_center.x())) {
            ROS_ERROR("[craic_demo] FAIL stage=FRAME_POST_EGO");
            safeLand();
            return 1;
        }

        logStage("REVERSE_YAW_OVERRIDE", "turn to initial yaw + 180deg");
        if (!overrideYawAlign("reverse_yaw", reverse_yaw, override_yaw_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=REVERSE_YAW_OVERRIDE");
            safeLand();
            return 1;
        }

        resetQrDetection();
        logStage("QR_GOAL_OVERRIDE", "locked-yaw direct move to QR point");
        if (!overrideMoveTo("qr_goal", qr_goal_, reverse_yaw,
                            override_pos_threshold_, override_move_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=QR_GOAL_OVERRIDE");
            safeLand();
            return 1;
        }

        logStage("QR_RECOGNITION", "aruco immediate check then 10s small search");
        const bool qr_ok = waitQrOrSearch(qr_goal_, reverse_yaw);
        ROS_INFO("[craic_demo] QR_RESULT detected=%s", yesNo(qr_ok));

        logStage("RETURN_FRAME_POST_OVERRIDE", "diagonal direct return to saved frame post control");
        if (!overrideMoveTo("return_frame_post_control", frame_post_control, reverse_yaw,
                            override_pos_threshold_, override_move_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=RETURN_FRAME_POST_OVERRIDE");
            safeLand();
            return 1;
        }

        logStage("ATTACK_ZONE_EGO", "ego goal back to attack zone with x overrun guard");
        if (!flyGoalWatchXOverrun("attack_zone", attack_zone_, reverse_yaw)) {
            ROS_ERROR("[craic_demo] FAIL stage=ATTACK_ZONE_EGO");
            safeLand();
            return 1;
        }

        logStage("ATTACK_YAW_OVERRIDE", "turn right 90deg from initial yaw");
        if (!overrideYawAlign("attack_yaw", attack_yaw, override_yaw_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=ATTACK_YAW_OVERRIDE");
            safeLand();
            return 1;
        }

        logStage("LOWER_ATTACK_HEIGHT_OVERRIDE", "descend to attack height");
        Eigen::Vector3d low = api_.getOdomPosition();
        low.z() = attack_height_;
        if (!overrideMoveTo("attack_height", low, attack_yaw,
                            override_pos_threshold_, override_move_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=LOWER_ATTACK_HEIGHT_OVERRIDE");
            safeLand();
            return 1;
        }

        logStage("BALLOON_ATTACK_OVERRIDE", "3d coarse approach, 2d servo, roi verified puncture");
        const bool balloon_popped = attackBalloon(attack_yaw);
        if (balloon_popped) {
            logStage("BALLOON_RETURN_HOME", "return to (0,0,1) then PX4 LAND mode");
            if (returnHomeAndStartPx4Land(attack_yaw)) {
                ROS_INFO("[craic_demo] MISSION_DONE result=balloon_popped_px4_land_started");
                return 0;
            }
            ROS_WARN("[craic_demo] BALLOON_RETURN_HOME failed action=robust_land");
        } else {
            ROS_WARN("[craic_demo] BALLOON_ATTACK result=not_completed action=no_blind_puncture");
        }

        logStage("LAND", land_after_finish_ ? "robust soft land and disarm" : "hold and finish");
        if (land_after_finish_) {
            if (!robustLand(attack_yaw, "mission_finish", 30.0)) {
                ROS_WARN("[craic_demo] LAND result=timeout flight_state=%s", api_.getFlightState().c_str());
            }
        } else {
            holdOverride(0.5, attack_yaw);
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
        ROS_INFO("[craic_demo] TASK step=%s action=\"%s\" odom=(%.2f %.2f %.2f) yaw=%.3f mode=%u reach=%u",
                 stage.c_str(), action.c_str(),
                 pos.x(), pos.y(), pos.z(), api_.getOdomYaw(),
                 static_cast<unsigned int>(api_.getControlMode()),
                 static_cast<unsigned int>(api_.getReachStatus()));
    }

    bool flyGoal(const std::string& label, const Eigen::Vector3d& target, double yaw) {
        ROS_INFO("[craic_demo] EGO_GOAL name=%s world=(%.2f %.2f %.2f) yaw=%.3f",
                 label.c_str(), target.x(), target.y(), target.z(), yaw);
        const bool reached = api_.sendGoalWithYaw(target.x(), target.y(), target.z(), yaw, goal_timeout_);
        if (reached) {
            ROS_INFO("[craic_demo] EGO_RESULT name=%s reached", label.c_str());
        } else {
            ROS_WARN("[craic_demo] EGO_RESULT name=%s timeout=%.1f", label.c_str(), goal_timeout_);
        }
        return reached;
    }

    bool flyGoalWatchFramePass(const std::string& label,
                               const Eigen::Vector3d& target,
                               double yaw,
                               double frame_center_x) {
        const double pass_x = frame_center_x + frame_pass_guard_x_offset_;
        ROS_INFO("[craic_demo] EGO_GOAL_FRAME_GUARD name=%s world=(%.2f %.2f %.2f) yaw=%.3f pass_x>%.2f guard_timeout=%.1f goal_timeout=%.1f",
                 label.c_str(), target.x(), target.y(), target.z(), yaw,
                 pass_x, frame_pass_guard_timeout_, goal_timeout_);
        api_.publishGoalOnly(target.x(), target.y(), target.z(), yaw);

        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        bool saw_reach_reset = api_.getReachStatus() == 0;
        while (ros::ok()) {
            ros::spinOnce();

            if (api_.getReachStatus() == 0) {
                saw_reach_reset = true;
            }
            if (saw_reach_reset && api_.getReachStatus() == 1) {
                ROS_INFO("[craic_demo] EGO_GOAL_FRAME_GUARD result name=%s reached", label.c_str());
                return true;
            }

            const Eigen::Vector3d pos = api_.getOdomPosition();
            const double elapsed = (ros::Time::now() - start).toSec();
            if (elapsed >= frame_pass_guard_timeout_ && pos.x() > pass_x) {
                ROS_WARN("[craic_demo] EGO_GOAL_FRAME_GUARD passed_without_reach name=%s odom=(%.2f %.2f %.2f) pass_x=%.2f elapsed=%.1f action=continue_next_task",
                         label.c_str(), pos.x(), pos.y(), pos.z(), pass_x, elapsed);
                return true;
            }

            if (!simple_logs_) {
                const double dist = (pos - target).norm();
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] EGO_GOAL_FRAME_GUARD name=%s dist=%.3f odom=(%.2f %.2f %.2f) pass_x=%.2f elapsed=%.1f reach=%u",
                                  label.c_str(), dist, pos.x(), pos.y(), pos.z(), pass_x, elapsed,
                                  static_cast<unsigned int>(api_.getReachStatus()));
            }

            if (elapsed >= goal_timeout_) {
                ROS_WARN("[craic_demo] EGO_GOAL_FRAME_GUARD timeout name=%s timeout=%.1f odom=(%.2f %.2f %.2f) pass_x=%.2f",
                         label.c_str(), goal_timeout_, pos.x(), pos.y(), pos.z(), pass_x);
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool flyGoalWatchXOverrun(const std::string& label, const Eigen::Vector3d& target, double yaw) {
        ROS_INFO("[craic_demo] EGO_GOAL_GUARD name=%s world=(%.2f %.2f %.2f) yaw=%.3f overrun_x<%.2f timeout=%.1f",
                 label.c_str(), target.x(), target.y(), target.z(), yaw,
                 attack_zone_overrun_x_, goal_timeout_);
        api_.publishGoalOnly(target.x(), target.y(), target.z(), yaw);

        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        bool saw_reach_reset = api_.getReachStatus() == 0;
        while (ros::ok()) {
            ros::spinOnce();

            const Eigen::Vector3d pos = api_.getOdomPosition();
            if (pos.x() < attack_zone_overrun_x_) {
                ROS_WARN("[craic_demo] EGO_GOAL_GUARD overrun name=%s odom=(%.2f %.2f %.2f) limit_x=%.2f action=override_hold_continue",
                         label.c_str(), pos.x(), pos.y(), pos.z(), attack_zone_overrun_x_);
                if (!api_.enableOverride()) {
                    ROS_ERROR("[craic_demo] EGO_GOAL_GUARD failed name=%s reason=enable_override", label.c_str());
                    return false;
                }
                holdOverride(0.4, yaw);
                ROS_INFO("[craic_demo] EGO_GOAL_GUARD result name=%s overrun_handled", label.c_str());
                return true;
            }

            if (api_.getReachStatus() == 0) {
                saw_reach_reset = true;
            }
            if (saw_reach_reset && api_.getReachStatus() == 1) {
                ROS_INFO("[craic_demo] EGO_GOAL_GUARD result name=%s reached", label.c_str());
                return true;
            }

            if (!simple_logs_) {
                const double dist = (pos - target).norm();
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] EGO_GOAL_GUARD name=%s dist=%.3f odom=(%.2f %.2f %.2f) reach=%u",
                                  label.c_str(), dist, pos.x(), pos.y(), pos.z(),
                                  static_cast<unsigned int>(api_.getReachStatus()));
            }

            if ((ros::Time::now() - start).toSec() >= goal_timeout_) {
                ROS_WARN("[craic_demo] EGO_GOAL_GUARD timeout name=%s timeout=%.1f",
                         label.c_str(), goal_timeout_);
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool overrideMoveTo(const std::string& label,
                        const Eigen::Vector3d& target,
                        double yaw,
                        double threshold,
                        double timeout) {
        if (!finiteVec(target)) {
            ROS_ERROR("[craic_demo] OVERRIDE_MOVE invalid label=%s reason=non_finite", label.c_str());
            return false;
        }
        ROS_INFO("[craic_demo] OVERRIDE_MOVE start name=%s target=(%.2f %.2f %.2f) yaw=%.3f threshold=%.2f speed=%.2f timeout=%.1f",
                 label.c_str(), target.x(), target.y(), target.z(), yaw, threshold,
                 override_smooth_speed_, timeout);
        if (!api_.enableOverride()) {
            ROS_ERROR("[craic_demo] OVERRIDE_MOVE failed name=%s reason=enable_override", label.c_str());
            return false;
        }
        const MoveResult result = moveOverrideLoop(label, target, yaw, threshold,
                                                   ros::Time::now() + ros::Duration(timeout),
                                                   false);
        holdOverride(0.2, yaw);
        const bool disabled = api_.disableOverride();
        const bool reached = result == MoveResult::Reached;
        ROS_INFO("[craic_demo] OVERRIDE_MOVE result name=%s reached=%s disabled=%s",
                 label.c_str(), yesNo(reached), yesNo(disabled));
        return reached && disabled;
    }

    MoveResult moveOverrideLoop(const std::string& label,
                                const Eigen::Vector3d& target,
                                double yaw,
                                double threshold,
                                const ros::Time& deadline,
                                bool stop_on_qr) {
        ros::Rate rate(50);
        Eigen::Vector3d setpoint = api_.getOdomPosition();
        ros::Time last = ros::Time::now();
        while (ros::ok()) {
            ros::spinOnce();
            if (stop_on_qr && qrDetected()) {
                ROS_INFO("[craic_demo] OVERRIDE_MOVE interrupted_by=qr name=%s", label.c_str());
                return MoveResult::QrDetected;
            }

            const Eigen::Vector3d odom = api_.getOdomPosition();
            const double dist = (odom - target).norm();
            if (dist <= threshold) {
                sendPositionCmd(target, yaw);
                ROS_INFO("[craic_demo] OVERRIDE_MOVE reached name=%s dist=%.3f", label.c_str(), dist);
                return MoveResult::Reached;
            }

            const ros::Time now = ros::Time::now();
            const double dt = std::max(0.0, (now - last).toSec());
            last = now;
            const double speed = std::max(0.03, override_smooth_speed_);
            const double step = speed * dt;
            const Eigen::Vector3d remaining = target - setpoint;
            const double remaining_norm = remaining.norm();
            if (remaining_norm <= step || remaining_norm < 1e-4) {
                setpoint = target;
            } else if (step > 0.0) {
                setpoint += remaining / remaining_norm * step;
            }
            sendPositionCmd(setpoint, yaw);

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] OVERRIDE_MOVE name=%s setpoint=(%.2f %.2f %.2f) target=(%.2f %.2f %.2f) dist=%.3f yaw_err=%.3f",
                                  label.c_str(), setpoint.x(), setpoint.y(), setpoint.z(),
                                  target.x(), target.y(), target.z(), dist,
                                  normalizeYaw(yaw - api_.getOdomYaw()));
            }

            if (ros::Time::now() >= deadline) {
                ROS_WARN("[craic_demo] OVERRIDE_MOVE timeout name=%s dist=%.3f", label.c_str(), dist);
                return MoveResult::Timeout;
            }
            rate.sleep();
        }
        return MoveResult::RosStopped;
    }

    bool overrideYawAlign(const std::string& label, double target_yaw, double timeout) {
        const double start_yaw = api_.getOdomYaw();
        const double rate_limit = std::max(0.05, override_yaw_rate_);
        const double expected_turn_time = std::abs(normalizeYaw(target_yaw - start_yaw)) / rate_limit;
        const double effective_timeout = std::max(timeout, expected_turn_time + override_yaw_hold_time_ + 1.0);
        ROS_INFO("[craic_demo] YAW_ALIGN start name=%s target_yaw=%.3f current_yaw=%.3f rate=%.2f timeout=%.1f",
                 label.c_str(), target_yaw, start_yaw, rate_limit, effective_timeout);
        if (!api_.enableOverride()) {
            ROS_ERROR("[craic_demo] YAW_ALIGN failed name=%s reason=enable_override", label.c_str());
            return false;
        }

        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time last = start;
        ros::Time stable_since;
        bool stable = false;
        bool reached = false;
        double cmd_yaw = start_yaw;
        while (ros::ok()) {
            ros::spinOnce();

            const ros::Time now = ros::Time::now();
            const double dt = std::max(0.0, (now - last).toSec());
            last = now;
            const double cmd_err = normalizeYaw(target_yaw - cmd_yaw);
            const double yaw_step = rate_limit * dt;
            if (std::abs(cmd_err) <= yaw_step || std::abs(cmd_err) < 1e-4) {
                cmd_yaw = target_yaw;
            } else if (yaw_step > 0.0) {
                cmd_yaw = normalizeYaw(cmd_yaw + (cmd_err > 0.0 ? yaw_step : -yaw_step));
            }

            sendPositionCmd(api_.getOdomPosition(), cmd_yaw);
            const double err = normalizeYaw(target_yaw - api_.getOdomYaw());
            if (std::abs(err) <= override_yaw_threshold_) {
                if (!stable) {
                    stable = true;
                    stable_since = ros::Time::now();
                } else if ((ros::Time::now() - stable_since).toSec() >= override_yaw_hold_time_) {
                    reached = true;
                    break;
                }
            } else {
                stable = false;
            }
            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] YAW_ALIGN name=%s err=%.3f cmd=%.3f target=%.3f current=%.3f",
                                  label.c_str(), err, cmd_yaw, target_yaw, api_.getOdomYaw());
            }
            if ((ros::Time::now() - start).toSec() >= effective_timeout) {
                ROS_WARN("[craic_demo] YAW_ALIGN timeout name=%s err=%.3f", label.c_str(), err);
                break;
            }
            rate.sleep();
        }
        holdOverride(0.15, target_yaw);
        const bool disabled = api_.disableOverride();
        ROS_INFO("[craic_demo] YAW_ALIGN result name=%s reached=%s disabled=%s",
                 label.c_str(), yesNo(reached), yesNo(disabled));
        return reached && disabled;
    }

    Eigen::Vector3d waitFrameCenterWithFallback() {
        if (frame_center_mode_ == "expected_direct") {
            ROS_INFO("[craic_demo] FRAME_CENTER source=expected_direct expected=(%.2f %.2f %.2f)",
                     expected_frame_center_.x(), expected_frame_center_.y(), expected_frame_center_.z());
            return expected_frame_center_;
        }
        if (frame_center_mode_ != "auto_detect") {
            ROS_WARN("[craic_demo] FRAME_CENTER invalid_mode mode=%s action=auto_detect",
                     frame_center_mode_.c_str());
        }

        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        std::vector<FrameCenterSample> samples;
        ros::Time last_sample_stamp;
        int rejected_samples = 0;

        ROS_INFO("[craic_demo] FRAME_CENTER robust_lock timeout=%.1f expected=(%.2f %.2f %.2f) reject_dist=%.2f stable<=%.2f frames strong=%d medium=%d partial=%d",
                 frame_detect_timeout_,
                 expected_frame_center_.x(), expected_frame_center_.y(), expected_frame_center_.z(),
                 frame_center_reject_distance_, frame_lock_stability_threshold_,
                 frame_strong_lock_frames_, frame_medium_lock_frames_, frame_partial_lock_frames_);

        while (ros::ok() && (ros::Time::now() - start).toSec() < frame_detect_timeout_) {
            ros::spinOnce();
            if (freshFrameCenter() &&
                (!last_sample_stamp.isValid() || frame_center_recv_stamp_ != last_sample_stamp)) {
                last_sample_stamp = frame_center_recv_stamp_;

                FrameCenterSample sample;
                sample.center = frame_center_;
                sample.stamp = frame_center_recv_stamp_;
                sample.status = frame_status_;
                sample.rank = frameStatusRank(frame_status_);
                sample.dist_to_expected = (frame_center_ - expected_frame_center_).norm();

                if (sample.dist_to_expected <= frame_center_reject_distance_) {
                    samples.push_back(sample);
                    pruneFrameSamples(samples);
                } else {
                    ++rejected_samples;
                    ROS_WARN_THROTTLE(mission_log_period_,
                                      "[craic_demo] FRAME_CENTER reject_sample status=%s rank=%s center=(%.2f %.2f %.2f) dist=%.2f limit=%.2f action=continue_wait",
                                      sample.status.c_str(), frameRankName(sample.rank),
                                      sample.center.x(), sample.center.y(), sample.center.z(),
                                      sample.dist_to_expected, frame_center_reject_distance_);
                }
            }

            Eigen::Vector3d locked = Eigen::Vector3d::Zero();
            std::string lock_status;
            double lock_span = 0.0;
            int lock_used = 0;
            const double elapsed = (ros::Time::now() - start).toSec();
            if (tryLockFrameSamples(samples, 3, frame_strong_lock_frames_,
                                    &locked, &lock_status, &lock_span, &lock_used)) {
                logFrameLock("strong", lock_status, locked, lock_span, lock_used);
                return locked;
            }
            if (elapsed >= frame_medium_min_wait_ &&
                tryLockFrameSamples(samples, 2, frame_medium_lock_frames_,
                                    &locked, &lock_status, &lock_span, &lock_used)) {
                logFrameLock("medium", lock_status, locked, lock_span, lock_used);
                return locked;
            }
            if (elapsed >= frame_partial_min_wait_ &&
                tryLockFrameSamples(samples, 1, frame_partial_lock_frames_,
                                    &locked, &lock_status, &lock_span, &lock_used)) {
                logFrameLock("partial", lock_status, locked, lock_span, lock_used);
                return locked;
            }

            if (!simple_logs_) {
                int strong_count = 0;
                int medium_count = 0;
                int partial_count = 0;
                for (const auto& sample : samples) {
                    if (sample.rank >= 3) {
                        ++strong_count;
                    } else if (sample.rank == 2) {
                        ++medium_count;
                    } else if (sample.rank == 1) {
                        ++partial_count;
                    }
                }
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] FRAME_CENTER collecting elapsed=%.1f/%.1f status=%s have_center=%s samples strong=%d medium=%d partial=%d rejected=%d",
                                  elapsed, frame_detect_timeout_, frame_status_.c_str(),
                                  yesNo(have_frame_center_), strong_count, medium_count,
                                  partial_count, rejected_samples);
            }
            rate.sleep();
        }
        int strong_count = 0;
        int medium_count = 0;
        int partial_count = 0;
        for (const auto& sample : samples) {
            if (sample.rank >= 3) {
                ++strong_count;
            } else if (sample.rank == 2) {
                ++medium_count;
            } else if (sample.rank == 1) {
                ++partial_count;
            }
        }
        const double center_age = frame_center_recv_stamp_.isValid()
                                      ? (ros::Time::now() - frame_center_recv_stamp_).toSec()
                                      : -1.0;
        const double status_age = frame_status_stamp_.isValid()
                                      ? (ros::Time::now() - frame_status_stamp_).toSec()
                                      : -1.0;
        ROS_WARN("[craic_demo] FRAME_CENTER fallback reason=timeout_or_unstable timeout=%.1f samples=%zu strong=%d medium=%d partial=%d rejected=%d have_center=%s status=%s center_age=%.2f status_age=%.2f expected=(%.2f %.2f %.2f)",
                 frame_detect_timeout_, samples.size(), strong_count, medium_count, partial_count,
                 rejected_samples, yesNo(have_frame_center_), frame_status_.c_str(),
                 center_age, status_age,
                 expected_frame_center_.x(), expected_frame_center_.y(), expected_frame_center_.z());
        return expected_frame_center_;
    }

    bool freshFrameCenter() const {
        return have_frame_center_ &&
               validFrameStatus(frame_status_) &&
               frame_center_recv_stamp_.isValid() &&
               (ros::Time::now() - frame_center_recv_stamp_).toSec() <= frame_valid_max_age_ &&
               finiteVec(frame_center_);
    }

    void pruneFrameSamples(std::vector<FrameCenterSample>& samples) const {
        if (frame_lock_history_duration_ <= 0.0) return;
        const ros::Time now = ros::Time::now();
        samples.erase(std::remove_if(samples.begin(), samples.end(),
                                     [&](const FrameCenterSample& sample) {
                                         return (now - sample.stamp).toSec() > frame_lock_history_duration_;
                                     }),
                      samples.end());
    }

    bool tryLockFrameSamples(const std::vector<FrameCenterSample>& samples,
                             int min_rank,
                             int required_frames,
                             Eigen::Vector3d* locked_center,
                             std::string* lock_status,
                             double* lock_span,
                             int* used_frames) const {
        const int required = std::max(1, required_frames);
        std::vector<FrameCenterSample> ranked;
        ranked.reserve(samples.size());
        for (const auto& sample : samples) {
            if (sample.rank >= min_rank) {
                ranked.push_back(sample);
            }
        }
        if (ranked.size() < static_cast<std::size_t>(required)) return false;

        std::vector<FrameCenterSample> window(ranked.end() - required, ranked.end());
        const Eigen::Vector3d center = medianCenter(window);
        const double dist = (center - expected_frame_center_).norm();
        if (dist > frame_center_reject_distance_) return false;

        double span = 0.0;
        for (const auto& sample : window) {
            span = std::max(span, (sample.center - center).norm());
        }
        if (span > frame_lock_stability_threshold_) return false;

        *locked_center = center;
        *lock_status = window.back().status;
        *lock_span = span;
        *used_frames = required;
        return true;
    }

    void logFrameLock(const std::string& rank_label,
                      const std::string& status,
                      const Eigen::Vector3d& center,
                      double span,
                      int used_frames) const {
        const double dist = (center - expected_frame_center_).norm();
        ROS_INFO("[craic_demo] FRAME_CENTER source=detected lock=%s status=%s center=(%.2f %.2f %.2f) expected=(%.2f %.2f %.2f) dist=%.2f stable_span=%.3f frames=%d",
                 rank_label.c_str(), status.c_str(),
                 center.x(), center.y(), center.z(),
                 expected_frame_center_.x(), expected_frame_center_.y(), expected_frame_center_.z(),
                 dist, span, used_frames);
    }

    bool waitQrOrSearch(const Eigen::Vector3d& qr_goal, double yaw) {
        if (waitForQr(qr_initial_wait_)) {
            return true;
        }

        ROS_WARN("[craic_demo] QR not_detected_initial action=small_override_search timeout=%.1f",
                 qr_search_timeout_);
        if (!api_.enableOverride()) {
            ROS_WARN("[craic_demo] QR_SEARCH skipped reason=enable_override_failed");
            return false;
        }

        const ros::Time deadline = ros::Time::now() + ros::Duration(qr_search_timeout_);
        const Eigen::Vector2d forward(std::cos(yaw), std::sin(yaw));
        const Eigen::Vector2d left(-std::sin(yaw), std::cos(yaw));

        Eigen::Vector3d raised = qr_goal;
        raised.z() += qr_search_raise_z_;
        MoveResult result = moveOverrideLoop("qr_search_raise", raised, yaw,
                                             override_pos_threshold_, deadline, true);
        if (result == MoveResult::QrDetected) {
            holdOverride(0.15, yaw);
            api_.disableOverride();
            return true;
        }
        if (ros::Time::now() >= deadline || result == MoveResult::RosStopped) {
            holdOverride(0.15, yaw);
            api_.disableOverride();
            return qrDetected();
        }

        std::vector<std::pair<std::string, Eigen::Vector3d> > targets;
        targets.push_back(std::make_pair("qr_search_front",
                                         raised + Eigen::Vector3d(qr_search_offset_ * forward.x(),
                                                                  qr_search_offset_ * forward.y(), 0.0)));
        targets.push_back(std::make_pair("qr_search_back",
                                         raised - Eigen::Vector3d(qr_search_offset_ * forward.x(),
                                                                  qr_search_offset_ * forward.y(), 0.0)));
        targets.push_back(std::make_pair("qr_search_left",
                                         raised + Eigen::Vector3d(qr_search_offset_ * left.x(),
                                                                  qr_search_offset_ * left.y(), 0.0)));
        targets.push_back(std::make_pair("qr_search_right",
                                         raised - Eigen::Vector3d(qr_search_offset_ * left.x(),
                                                                  qr_search_offset_ * left.y(), 0.0)));

        for (const auto& item : targets) {
            if (ros::Time::now() >= deadline) break;
            result = moveOverrideLoop(item.first, item.second, yaw,
                                      override_pos_threshold_, deadline, true);
            if (result == MoveResult::QrDetected || qrDetected()) {
                holdOverride(0.15, yaw);
                api_.disableOverride();
                return true;
            }
            if (result == MoveResult::Reached && holdUntilQr(qr_search_hold_, deadline, yaw)) {
                api_.disableOverride();
                return true;
            }
        }

        holdOverride(0.15, yaw);
        api_.disableOverride();
        return qrDetected();
    }

    bool waitForQr(double timeout) {
        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < timeout) {
            ros::spinOnce();
            if (qrDetected()) {
                ROS_INFO("[craic_demo] QR detected ids_count=%zu", qr_ids_.size());
                return true;
            }
            rate.sleep();
        }
        return qrDetected();
    }

    bool holdUntilQr(double seconds, const ros::Time& deadline, double yaw) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        while (ros::ok() &&
               (ros::Time::now() - start).toSec() < seconds &&
               ros::Time::now() < deadline) {
            ros::spinOnce();
            if (qrDetected()) return true;
            sendPositionCmd(api_.getOdomPosition(), yaw);
            rate.sleep();
        }
        return qrDetected();
    }

    bool qrDetected() const {
        return qr_detected_ && qr_detect_stamp_.isValid();
    }

    void resetQrDetection() {
        qr_detected_ = false;
        qr_detect_stamp_ = ros::Time();
        qr_ids_.clear();
    }

    bool waitBalloonWorldPoint(Eigen::Vector3d& out) {
        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < balloon_wait_timeout_) {
            ros::spinOnce();
            if (freshBalloonWorldPoint()) {
                out = balloon_world_point_;
                ROS_INFO("[craic_demo] BALLOON_WORLD point=(%.2f %.2f %.2f)",
                         out.x(), out.y(), out.z());
                return true;
            }
            rate.sleep();
        }
        ROS_WARN("[craic_demo] BALLOON_WORLD timeout=%.1f have=%s",
                 balloon_wait_timeout_, yesNo(have_balloon_world_point_));
        return false;
    }

    bool freshBalloonWorldPoint() const {
        return have_balloon_world_point_ &&
               finiteVec(balloon_world_point_) &&
               balloon_world_stamp_.isValid() &&
               (ros::Time::now() - balloon_world_stamp_).toSec() <= balloon_valid_max_age_;
    }

    bool attackBalloon(double attack_yaw) {
        const Eigen::Vector2d forward(std::cos(attack_yaw), std::sin(attack_yaw));
        const Eigen::Vector2d left(-std::sin(attack_yaw), std::cos(attack_yaw));
        if (!api_.enableOverride()) {
            ROS_ERROR("[craic_demo] BALLOON_ATTACK failed reason=enable_override");
            return false;
        }

        bool popped = false;
        const int attempts = std::max(1, balloon_max_retry_ + 1);
        for (int attempt = 1; ros::ok() && attempt <= attempts; ++attempt) {
            clearVerifyRoi();
            Eigen::Vector3d balloon;
            if (!waitBalloonWorldPoint(balloon)) {
                ROS_WARN("[craic_demo] BALLOON_ATTACK attempt=%d result=no_world_point", attempt);
                break;
            }

            Eigen::Vector3d approach = balloonApproachTarget(balloon, forward);
            ROS_INFO("[craic_demo] BALLOON_APPROACH attempt=%d target=(%.2f %.2f %.2f) balloon=(%.2f %.2f %.2f) standoff=%.2f yaw=%.3f",
                     attempt,
                     approach.x(), approach.y(), approach.z(),
                     balloon.x(), balloon.y(), balloon.z(),
                     balloon_standoff_, attack_yaw);

            MoveResult approach_result = moveOverrideLoop("balloon_safe_standoff", approach, attack_yaw,
                                                          override_pos_threshold_,
                                                          ros::Time::now() + ros::Duration(balloon_approach_timeout_),
                                                          false);
            if (approach_result != MoveResult::Reached) {
                ROS_WARN("[craic_demo] BALLOON_APPROACH attempt=%d result=not_reached", attempt);
                continue;
            }

            BalloonServo locked_servo;
            if (!servoAlign(attack_yaw, forward, left, locked_servo)) {
                ROS_WARN("[craic_demo] BALLOON_SERVO_ALIGN attempt=%d result=timeout", attempt);
                continue;
            }

            if (!servoFineForward(attack_yaw, forward, left, locked_servo)) {
                ROS_WARN("[craic_demo] BALLOON_FINE_FORWARD attempt=%d result=failed", attempt);
                continue;
            }

            if (!freshBalloonServo()) {
                ROS_WARN("[craic_demo] BALLOON_LOCK_ROI attempt=%d result=no_fresh_servo", attempt);
                continue;
            }
            locked_servo = balloon_servo_;
            VerifyRoi roi = makeVerifyRoi(locked_servo);
            publishVerifyRoi(roi);
            if (waitVerifyRoiBaseline(roi)) {
                publishVerifyRoi(roi);
            }
            ROS_INFO("[craic_demo] BALLOON_LOCK_ROI attempt=%d center=(%.1f %.1f) bbox=(%.1f %.1f %.1f %.1f) roi=(%.1f %.1f %.1f %.1f) baseline=%.0f",
                     attempt,
                     locked_servo.u, locked_servo.v,
                     locked_servo.bbox_x, locked_servo.bbox_y, locked_servo.bbox_w, locked_servo.bbox_h,
                     roi.x, roi.y, roi.w, roi.h, roi.baseline_area);

            if (!driveAlongForward("balloon_puncture", attack_yaw, forward,
                                   balloon_puncture_speed_, balloon_puncture_distance_)) {
                ROS_WARN("[craic_demo] BALLOON_PUNCTURE attempt=%d result=distance_not_reached", attempt);
            }

            driveAlongForward("balloon_backoff", attack_yaw, forward,
                              -std::abs(balloon_puncture_speed_), balloon_backoff_distance_);
            popped = verifyBalloonPopped(roi);
            ROS_INFO("[craic_demo] BALLOON_VERIFY attempt=%d popped=%s", attempt, yesNo(popped));
            if (popped) {
                break;
            }
        }
        holdOverride(0.25, attack_yaw);
        const bool disabled = api_.disableOverride();
        clearVerifyRoi();
        ROS_INFO("[craic_demo] BALLOON_ATTACK result=%s disabled=%s", yesNo(popped), yesNo(disabled));
        return popped && disabled;
    }

    Eigen::Vector3d balloonApproachTarget(const Eigen::Vector3d& balloon,
                                          const Eigen::Vector2d& forward) const {
        const double z = attack_height_;
        return Eigen::Vector3d(balloon.x() - balloon_standoff_ * forward.x(),
                               balloon.y() - balloon_standoff_ * forward.y(),
                               z);
    }

    bool freshBalloonServo() const {
        return have_balloon_servo_ &&
               balloon_servo_.found &&
               balloon_servo_stamp_.isValid() &&
               (ros::Time::now() - balloon_servo_stamp_).toSec() <= balloon_valid_max_age_ &&
               std::isfinite(balloon_servo_.err_u) &&
               std::isfinite(balloon_servo_.err_v) &&
               std::isfinite(balloon_servo_.area_ratio);
    }

    bool servoCentered(const BalloonServo& servo) const {
        return servo.found &&
               std::abs(servo.err_u) <= balloon_servo_u_threshold_ &&
               std::abs(servo.err_v) <= balloon_servo_v_threshold_;
    }

    bool servoAlign(double yaw,
                    const Eigen::Vector2d& forward,
                    const Eigen::Vector2d& left,
                    BalloonServo& locked) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time stable_since;
        bool stable = false;
        while (ros::ok() && (ros::Time::now() - start).toSec() < balloon_servo_timeout_) {
            ros::spinOnce();
            if (!freshBalloonServo()) {
                sendPositionCmd(api_.getOdomPosition(), yaw);
                stable = false;
                rate.sleep();
                continue;
            }

            const BalloonServo servo = balloon_servo_;
            const double lateral_speed = clampValue(-balloon_servo_lateral_gain_ * servo.err_u,
                                                    -balloon_servo_max_lateral_speed_,
                                                    balloon_servo_max_lateral_speed_);
            const double z_speed = clampValue(-balloon_servo_z_gain_ * servo.err_v,
                                              -balloon_servo_max_z_speed_,
                                              balloon_servo_max_z_speed_);
            sendVelocityCmdWithYaw(lateral_speed * left.x(),
                                   lateral_speed * left.y(),
                                   z_speed,
                                   yaw);

            if (servoCentered(servo)) {
                if (!stable) {
                    stable = true;
                    stable_since = ros::Time::now();
                } else if ((ros::Time::now() - stable_since).toSec() >= balloon_servo_hold_time_) {
                    locked = servo;
                    ROS_INFO("[craic_demo] BALLOON_SERVO_ALIGN centered u=%.1f v=%.1f err=(%.3f %.3f) area_ratio=%.3f bbox=(%.1f %.1f %.1f %.1f)",
                             servo.u, servo.v, servo.err_u, servo.err_v, servo.area_ratio,
                             servo.bbox_x, servo.bbox_y, servo.bbox_w, servo.bbox_h);
                    return true;
                }
            } else {
                stable = false;
            }

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] BALLOON_SERVO_ALIGN err=(%.3f %.3f) speed_lat=%.3f speed_z=%.3f centered=%s area_ratio=%.3f",
                                  servo.err_u, servo.err_v, lateral_speed, z_speed,
                                  yesNo(servoCentered(servo)), servo.area_ratio);
            }
            rate.sleep();
        }
        holdOverride(0.15, yaw);
        return false;
    }

    bool servoFineForward(double yaw,
                          const Eigen::Vector2d& forward,
                          const Eigen::Vector2d& left,
                          BalloonServo& locked) {
        const Eigen::Vector3d start = api_.getOdomPosition();
        const ros::Time deadline = ros::Time::now() + ros::Duration(
            std::max(2.0, balloon_fine_forward_distance_ / std::max(0.03, balloon_fine_forward_speed_) + 1.0));
        ros::Rate rate(50);
        bool reached = false;
        while (ros::ok() && ros::Time::now() < deadline) {
            ros::spinOnce();
            double lateral_speed = 0.0;
            double z_speed = 0.0;
            if (freshBalloonServo()) {
                locked = balloon_servo_;
                lateral_speed = clampValue(-balloon_servo_lateral_gain_ * locked.err_u,
                                           -balloon_servo_max_lateral_speed_,
                                           balloon_servo_max_lateral_speed_);
                z_speed = clampValue(-balloon_servo_z_gain_ * locked.err_v,
                                     -balloon_servo_max_z_speed_,
                                     balloon_servo_max_z_speed_);
            }

            sendVelocityCmdWithYaw(balloon_fine_forward_speed_ * forward.x() + lateral_speed * left.x(),
                                   balloon_fine_forward_speed_ * forward.y() + lateral_speed * left.y(),
                                   z_speed,
                                   yaw);
            const Eigen::Vector3d now = api_.getOdomPosition();
            const Eigen::Vector2d delta(now.x() - start.x(), now.y() - start.y());
            const double progress = delta.dot(forward);
            if (progress >= balloon_fine_forward_distance_) {
                reached = true;
                break;
            }
            rate.sleep();
        }
        holdOverride(0.15, yaw);
        ROS_INFO("[craic_demo] BALLOON_FINE_FORWARD reached=%s distance=%.2f speed=%.2f",
                 yesNo(reached), balloon_fine_forward_distance_, balloon_fine_forward_speed_);
        return reached;
    }

    bool driveAlongForward(const std::string& label,
                           double yaw,
                           const Eigen::Vector2d& forward,
                           double speed,
                           double distance) {
        const double abs_distance = std::abs(distance);
        const double signed_speed = speed;
        const Eigen::Vector3d start = api_.getOdomPosition();
        const ros::Time deadline = ros::Time::now() + ros::Duration(
            std::max(2.0, abs_distance / std::max(0.03, std::abs(signed_speed)) + 1.0));
        ros::Rate rate(50);
        bool reached = false;
        while (ros::ok() && ros::Time::now() < deadline) {
            ros::spinOnce();
            sendVelocityCmdWithYaw(signed_speed * forward.x(),
                                   signed_speed * forward.y(),
                                   0.0,
                                   yaw);
            const Eigen::Vector3d now = api_.getOdomPosition();
            const Eigen::Vector2d delta(now.x() - start.x(), now.y() - start.y());
            const double progress = std::abs(delta.dot(forward));
            if (progress >= abs_distance) {
                reached = true;
                break;
            }
            rate.sleep();
        }
        holdOverride(0.15, yaw);
        ROS_INFO("[craic_demo] DRIVE_FORWARD name=%s reached=%s distance=%.2f speed=%.2f",
                 label.c_str(), yesNo(reached), abs_distance, signed_speed);
        return reached;
    }

    VerifyRoi makeVerifyRoi(const BalloonServo& servo) const {
        VerifyRoi roi;
        const double raw_w = std::max(servo.bbox_w * balloon_pop_verify_roi_scale_,
                                      balloon_pop_verify_roi_min_px_);
        const double raw_h = std::max(servo.bbox_h * balloon_pop_verify_roi_scale_,
                                      balloon_pop_verify_roi_min_px_);
        roi.w = raw_w;
        roi.h = raw_h;
        roi.x = servo.u - 0.5 * raw_w;
        roi.y = servo.v - 0.5 * raw_h;
        roi.baseline_area = std::max(1.0, servo.area_ratio * raw_w * raw_h);
        return roi;
    }

    void publishVerifyRoi(const VerifyRoi& roi) {
        std_msgs::Float32MultiArray msg;
        msg.data = {
            1.0f,
            static_cast<float>(roi.x),
            static_cast<float>(roi.y),
            static_cast<float>(roi.w),
            static_cast<float>(roi.h),
            static_cast<float>(roi.baseline_area),
        };
        balloon_verify_roi_pub_.publish(msg);
    }

    void clearVerifyRoi() {
        std_msgs::Float32MultiArray msg;
        msg.data = {0.0f};
        balloon_verify_roi_pub_.publish(msg);
        have_verify_roi_result_ = false;
    }

    bool waitVerifyRoiBaseline(VerifyRoi& roi) {
        ros::Rate rate(30);
        const ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < 0.5) {
            ros::spinOnce();
            if (freshVerifyRoiResult() && verify_roi_result_.area > 1.0) {
                roi.baseline_area = verify_roi_result_.area;
                ROS_INFO("[craic_demo] BALLOON_LOCK_ROI baseline_sample area=%.0f ratio=%.3f",
                         verify_roi_result_.area, verify_roi_result_.area_ratio);
                return true;
            }
            rate.sleep();
        }
        ROS_WARN("[craic_demo] BALLOON_LOCK_ROI baseline_sample unavailable fallback=%.0f",
                 roi.baseline_area);
        return false;
    }

    bool verifyBalloonPopped(const VerifyRoi& roi) {
        ros::Rate rate(30);
        const ros::Time start = ros::Time::now();
        ros::Time empty_since;
        bool empty_active = false;
        const double threshold = std::max(1.0, roi.baseline_area * balloon_pop_area_drop_ratio_);
        while (ros::ok() && (ros::Time::now() - start).toSec() < balloon_pop_verify_timeout_) {
            ros::spinOnce();
            if (!freshVerifyRoiResult()) {
                rate.sleep();
                continue;
            }

            const double area = verify_roi_result_.area;
            if (area <= threshold) {
                ROS_INFO("[craic_demo] BALLOON_POP_VERIFY success reason=area_drop area=%.0f threshold=%.0f baseline=%.0f",
                         area, threshold, roi.baseline_area);
                return true;
            }

            if (area <= 1.0) {
                if (!empty_active) {
                    empty_active = true;
                    empty_since = ros::Time::now();
                } else if ((ros::Time::now() - empty_since).toSec() >= balloon_pop_verify_timeout_) {
                    ROS_INFO("[craic_demo] BALLOON_POP_VERIFY success reason=roi_empty");
                    return true;
                }
            } else {
                empty_active = false;
            }
            rate.sleep();
        }
        if (freshVerifyRoiResult()) {
            ROS_WARN("[craic_demo] BALLOON_POP_VERIFY failed area=%.0f threshold=%.0f baseline=%.0f",
                     verify_roi_result_.area, threshold, roi.baseline_area);
        } else {
            ROS_WARN("[craic_demo] BALLOON_POP_VERIFY failed reason=no_roi_feedback");
        }
        return false;
    }

    bool freshVerifyRoiResult() const {
        return have_verify_roi_result_ &&
               verify_roi_result_.active &&
               verify_roi_result_stamp_.isValid() &&
               (ros::Time::now() - verify_roi_result_stamp_).toSec() <= balloon_valid_max_age_;
    }

    bool returnHomeAndStartPx4Land(double yaw) {
        if (!api_.enableOverride()) {
            ROS_ERROR("[craic_demo] RETURN_HOME failed reason=enable_override");
            return false;
        }
        const Eigen::Vector3d home(0.0, 0.0, balloon_return_home_land_z_);
        const MoveResult result = moveOverrideLoop("balloon_return_home_land_point", home, yaw,
                                                   override_pos_threshold_,
                                                   ros::Time::now() + ros::Duration(override_move_timeout_),
                                                   false);
        holdOverride(0.2, yaw);
        const bool disabled = api_.disableOverride();
        if (result != MoveResult::Reached || !disabled) {
            ROS_WARN("[craic_demo] RETURN_HOME result=reached_%s disabled=%s",
                     yesNo(result == MoveResult::Reached), yesNo(disabled));
            return false;
        }
        if (land_after_finish_) {
            if (startPx4LandMode("balloon_return_home")) {
                return true;
            }
            return robustLand(yaw, "px4_land_start_failed", 30.0);
        }
        ROS_WARN("[craic_demo] RETURN_HOME land skipped land_after_finish=false");
        return true;
    }

    bool startPx4LandMode(const std::string& reason) {
        if (mavrosDisarmed()) {
            ROS_INFO("[craic_demo] PX4_LAND already_disarmed reason=%s", reason.c_str());
            return true;
        }

        ROS_INFO("[craic_demo] PX4_LAND start reason=%s mode=%s timeout=%.1f",
                 reason.c_str(), px4_land_mode_.c_str(), px4_land_mode_timeout_);
        api_.disableOverride();

        ros::Rate rate(10);
        const ros::Time start = ros::Time::now();
        ros::Time last_try(0);
        while (ros::ok() && (ros::Time::now() - start).toSec() < px4_land_mode_timeout_) {
            ros::spinOnce();
            if (mavrosModeLooksLand()) {
                ROS_INFO("[craic_demo] PX4_LAND confirmed mode=%s armed=%s",
                         mavros_mode_.c_str(), yesNo(mavros_armed_));
                return true;
            }

            const ros::Time now = ros::Time::now();
            if (!last_try.isValid() || (now - last_try).toSec() >= px4_land_retry_period_) {
                last_try = now;
                requestPx4LandModeOnce();
            }
            rate.sleep();
        }

        ROS_WARN("[craic_demo] PX4_LAND timeout mode=%s armed=%s",
                 mavros_mode_.c_str(), yesNo(mavros_armed_));
        return mavrosModeLooksLand();
    }

    bool requestPx4LandModeOnce() {
        mavros_msgs::SetMode req;
        req.request.base_mode = 0;
        req.request.custom_mode = px4_land_mode_;
        if (set_mode_client_.call(req) && req.response.mode_sent) {
            ROS_INFO("[craic_demo] PX4_LAND set_mode accepted mode=%s", px4_land_mode_.c_str());
            return true;
        }
        ROS_WARN_THROTTLE(1.0, "[craic_demo] PX4_LAND set_mode rejected_or_unavailable mode=%s",
                          px4_land_mode_.c_str());
        return false;
    }

    bool mavrosModeLooksLand() const {
        return have_mavros_state_ && mavros_mode_.find("LAND") != std::string::npos;
    }

    bool robustLand(double yaw, const std::string& reason, double fallback_timeout) {
        if (!land_after_finish_) {
            ROS_WARN("[craic_demo] ROBUST_LAND skipped reason=%s land_after_finish=false", reason.c_str());
            return true;
        }
        if (api_.getFlightState() == "IDLE" || mavrosDisarmed()) {
            ROS_INFO("[craic_demo] ROBUST_LAND already_disarmed_or_idle reason=%s", reason.c_str());
            return true;
        }
        if (!robust_land_enable_) {
            ROS_WARN("[craic_demo] ROBUST_LAND disabled reason=%s action=bridge_land", reason.c_str());
            return api_.land(fallback_timeout);
        }

        ROS_INFO("[craic_demo] ROBUST_LAND start reason=%s soft_z=%.2f press_z=%.2f",
                 reason.c_str(), robust_land_soft_z_, robust_land_press_z_);
        if (!api_.enableOverride()) {
            ROS_WARN("[craic_demo] ROBUST_LAND enable_override_failed reason=%s", reason.c_str());
            return fallbackBridgeLand(reason, fallback_timeout);
        }

        const Eigen::Vector3d anchor = api_.getOdomPosition();
        const bool descended = softDescendToZ(anchor, yaw);
        holdOverride(robust_land_settle_time_, yaw);

        bool disarmed = false;
        if (descended || api_.getOdomPosition().z() <= robust_land_soft_z_ + robust_land_z_threshold_) {
            disarmed = pressAndDisarm(anchor, yaw);
        } else {
            ROS_WARN("[craic_demo] ROBUST_LAND soft_descent_failed reason=%s z=%.2f target=%.2f",
                     reason.c_str(), api_.getOdomPosition().z(), robust_land_soft_z_);
        }

        const bool override_disabled = api_.disableOverride();
        if (disarmed || mavrosDisarmed()) {
            ROS_INFO("[craic_demo] ROBUST_LAND result=disarmed reason=%s override_disabled=%s",
                     reason.c_str(), yesNo(override_disabled));
            return true;
        }

        ROS_WARN("[craic_demo] ROBUST_LAND result=not_disarmed reason=%s override_disabled=%s",
                 reason.c_str(), yesNo(override_disabled));
        return fallbackBridgeLand(reason, fallback_timeout);
    }

    bool softDescendToZ(const Eigen::Vector3d& anchor, double yaw) {
        const double start_z = api_.getOdomPosition().z();
        if (start_z <= robust_land_soft_z_ + robust_land_z_threshold_) {
            ROS_INFO("[craic_demo] ROBUST_LAND soft_descent skipped z=%.2f target=%.2f",
                     start_z, robust_land_soft_z_);
            return true;
        }

        const double speed = std::max(0.03, robust_land_soft_speed_);
        const double timeout = std::max(4.0, (start_z - robust_land_soft_z_) / speed + 3.0);
        const ros::Time start = ros::Time::now();
        ros::Time last = start;
        ros::Rate rate(50);
        double cmd_z = start_z;

        while (ros::ok() && (ros::Time::now() - start).toSec() < timeout) {
            ros::spinOnce();
            const ros::Time now = ros::Time::now();
            const double dt = std::max(0.0, (now - last).toSec());
            last = now;
            cmd_z = std::max(robust_land_soft_z_, cmd_z - speed * dt);

            sendPositionCmd(Eigen::Vector3d(anchor.x(), anchor.y(), cmd_z), yaw);
            const double z = api_.getOdomPosition().z();
            if (z <= robust_land_soft_z_ + robust_land_z_threshold_) {
                ROS_INFO("[craic_demo] ROBUST_LAND soft_descent reached z=%.2f target=%.2f",
                         z, robust_land_soft_z_);
                return true;
            }
            rate.sleep();
        }

        ROS_WARN("[craic_demo] ROBUST_LAND soft_descent timeout z=%.2f target=%.2f",
                 api_.getOdomPosition().z(), robust_land_soft_z_);
        return false;
    }

    bool pressAndDisarm(const Eigen::Vector3d& anchor, double yaw) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time last_try(0);
        const Eigen::Vector3d press_target(anchor.x(), anchor.y(), robust_land_press_z_);

        while (ros::ok() &&
               (ros::Time::now() - start).toSec() < robust_land_disarm_timeout_) {
            ros::spinOnce();
            sendPositionCmd(press_target, yaw);

            if (mavrosDisarmed()) {
                return true;
            }

            const ros::Time now = ros::Time::now();
            if (!last_try.isValid() ||
                (now - last_try).toSec() >= robust_land_disarm_retry_period_) {
                last_try = now;
                if (requestDisarmOnce()) {
                    return true;
                }
            }
            rate.sleep();
        }
        return mavrosDisarmed();
    }

    bool requestDisarmOnce() {
        mavros_msgs::CommandBool req;
        req.request.value = false;
        if (arming_client_.call(req) && req.response.success) {
            ROS_INFO("[craic_demo] ROBUST_LAND disarm service accepted");
            return true;
        }
        ROS_WARN_THROTTLE(1.0, "[craic_demo] ROBUST_LAND disarm service rejected_or_unavailable");
        return mavrosDisarmed();
    }

    bool mavrosDisarmed() const {
        return have_mavros_state_ &&
               mavros_state_stamp_.isValid() &&
               (ros::Time::now() - mavros_state_stamp_).toSec() <= 1.0 &&
               !mavros_armed_;
    }

    bool fallbackBridgeLand(const std::string& reason, double timeout) {
        if (!robust_land_fallback_bridge_land_) {
            ROS_WARN("[craic_demo] ROBUST_LAND fallback disabled reason=%s", reason.c_str());
            return false;
        }
        ROS_WARN("[craic_demo] ROBUST_LAND fallback action=ego_bridge_land reason=%s", reason.c_str());
        return api_.land(timeout);
    }

    void sendPositionCmd(const Eigen::Vector3d& target, double yaw) {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        cmd.position.x = target.x();
        cmd.position.y = target.y();
        cmd.position.z = target.z();
        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
        cmd.yaw = yaw;
        cmd.yaw_dot = 0.0;
        cmd.trajectory_id = 0;
        cmd.trajectory_flag = 0;
        api_.sendOverrideCmd(cmd);
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

    void holdOverride(double seconds, double yaw) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < seconds) {
            ros::spinOnce();
            sendPositionCmd(api_.getOdomPosition(), yaw);
            rate.sleep();
        }
    }

    void safeLand() {
        if (land_after_finish_) {
            robustLand(api_.getOdomYaw(), "safe_land", 20.0);
        }
    }

    void frameCenterCb(const geometry_msgs::PointStamped::ConstPtr& msg) {
        frame_center_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
        frame_center_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        frame_center_recv_stamp_ = ros::Time::now();
        have_frame_center_ = true;
    }

    void frameStatusCb(const std_msgs::String::ConstPtr& msg) {
        frame_status_ = msg->data;
        frame_status_stamp_ = ros::Time::now();
    }

    void qrIdsCb(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        if (!msg->data.empty()) {
            qr_detected_ = true;
            qr_detect_stamp_ = ros::Time::now();
            qr_ids_.assign(msg->data.begin(), msg->data.end());
        }
    }

    void balloonWorldCb(const geometry_msgs::PointStamped::ConstPtr& msg) {
        balloon_world_point_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
        balloon_world_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        have_balloon_world_point_ = true;
    }

    void balloonServoCb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 12) {
            have_balloon_servo_ = false;
            return;
        }
        balloon_servo_.found = msg->data[0] > 0.5f;
        balloon_servo_.u = msg->data[1];
        balloon_servo_.v = msg->data[2];
        balloon_servo_.err_u = msg->data[3];
        balloon_servo_.err_v = msg->data[4];
        balloon_servo_.area_ratio = msg->data[5];
        balloon_servo_.confidence = msg->data[6];
        balloon_servo_.state = static_cast<int>(std::round(msg->data[7]));
        balloon_servo_.bbox_x = msg->data[8];
        balloon_servo_.bbox_y = msg->data[9];
        balloon_servo_.bbox_w = msg->data[10];
        balloon_servo_.bbox_h = msg->data[11];
        balloon_servo_stamp_ = ros::Time::now();
        have_balloon_servo_ = true;
    }

    void balloonVerifyResultCb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() < 4) {
            have_verify_roi_result_ = false;
            return;
        }
        verify_roi_result_.active = msg->data[0] > 0.5f;
        verify_roi_result_.area = msg->data[1];
        verify_roi_result_.area_ratio = msg->data[2];
        verify_roi_result_.baseline_area = msg->data[3];
        verify_roi_result_stamp_ = ros::Time::now();
        have_verify_roi_result_ = true;
    }

    void mavrosStateCb(const mavros_msgs::State::ConstPtr& msg) {
        mavros_armed_ = msg->armed;
        mavros_mode_ = msg->mode;
        mavros_state_stamp_ = ros::Time::now();
        have_mavros_state_ = true;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    EgoApi api_;

    ros::Subscriber frame_center_sub_;
    ros::Subscriber frame_status_sub_;
    ros::Subscriber qr_ids_sub_;
    ros::Subscriber balloon_world_sub_;
    ros::Subscriber balloon_servo_sub_;
    ros::Subscriber balloon_verify_result_sub_;
    ros::Subscriber mavros_state_sub_;
    ros::Publisher balloon_verify_roi_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    std::string frame_center_topic_;
    std::string frame_status_topic_;
    std::string qr_ids_topic_;
    std::string balloon_world_topic_;
    std::string balloon_servo_topic_;
    std::string balloon_verify_roi_topic_;
    std::string balloon_verify_roi_result_topic_;

    double takeoff_timeout_ = 30.0;
    double goal_timeout_ = 60.0;
    double mission_log_period_ = 1.0;
    bool simple_logs_ = true;
    bool land_after_finish_ = true;

    Eigen::Vector3d initial_wait_ = Eigen::Vector3d(0.0, -0.85, 1.0);
    Eigen::Vector3d expected_frame_center_ = Eigen::Vector3d(3.2, -1.25, 1.25);
    std::string frame_center_mode_ = "auto_detect";
    double frame_center_reject_distance_ = 0.55;
    double frame_post_x_offset_ = 1.1;
    double frame_pass_guard_x_offset_ = 0.35;
    double frame_pass_guard_timeout_ = 10.0;
    double frame_detect_timeout_ = 5.0;
    double frame_valid_max_age_ = 0.8;
    double frame_lock_stability_threshold_ = 0.12;
    int frame_strong_lock_frames_ = 3;
    int frame_medium_lock_frames_ = 4;
    int frame_partial_lock_frames_ = 5;
    double frame_medium_min_wait_ = 0.8;
    double frame_partial_min_wait_ = 2.0;
    double frame_lock_history_duration_ = 2.0;

    Eigen::Vector3d qr_goal_ = Eigen::Vector3d(4.0, 0.25, 1.3);
    double qr_initial_wait_ = 1.0;
    double qr_search_timeout_ = 10.0;
    double qr_search_raise_z_ = 0.2;
    double qr_search_offset_ = 0.3;
    double qr_search_hold_ = 0.35;

    Eigen::Vector3d attack_zone_ = Eigen::Vector3d(0.0, -0.8, 1.25);
    double attack_height_ = 0.3;
    double balloon_wait_timeout_ = 8.0;
    double balloon_valid_max_age_ = 0.8;
    double balloon_standoff_ = 0.70;
    double balloon_puncture_distance_ = 0.20;
    double balloon_puncture_speed_ = 0.14;
    double balloon_approach_timeout_ = 8.0;
    double balloon_align_timeout_ = 6.0;
    double balloon_align_lateral_threshold_ = 0.06;
    double balloon_align_forward_threshold_ = 0.08;
    double balloon_align_z_threshold_ = 0.06;
    double balloon_align_hold_time_ = 0.35;
    double balloon_center_z_bias_ = 0.0;
    double balloon_align_z_min_ = 0.25;
    double balloon_align_z_max_ = 0.60;
    double balloon_needle_length_ = 0.18;
    double balloon_puncture_extra_ = 0.04;
    double balloon_servo_timeout_ = 8.0;
    double balloon_servo_u_threshold_ = 0.08;
    double balloon_servo_v_threshold_ = 0.10;
    double balloon_servo_hold_time_ = 0.25;
    double balloon_servo_lateral_gain_ = 0.18;
    double balloon_servo_z_gain_ = 0.12;
    double balloon_servo_max_lateral_speed_ = 0.10;
    double balloon_servo_max_z_speed_ = 0.08;
    double balloon_fine_forward_speed_ = 0.07;
    double balloon_fine_forward_distance_ = 0.20;
    double balloon_backoff_distance_ = 0.12;
    double balloon_pop_verify_timeout_ = 1.0;
    double balloon_pop_area_drop_ratio_ = 0.35;
    double balloon_pop_verify_roi_scale_ = 1.5;
    double balloon_pop_verify_roi_min_px_ = 120.0;
    int balloon_max_retry_ = 1;
    double balloon_return_home_land_z_ = 1.0;

    double override_move_timeout_ = 30.0;
    double override_pos_threshold_ = 0.16;
    double override_smooth_speed_ = 0.35;
    double override_yaw_timeout_ = 10.0;
    double override_yaw_threshold_ = 0.10;
    double override_yaw_hold_time_ = 0.25;
    double override_yaw_rate_ = 0.5;
    double attack_zone_overrun_x_ = 0.25;
    bool robust_land_enable_ = true;
    double robust_land_soft_z_ = 0.10;
    double robust_land_soft_speed_ = 0.08;
    double robust_land_z_threshold_ = 0.04;
    double robust_land_settle_time_ = 0.30;
    double robust_land_press_z_ = -0.05;
    double robust_land_disarm_timeout_ = 3.0;
    double robust_land_disarm_retry_period_ = 0.25;
    bool robust_land_fallback_bridge_land_ = true;
    std::string px4_land_mode_ = "AUTO.LAND";
    double px4_land_mode_timeout_ = 5.0;
    double px4_land_retry_period_ = 0.5;

    bool have_frame_center_ = false;
    Eigen::Vector3d frame_center_ = Eigen::Vector3d::Zero();
    ros::Time frame_center_stamp_;
    ros::Time frame_center_recv_stamp_;
    std::string frame_status_;
    ros::Time frame_status_stamp_;

    bool qr_detected_ = false;
    ros::Time qr_detect_stamp_;
    std::vector<int> qr_ids_;

    bool have_balloon_world_point_ = false;
    Eigen::Vector3d balloon_world_point_ = Eigen::Vector3d::Zero();
    ros::Time balloon_world_stamp_;
    bool have_balloon_servo_ = false;
    BalloonServo balloon_servo_;
    ros::Time balloon_servo_stamp_;
    bool have_verify_roi_result_ = false;
    VerifyRoiResult verify_roi_result_;
    ros::Time verify_roi_result_stamp_;

    bool have_mavros_state_ = false;
    bool mavros_armed_ = true;
    std::string mavros_mode_;
    ros::Time mavros_state_stamp_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "craic_competition_demo");
    CraicCompetitionDemo demo;
    return demo.run();
}
