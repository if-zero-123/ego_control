#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
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

enum class MoveResult {
    Reached,
    QrDetected,
    Timeout,
    RosStopped
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

        pnh_.param<double>("expected_frame_x", expected_frame_center_.x(), 2.7);
        pnh_.param<double>("expected_frame_y", expected_frame_center_.y(), -1.5);
        pnh_.param<double>("expected_frame_z", expected_frame_center_.z(), 1.3);
        pnh_.param<double>("frame_center_reject_distance", frame_center_reject_distance_, 0.7);
        pnh_.param<double>("frame_post_x_offset", frame_post_x_offset_, 1.0);
        pnh_.param<double>("frame_detect_timeout", frame_detect_timeout_, 20.0);
        pnh_.param<double>("frame_valid_max_age", frame_valid_max_age_, 0.8);

        pnh_.param<double>("qr_goal_x", qr_goal_.x(), 3.6);
        pnh_.param<double>("qr_goal_y", qr_goal_.y(), 0.25);
        pnh_.param<double>("qr_goal_z", qr_goal_.z(), 1.3);
        pnh_.param<double>("qr_initial_wait", qr_initial_wait_, 1.0);
        pnh_.param<double>("qr_search_timeout", qr_search_timeout_, 10.0);
        pnh_.param<double>("qr_search_raise_z", qr_search_raise_z_, 0.2);
        pnh_.param<double>("qr_search_offset", qr_search_offset_, 0.3);
        pnh_.param<double>("qr_search_hold", qr_search_hold_, 0.35);

        pnh_.param<double>("attack_zone_x", attack_zone_.x(), 0.2);
        pnh_.param<double>("attack_zone_y", attack_zone_.y(), -0.9);
        pnh_.param<double>("attack_zone_z", attack_zone_.z(), 1.0);
        pnh_.param<double>("attack_height", attack_height_, 0.3);
        pnh_.param<double>("balloon_wait_timeout", balloon_wait_timeout_, 8.0);
        pnh_.param<double>("balloon_valid_max_age", balloon_valid_max_age_, 0.8);
        pnh_.param<double>("balloon_standoff", balloon_standoff_, 0.35);
        pnh_.param<double>("balloon_puncture_distance", balloon_puncture_distance_, 0.28);
        pnh_.param<double>("balloon_puncture_speed", balloon_puncture_speed_, 0.10);
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

        pnh_.param<double>("override_move_timeout", override_move_timeout_, 30.0);
        pnh_.param<double>("override_pos_threshold", override_pos_threshold_, 0.16);
        pnh_.param<double>("override_yaw_timeout", override_yaw_timeout_, 5.0);
        pnh_.param<double>("override_yaw_threshold", override_yaw_threshold_, 0.10);
        pnh_.param<double>("override_yaw_hold_time", override_yaw_hold_time_, 0.25);

        pnh_.param<std::string>("frame_center_topic", frame_center_topic_, "/craic/frame_center");
        pnh_.param<std::string>("frame_status_topic", frame_status_topic_, "/craic/frame_status");
        pnh_.param<std::string>("qr_ids_topic", qr_ids_topic_, "/usb_camera_vision/aruco_ids");
        pnh_.param<std::string>("balloon_world_topic", balloon_world_topic_, "/balloon/world_point");

        frame_center_sub_ = nh_.subscribe(frame_center_topic_, 1, &CraicCompetitionDemo::frameCenterCb, this);
        frame_status_sub_ = nh_.subscribe(frame_status_topic_, 1, &CraicCompetitionDemo::frameStatusCb, this);
        qr_ids_sub_ = nh_.subscribe(qr_ids_topic_, 1, &CraicCompetitionDemo::qrIdsCb, this);
        balloon_world_sub_ = nh_.subscribe(balloon_world_topic_, 1, &CraicCompetitionDemo::balloonWorldCb, this);
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

        logStage("FRAME_POST_EGO", "ego goal to frame post control");
        if (!flyGoal("frame_post_control", frame_post_control, initial_yaw)) {
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

        logStage("ATTACK_ZONE_EGO", "ego goal back to attack zone");
        if (!flyGoal("attack_zone", attack_zone_, reverse_yaw)) {
            ROS_ERROR("[craic_demo] FAIL stage=ATTACK_ZONE_EGO");
            safeLand();
            return 1;
        }

        logStage("LOWER_ATTACK_HEIGHT_OVERRIDE", "descend to attack height");
        Eigen::Vector3d low = api_.getOdomPosition();
        low.z() = attack_height_;
        if (!overrideMoveTo("attack_height", low, reverse_yaw,
                            override_pos_threshold_, override_move_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=LOWER_ATTACK_HEIGHT_OVERRIDE");
            safeLand();
            return 1;
        }

        logStage("ATTACK_YAW_OVERRIDE", "turn right 90deg from initial yaw");
        if (!overrideYawAlign("attack_yaw", attack_yaw, override_yaw_timeout_)) {
            ROS_ERROR("[craic_demo] FAIL stage=ATTACK_YAW_OVERRIDE");
            safeLand();
            return 1;
        }

        logStage("BALLOON_ATTACK_OVERRIDE", "world balloon point approach and puncture");
        if (!attackBalloon(attack_yaw)) {
            ROS_WARN("[craic_demo] BALLOON_ATTACK result=not_completed action=no_blind_puncture");
        }

        logStage("LAND", land_after_finish_ ? "ego_bridge land" : "hold and finish");
        if (land_after_finish_) {
            if (!api_.land(30.0)) {
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

    bool overrideMoveTo(const std::string& label,
                        const Eigen::Vector3d& target,
                        double yaw,
                        double threshold,
                        double timeout) {
        if (!finiteVec(target)) {
            ROS_ERROR("[craic_demo] OVERRIDE_MOVE invalid label=%s reason=non_finite", label.c_str());
            return false;
        }
        ROS_INFO("[craic_demo] OVERRIDE_MOVE start name=%s target=(%.2f %.2f %.2f) yaw=%.3f threshold=%.2f timeout=%.1f",
                 label.c_str(), target.x(), target.y(), target.z(), yaw, threshold, timeout);
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
        while (ros::ok()) {
            ros::spinOnce();
            if (stop_on_qr && qrDetected()) {
                ROS_INFO("[craic_demo] OVERRIDE_MOVE interrupted_by=qr name=%s", label.c_str());
                return MoveResult::QrDetected;
            }

            sendPositionCmd(target, yaw);
            const double dist = (api_.getOdomPosition() - target).norm();
            if (dist <= threshold) {
                ROS_INFO("[craic_demo] OVERRIDE_MOVE reached name=%s dist=%.3f", label.c_str(), dist);
                return MoveResult::Reached;
            }

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] OVERRIDE_MOVE name=%s target=(%.2f %.2f %.2f) dist=%.3f yaw_err=%.3f",
                                  label.c_str(), target.x(), target.y(), target.z(), dist,
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
        ROS_INFO("[craic_demo] YAW_ALIGN start name=%s target_yaw=%.3f current_yaw=%.3f",
                 label.c_str(), target_yaw, api_.getOdomYaw());
        if (!api_.enableOverride()) {
            ROS_ERROR("[craic_demo] YAW_ALIGN failed name=%s reason=enable_override", label.c_str());
            return false;
        }

        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time stable_since;
        bool stable = false;
        bool reached = false;
        while (ros::ok()) {
            ros::spinOnce();
            sendPositionCmd(api_.getOdomPosition(), target_yaw);
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
                                  "[craic_demo] YAW_ALIGN name=%s err=%.3f target=%.3f current=%.3f",
                                  label.c_str(), err, target_yaw, api_.getOdomYaw());
            }
            if ((ros::Time::now() - start).toSec() >= timeout) {
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
        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < frame_detect_timeout_) {
            ros::spinOnce();
            if (freshFrameCenter()) {
                const double dist = (frame_center_ - expected_frame_center_).norm();
                if (dist <= frame_center_reject_distance_) {
                    ROS_INFO("[craic_demo] FRAME_CENTER source=detected status=%s center=(%.2f %.2f %.2f) expected=(%.2f %.2f %.2f) dist=%.2f",
                             frame_status_.c_str(),
                             frame_center_.x(), frame_center_.y(), frame_center_.z(),
                             expected_frame_center_.x(), expected_frame_center_.y(), expected_frame_center_.z(),
                             dist);
                    return frame_center_;
                }
                ROS_WARN("[craic_demo] FRAME_CENTER reject_detected status=%s center=(%.2f %.2f %.2f) expected=(%.2f %.2f %.2f) dist=%.2f limit=%.2f",
                         frame_status_.c_str(),
                         frame_center_.x(), frame_center_.y(), frame_center_.z(),
                         expected_frame_center_.x(), expected_frame_center_.y(), expected_frame_center_.z(),
                         dist, frame_center_reject_distance_);
                return expected_frame_center_;
            }
            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] FRAME_CENTER waiting status=%s have_center=%s",
                                  frame_status_.c_str(), yesNo(have_frame_center_));
            }
            rate.sleep();
        }
        ROS_WARN("[craic_demo] FRAME_CENTER fallback reason=timeout_or_invalid expected=(%.2f %.2f %.2f)",
                 expected_frame_center_.x(), expected_frame_center_.y(), expected_frame_center_.z());
        return expected_frame_center_;
    }

    bool freshFrameCenter() const {
        return have_frame_center_ &&
               validFrameStatus(frame_status_) &&
               frame_center_stamp_.isValid() &&
               (ros::Time::now() - frame_center_stamp_).toSec() <= frame_valid_max_age_ &&
               finiteVec(frame_center_);
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
        Eigen::Vector3d balloon;
        if (!waitBalloonWorldPoint(balloon)) {
            holdOverride(0.5, attack_yaw);
            return false;
        }

        const Eigen::Vector2d forward(std::cos(attack_yaw), std::sin(attack_yaw));
        const Eigen::Vector2d left(-std::sin(attack_yaw), std::cos(attack_yaw));
        Eigen::Vector3d approach = balloonApproachTarget(balloon, forward);
        ROS_INFO("[craic_demo] BALLOON_APPROACH target=(%.2f %.2f %.2f) balloon=(%.2f %.2f %.2f) standoff=%.2f yaw=%.3f",
                 approach.x(), approach.y(), approach.z(),
                 balloon.x(), balloon.y(), balloon.z(),
                 balloon_standoff_, attack_yaw);

        if (!api_.enableOverride()) {
            ROS_ERROR("[craic_demo] BALLOON_ATTACK failed reason=enable_override");
            return false;
        }

        MoveResult approach_result = moveOverrideLoop("balloon_safe_standoff", approach, attack_yaw,
                                                      override_pos_threshold_,
                                                      ros::Time::now() + ros::Duration(balloon_approach_timeout_),
                                                      false);
        if (approach_result != MoveResult::Reached) {
            holdOverride(0.25, attack_yaw);
            api_.disableOverride();
            return false;
        }

        ros::Rate align_rate(50);
        const ros::Time align_start = ros::Time::now();
        ros::Time stable_since;
        bool stable = false;
        bool aligned = false;
        while (ros::ok() && (ros::Time::now() - align_start).toSec() < balloon_align_timeout_) {
            ros::spinOnce();
            if (!freshBalloonWorldPoint()) {
                stable = false;
                sendPositionCmd(api_.getOdomPosition(), attack_yaw);
                ROS_WARN_THROTTLE(mission_log_period_,
                                  "[craic_demo] BALLOON_ALIGN waiting_for_fresh_world_point");
                align_rate.sleep();
                continue;
            }

            balloon = balloon_world_point_;
            approach = balloonApproachTarget(balloon, forward);
            sendPositionCmd(approach, attack_yaw);

            const Eigen::Vector3d pos = api_.getOdomPosition();
            const Eigen::Vector2d to_balloon(balloon.x() - pos.x(), balloon.y() - pos.y());
            const double forward_gap = to_balloon.dot(forward);
            const double forward_err = forward_gap - balloon_standoff_;
            const double lateral_err = to_balloon.dot(left);
            const double z_err = approach.z() - pos.z();
            const bool centered =
                std::abs(lateral_err) <= balloon_align_lateral_threshold_ &&
                std::abs(forward_err) <= balloon_align_forward_threshold_ &&
                std::abs(z_err) <= balloon_align_z_threshold_;

            if (centered) {
                if (!stable) {
                    stable = true;
                    stable_since = ros::Time::now();
                } else if ((ros::Time::now() - stable_since).toSec() >= balloon_align_hold_time_) {
                    aligned = true;
                    ROS_INFO("[craic_demo] BALLOON_ALIGN centered lateral_err=%.3f forward_err=%.3f z_err=%.3f gap=%.3f",
                             lateral_err, forward_err, z_err, forward_gap);
                    break;
                }
            } else {
                stable = false;
            }

            if (!simple_logs_) {
                ROS_INFO_THROTTLE(mission_log_period_,
                                  "[craic_demo] BALLOON_ALIGN target=(%.2f %.2f %.2f) balloon=(%.2f %.2f %.2f) lateral_err=%.3f forward_err=%.3f z_err=%.3f gap=%.3f centered=%s",
                                  approach.x(), approach.y(), approach.z(),
                                  balloon.x(), balloon.y(), balloon.z(),
                                  lateral_err, forward_err, z_err, forward_gap,
                                  yesNo(centered));
            }
            align_rate.sleep();
        }

        if (!aligned) {
            ROS_WARN("[craic_demo] BALLOON_ALIGN result=timeout action=no_puncture");
            holdOverride(0.25, attack_yaw);
            api_.disableOverride();
            return false;
        }

        const double needle_based_distance =
            balloon_standoff_ - balloon_needle_length_ + balloon_puncture_extra_;
        const double puncture_distance = clampValue(std::min(balloon_puncture_distance_,
                                                             needle_based_distance),
                                                    0.05,
                                                    balloon_puncture_distance_);
        const Eigen::Vector3d start = api_.getOdomPosition();
        const ros::Time deadline = ros::Time::now() + ros::Duration(
            std::max(2.0, puncture_distance / std::max(0.05, balloon_puncture_speed_) + 1.0));
        ros::Rate rate(50);
        bool done = false;
        while (ros::ok() && ros::Time::now() < deadline) {
            ros::spinOnce();
            sendVelocityCmdWithYaw(balloon_puncture_speed_ * forward.x(),
                                   balloon_puncture_speed_ * forward.y(),
                                   0.0,
                                   attack_yaw);
            const Eigen::Vector3d now = api_.getOdomPosition();
            const Eigen::Vector2d delta(now.x() - start.x(), now.y() - start.y());
            const double progress = delta.dot(forward);
            if (progress >= puncture_distance) {
                done = true;
                break;
            }
            rate.sleep();
        }
        holdOverride(0.3, attack_yaw);
        const bool disabled = api_.disableOverride();
        ROS_INFO("[craic_demo] BALLOON_PUNCTURE result=%s disabled=%s distance=%.2f configured=%.2f needle=%.2f extra=%.2f speed=%.2f",
                 yesNo(done), yesNo(disabled), puncture_distance, balloon_puncture_distance_,
                 balloon_needle_length_, balloon_puncture_extra_, balloon_puncture_speed_);
        return done && disabled;
    }

    Eigen::Vector3d balloonApproachTarget(const Eigen::Vector3d& balloon,
                                          const Eigen::Vector2d& forward) const {
        const double z = clampValue(balloon.z() + balloon_center_z_bias_,
                                    balloon_align_z_min_,
                                    balloon_align_z_max_);
        return Eigen::Vector3d(balloon.x() - balloon_standoff_ * forward.x(),
                               balloon.y() - balloon_standoff_ * forward.y(),
                               z);
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
        api_.disableOverride();
        if (land_after_finish_) {
            api_.land(20.0);
        }
    }

    void frameCenterCb(const geometry_msgs::PointStamped::ConstPtr& msg) {
        frame_center_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
        frame_center_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        have_frame_center_ = true;
    }

    void frameStatusCb(const std_msgs::String::ConstPtr& msg) {
        frame_status_ = msg->data;
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

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    EgoApi api_;

    ros::Subscriber frame_center_sub_;
    ros::Subscriber frame_status_sub_;
    ros::Subscriber qr_ids_sub_;
    ros::Subscriber balloon_world_sub_;

    std::string frame_center_topic_;
    std::string frame_status_topic_;
    std::string qr_ids_topic_;
    std::string balloon_world_topic_;

    double takeoff_timeout_ = 30.0;
    double goal_timeout_ = 60.0;
    double mission_log_period_ = 1.0;
    bool simple_logs_ = true;
    bool land_after_finish_ = true;

    Eigen::Vector3d initial_wait_ = Eigen::Vector3d(0.0, -0.85, 1.0);
    Eigen::Vector3d expected_frame_center_ = Eigen::Vector3d(2.7, -1.5, 1.3);
    double frame_center_reject_distance_ = 0.7;
    double frame_post_x_offset_ = 1.0;
    double frame_detect_timeout_ = 20.0;
    double frame_valid_max_age_ = 0.8;

    Eigen::Vector3d qr_goal_ = Eigen::Vector3d(3.6, 0.25, 1.3);
    double qr_initial_wait_ = 1.0;
    double qr_search_timeout_ = 10.0;
    double qr_search_raise_z_ = 0.2;
    double qr_search_offset_ = 0.3;
    double qr_search_hold_ = 0.35;

    Eigen::Vector3d attack_zone_ = Eigen::Vector3d(0.2, -0.9, 1.0);
    double attack_height_ = 0.3;
    double balloon_wait_timeout_ = 8.0;
    double balloon_valid_max_age_ = 0.8;
    double balloon_standoff_ = 0.35;
    double balloon_puncture_distance_ = 0.28;
    double balloon_puncture_speed_ = 0.10;
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

    double override_move_timeout_ = 30.0;
    double override_pos_threshold_ = 0.16;
    double override_yaw_timeout_ = 5.0;
    double override_yaw_threshold_ = 0.10;
    double override_yaw_hold_time_ = 0.25;

    bool have_frame_center_ = false;
    Eigen::Vector3d frame_center_ = Eigen::Vector3d::Zero();
    ros::Time frame_center_stamp_;
    std::string frame_status_;

    bool qr_detected_ = false;
    ros::Time qr_detect_stamp_;
    std::vector<int> qr_ids_;

    bool have_balloon_world_point_ = false;
    Eigen::Vector3d balloon_world_point_ = Eigen::Vector3d::Zero();
    ros::Time balloon_world_stamp_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "craic_competition_demo");
    CraicCompetitionDemo demo;
    return demo.run();
}
