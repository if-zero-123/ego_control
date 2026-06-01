#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

namespace {

double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double yawFromPose(const geometry_msgs::PoseStamped& pose) {
    const auto& q = pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Eigen::Vector3d posePosition(const geometry_msgs::PoseStamped& pose) {
    return Eigen::Vector3d(pose.pose.position.x,
                           pose.pose.position.y,
                           pose.pose.position.z);
}

bool validFrameStatus(const std::string& status) {
    return status == "valid_full" ||
           status == "valid_weak_pair" ||
           status == "valid_split_frame" ||
           status == "valid_partial_left" ||
           status == "valid_partial_right";
}

class FrameCrossDemo {
public:
    FrameCrossDemo() : nh_(), pnh_("~") {
        pnh_.param<std::string>("bridge_ns", bridge_ns_, "/ego_bridge");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
        pnh_.param<std::string>("control_method", control_method_, "planner");
        pnh_.param<std::string>("failure_action", failure_action_, "land");
        pnh_.param<std::string>("goal_frame_id", goal_frame_id_, "world");
        pnh_.param<std::string>("ego_use_goal_msg_z_param", ego_use_goal_msg_z_param_,
                                "/drone_0_ego_planner_node/fsm/use_goal_msg_z");
        pnh_.param<double>("takeoff_timeout", takeoff_timeout_, 35.0);
        pnh_.param<double>("detect_timeout", detect_timeout_, 20.0);
        pnh_.param<double>("pre_timeout", pre_timeout_, 30.0);
        pnh_.param<double>("post_timeout", post_timeout_, 35.0);
        pnh_.param<double>("command_publish_timeout", command_publish_timeout_, 4.0);
        pnh_.param<double>("reach_threshold", reach_threshold_, 0.20);
        pnh_.param<double>("max_goal_distance", max_goal_distance_, 5.0);
        pnh_.param<double>("min_goal_z", min_goal_z_, 0.35);
        pnh_.param<double>("max_goal_z", max_goal_z_, 1.8);
        pnh_.param<int>("valid_required_frames", valid_required_frames_, 5);
        pnh_.param<double>("valid_max_age", valid_max_age_, 0.6);
        pnh_.param<double>("hover_before_detect", hover_before_detect_, 1.0);
        pnh_.param<double>("hold_at_pre", hold_at_pre_, 1.0);
        pnh_.param<double>("hold_after_pass", hold_after_pass_, 5.0);
        pnh_.param<double>("land_timeout", land_timeout_, 35.0);
        pnh_.param<bool>("refresh_before_pass", refresh_before_pass_, true);
        pnh_.param<double>("z_kp", z_kp_, 0.9);
        pnh_.param<double>("max_vz", max_vz_, 0.15);

        odom_sub_ = nh_.subscribe(odom_topic_, 20, &FrameCrossDemo::odomCb, this);
        flight_state_sub_ = nh_.subscribe(bridge_ns_ + "/flight_state", 10,
                                          &FrameCrossDemo::flightStateCb, this);
        control_mode_sub_ = nh_.subscribe(bridge_ns_ + "/control_mode", 10,
                                          &FrameCrossDemo::controlModeCb, this);
        reach_status_sub_ = nh_.subscribe(bridge_ns_ + "/reach_status", 10,
                                          &FrameCrossDemo::reachStatusCb, this);
        frame_status_sub_ = nh_.subscribe("/craic_debug/frame_cloud_status", 10,
                                          &FrameCrossDemo::frameStatusCb, this);
        frame_pre_sub_ = nh_.subscribe("/craic_debug/frame_cloud_pre_goal", 10,
                                       &FrameCrossDemo::framePreCb, this);
        frame_post_sub_ = nh_.subscribe("/craic_debug/frame_cloud_post_goal", 10,
                                        &FrameCrossDemo::framePostCb, this);

        takeoff_land_pub_ = nh_.advertise<quadrotor_msgs::TakeoffLand>(
            bridge_ns_ + "/takeoff_land", 1);
        set_control_mode_pub_ = nh_.advertise<std_msgs::UInt8>(
            bridge_ns_ + "/set_control_mode", 1);
        override_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(
            bridge_ns_ + "/override_cmd", 10);
        planner_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            "/move_base_simple/goal", 1);
        target_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            bridge_ns_ + "/target_point", 1);

        ROS_INFO("[frame_cross_demo] bridge=%s odom=%s method=%s goal_frame=%s reach=%.2f valid_frames=%d safety_dist=%.2f z=[%.2f %.2f] refresh_before_pass=%d failure=%s",
                 bridge_ns_.c_str(), odom_topic_.c_str(), control_method_.c_str(),
                 goal_frame_id_.c_str(), reach_threshold_, valid_required_frames_,
                 max_goal_distance_, min_goal_z_, max_goal_z_, refresh_before_pass_,
                 failure_action_.c_str());
    }

    int run() {
        if (!waitReady()) return 1;

        ROS_INFO("[frame_cross_demo] ===== TAKEOFF =====");
        if (!sendTakeoffAndWait()) {
            handleFailure("takeoff_failed");
            return 1;
        }

        ROS_INFO("[frame_cross_demo] ===== HOVER_BEFORE_DETECT =====");
        holdCurrent(hover_before_detect_);

        ROS_INFO("[frame_cross_demo] ===== WAIT_FRAME =====");
        if (!waitForFrame()) {
            handleFailure("frame_detect_timeout");
            return 1;
        }

        geometry_msgs::PoseStamped pre_goal = normalizeWorldGoal(frame_pre_);
        geometry_msgs::PoseStamped post_goal = normalizeWorldGoal(frame_post_);
        if (!validateGoal(pre_goal, "pre") || !validateGoal(post_goal, "post")) {
            handleFailure("invalid_frame_goal");
            return 1;
        }
        logGoal("locked_pre", frame_pre_, pre_goal);
        logGoal("locked_post", frame_post_, post_goal);

        warnIfPlannerGoalZIgnored();

        bool ok = false;
        if (control_method_ == "override") {
            ok = runOverrideCross(pre_goal, post_goal);
        } else {
            ok = runPlannerCross(pre_goal, post_goal);
        }

        if (!ok) {
            handleFailure("cross_failed");
            return 2;
        }

        ROS_INFO("[frame_cross_demo] ===== HOLD_AFTER_PASS =====");
        holdCurrent(hold_after_pass_);
        ROS_INFO("[frame_cross_demo] ===== DONE =====");
        return 0;
    }

private:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_pos_ << msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     msg->pose.pose.position.z;
        odom_vel_ << msg->twist.twist.linear.x,
                     msg->twist.twist.linear.y,
                     msg->twist.twist.linear.z;
        const auto& q = msg->pose.pose.orientation;
        odom_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        odom_stamp_ = ros::Time::now();
        have_odom_ = true;
    }

    void flightStateCb(const std_msgs::String::ConstPtr& msg) {
        flight_state_ = msg->data;
        have_bridge_ = true;
    }

    void controlModeCb(const std_msgs::UInt8::ConstPtr& msg) {
        control_mode_ = msg->data;
    }

    void reachStatusCb(const std_msgs::UInt8::ConstPtr& msg) {
        reach_status_ = msg->data;
    }

    void frameStatusCb(const std_msgs::String::ConstPtr& msg) {
        frame_status_ = msg->data;
        frame_status_stamp_ = ros::Time::now();
    }

    void framePreCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        frame_pre_ = *msg;
        have_pre_ = true;
        frame_pre_stamp_ = ros::Time::now();
    }

    void framePostCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        frame_post_ = *msg;
        have_post_ = true;
        frame_post_stamp_ = ros::Time::now();
    }

    bool waitReady() {
        ros::Rate rate(20);
        ROS_INFO("[frame_cross_demo] WAIT_READY");
        while (ros::ok()) {
            ros::spinOnce();
            if (have_bridge_ && have_odom_) {
                ROS_INFO("[frame_cross_demo] Ready. state=%s odom=(%.2f %.2f %.2f)",
                         flight_state_.c_str(), odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
                return true;
            }
            ROS_INFO_THROTTLE(1.0, "[frame_cross_demo] waiting bridge=%d odom=%d",
                              have_bridge_, have_odom_);
            rate.sleep();
        }
        return false;
    }

    bool sendTakeoffAndWait() {
        if (flight_state_ == "HOVER" || flight_state_ == "TRACKING") {
            ROS_WARN("[frame_cross_demo] Already airborne state=%s, skip TAKEOFF.",
                     flight_state_.c_str());
            return true;
        }

        quadrotor_msgs::TakeoffLand msg;
        msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;

        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        ros::Time last_pub(0);
        bool command_seen = false;
        while (ros::ok()) {
            ros::spinOnce();
            if (flight_state_ == "HOVER") {
                ROS_INFO("[frame_cross_demo] Takeoff complete.");
                return true;
            }

            if ((flight_state_ == "IDLE" || flight_state_ == "UNKNOWN") &&
                (ros::Time::now() - last_pub).toSec() > 0.2) {
                last_pub = ros::Time::now();
                takeoff_land_pub_.publish(msg);
                ROS_INFO_THROTTLE(1.0, "[frame_cross_demo] Publishing TAKEOFF subscribers=%u state=%s",
                                  takeoff_land_pub_.getNumSubscribers(), flight_state_.c_str());
            }

            if (flight_state_ == "PRE_OFFBOARD" || flight_state_ == "TAKEOFF") {
                command_seen = true;
            }

            if (!command_seen && (ros::Time::now() - start).toSec() > command_publish_timeout_) {
                ROS_ERROR("[frame_cross_demo] TAKEOFF not accepted state=%s subscribers=%u",
                          flight_state_.c_str(), takeoff_land_pub_.getNumSubscribers());
                return false;
            }
            if ((ros::Time::now() - start).toSec() > takeoff_timeout_) {
                ROS_ERROR("[frame_cross_demo] Takeoff timeout state=%s", flight_state_.c_str());
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool waitForFrame() {
        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        int valid_count = 0;
        while (ros::ok()) {
            ros::spinOnce();
            if (freshFrame()) {
                ++valid_count;
                if (valid_count >= valid_required_frames_) {
                    ROS_INFO("[frame_cross_demo] Frame stable status=%s pre=(%.2f %.2f %.2f) post=(%.2f %.2f %.2f)",
                             frame_status_.c_str(),
                             frame_pre_.pose.position.x, frame_pre_.pose.position.y, frame_pre_.pose.position.z,
                             frame_post_.pose.position.x, frame_post_.pose.position.y, frame_post_.pose.position.z);
                    return true;
                }
            } else {
                valid_count = 0;
            }

            ROS_INFO_THROTTLE(0.8, "[frame_cross_demo] waiting frame status=%s have_pre=%d have_post=%d valid_count=%d",
                              frame_status_.c_str(), have_pre_, have_post_, valid_count);
            if ((ros::Time::now() - start).toSec() > detect_timeout_) {
                ROS_ERROR("[frame_cross_demo] Frame detect timeout status=%s have_pre=%d have_post=%d",
                          frame_status_.c_str(), have_pre_, have_post_);
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool freshFrame() const {
        const ros::Time now = ros::Time::now();
        return validFrameStatus(frame_status_) &&
               have_pre_ &&
               have_post_ &&
               (now - frame_status_stamp_).toSec() < valid_max_age_ &&
               (now - frame_pre_stamp_).toSec() < valid_max_age_ &&
               (now - frame_post_stamp_).toSec() < valid_max_age_;
    }

    geometry_msgs::PoseStamped normalizeWorldGoal(const geometry_msgs::PoseStamped& goal) const {
        geometry_msgs::PoseStamped out = goal;
        out.header.frame_id = goal_frame_id_;
        return out;
    }

    Eigen::Vector2d bodyRelativeXY(const Eigen::Vector3d& target) const {
        const double dx = target.x() - odom_pos_.x();
        const double dy = target.y() - odom_pos_.y();
        const double c = std::cos(odom_yaw_);
        const double s = std::sin(odom_yaw_);
        return Eigen::Vector2d(c * dx + s * dy, -s * dx + c * dy);
    }

    bool validateGoal(const geometry_msgs::PoseStamped& goal, const std::string& label) const {
        const Eigen::Vector3d target = posePosition(goal);
        const double dist = (target - odom_pos_).norm();
        if (!std::isfinite(target.x()) || !std::isfinite(target.y()) || !std::isfinite(target.z())) {
            ROS_ERROR("[frame_cross_demo] %s goal has non-finite position.", label.c_str());
            return false;
        }
        if (dist > max_goal_distance_) {
            ROS_ERROR("[frame_cross_demo] %s goal too far dist=%.2f max=%.2f target=(%.2f %.2f %.2f) odom=(%.2f %.2f %.2f)",
                      label.c_str(), dist, max_goal_distance_,
                      target.x(), target.y(), target.z(),
                      odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
            return false;
        }
        if (target.z() < min_goal_z_ || target.z() > max_goal_z_) {
            ROS_ERROR("[frame_cross_demo] %s goal z out of range z=%.2f allowed=[%.2f %.2f]",
                      label.c_str(), target.z(), min_goal_z_, max_goal_z_);
            return false;
        }
        return true;
    }

    void logGoal(const std::string& label,
                 const geometry_msgs::PoseStamped& raw,
                 const geometry_msgs::PoseStamped& world) const {
        const Eigen::Vector3d target = posePosition(world);
        const Eigen::Vector2d rel = bodyRelativeXY(target);
        ROS_INFO("[frame_cross_demo] %s raw_frame=%s world=(%.2f %.2f %.2f) body_rel=(%.2f %.2f %.2f) yaw=%.2f",
                 label.c_str(), raw.header.frame_id.c_str(),
                 target.x(), target.y(), target.z(),
                 rel.x(), rel.y(), target.z() - odom_pos_.z(),
                 yawFromPose(world));
    }

    void warnIfPlannerGoalZIgnored() const {
        if (control_method_ != "planner") return;

        bool use_goal_msg_z = false;
        if (ros::param::get(ego_use_goal_msg_z_param_, use_goal_msg_z)) {
            if (!use_goal_msg_z) {
                ROS_WARN("[frame_cross_demo] EGO parameter %s is false; planner may ignore frame goal z. Launch EGO with use_goal_msg_z:=true.",
                         ego_use_goal_msg_z_param_.c_str());
            }
        } else {
            ROS_WARN("[frame_cross_demo] Could not read %s. For auto frame height, launch EGO with use_goal_msg_z:=true.",
                     ego_use_goal_msg_z_param_.c_str());
        }
    }

    bool refreshFrameGoals(geometry_msgs::PoseStamped& pre_goal,
                           geometry_msgs::PoseStamped& post_goal,
                           double timeout,
                           const std::string& reason) {
        const ros::Time start = ros::Time::now();
        ros::Rate rate(20);
        int valid_count = 0;
        while (ros::ok()) {
            ros::spinOnce();
            if (freshFrame()) {
                ++valid_count;
                if (valid_count >= valid_required_frames_) {
                    pre_goal = normalizeWorldGoal(frame_pre_);
                    post_goal = normalizeWorldGoal(frame_post_);
                    if (!validateGoal(pre_goal, reason + "_pre") ||
                        !validateGoal(post_goal, reason + "_post")) {
                        return false;
                    }
                    logGoal(reason + "_pre", frame_pre_, pre_goal);
                    logGoal(reason + "_post", frame_post_, post_goal);
                    return true;
                }
            } else {
                valid_count = 0;
            }
            if ((ros::Time::now() - start).toSec() > timeout) {
                ROS_ERROR("[frame_cross_demo] refresh frame goals timeout reason=%s status=%s",
                          reason.c_str(), frame_status_.c_str());
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool runPlannerCross(geometry_msgs::PoseStamped pre_goal,
                         geometry_msgs::PoseStamped post_goal) {
        if (control_method_ != "planner" && control_method_ != "override") {
            ROS_WARN("[frame_cross_demo] Unknown control_method=%s, fallback to planner.",
                     control_method_.c_str());
        }

        if (control_mode_ == 1 && !disableOverride()) {
            ROS_ERROR("[frame_cross_demo] Cannot leave override before planner mode.");
            return false;
        }

        ROS_INFO("[frame_cross_demo] ===== GO_PRE planner =====");
        if (!sendPlannerGoalAndWait(pre_goal, pre_timeout_, "pre")) return false;

        ROS_INFO("[frame_cross_demo] ===== HOLD_AT_PRE planner =====");
        holdCurrent(hold_at_pre_);
        if (refresh_before_pass_ &&
            !refreshFrameGoals(pre_goal, post_goal,
                               std::min(5.0, detect_timeout_), "refresh_before_pass")) {
            return false;
        }

        ROS_INFO("[frame_cross_demo] ===== PASS_FRAME planner =====");
        if (!sendPlannerGoalAndWait(post_goal, post_timeout_, "post")) return false;
        return true;
    }

    bool runOverrideCross(geometry_msgs::PoseStamped pre_goal,
                          geometry_msgs::PoseStamped post_goal) {
        if (!enableOverride()) return false;

        ROS_INFO("[frame_cross_demo] ===== GO_PRE override =====");
        if (!moveToOverride(pre_goal, pre_timeout_, "pre")) {
            disableOverride();
            return false;
        }

        ROS_INFO("[frame_cross_demo] ===== HOLD_AT_PRE override =====");
        holdCurrent(hold_at_pre_);
        if (refresh_before_pass_ &&
            !refreshFrameGoals(pre_goal, post_goal,
                               std::min(5.0, detect_timeout_), "refresh_before_pass")) {
            disableOverride();
            return false;
        }

        ROS_INFO("[frame_cross_demo] ===== PASS_FRAME override =====");
        if (!moveToOverride(post_goal, post_timeout_, "post")) {
            disableOverride();
            return false;
        }

        return disableOverride();
    }

    bool sendPlannerGoalAndWait(const geometry_msgs::PoseStamped& goal,
                                double timeout,
                                const std::string& label) {
        ros::Rate rate(10);
        const ros::Time start = ros::Time::now();
        ros::Time last_pub(0);
        ros::Time first_pub_time(0);
        const Eigen::Vector3d target = posePosition(goal);
        const double yaw = yawFromPose(goal);

        while (ros::ok()) {
            ros::spinOnce();
            if ((ros::Time::now() - last_pub).toSec() > 0.5) {
                last_pub = ros::Time::now();
                geometry_msgs::PoseStamped out = goal;
                out.header.stamp = ros::Time::now();
                out.header.frame_id = goal_frame_id_;
                planner_goal_pub_.publish(out);
                target_point_pub_.publish(out);
                if (!first_pub_time.isValid()) {
                    first_pub_time = out.header.stamp;
                }
                ROS_INFO_THROTTLE(1.0,
                                  "[frame_cross_demo] planner %s goal=(%.2f %.2f %.2f) yaw=%.2f reach_status=%u",
                                  label.c_str(), target.x(), target.y(), target.z(), yaw, reach_status_);
            }

            const double dist = (odom_pos_ - target).norm();
            const bool reach_status_fresh_enough =
                first_pub_time.isValid() && (ros::Time::now() - first_pub_time).toSec() > 0.5;
            if ((reach_status_fresh_enough && reach_status_ == 1) || dist < reach_threshold_) {
                ROS_INFO("[frame_cross_demo] planner %s reached dist=%.3f reach_status=%u",
                         label.c_str(), dist, reach_status_);
                return true;
            }
            if ((ros::Time::now() - start).toSec() > timeout) {
                ROS_ERROR("[frame_cross_demo] planner %s timeout dist=%.3f reach_status=%u",
                          label.c_str(), dist, reach_status_);
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool enableOverride() {
        std_msgs::UInt8 msg;
        msg.data = 1;
        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        while (ros::ok()) {
            ros::spinOnce();
            set_control_mode_pub_.publish(msg);
            if (control_mode_ == 1) {
                ROS_INFO("[frame_cross_demo] Override enabled.");
                return true;
            }
            if ((ros::Time::now() - start).toSec() > 2.0) {
                ROS_ERROR("[frame_cross_demo] enableOverride timeout state=%s mode=%u",
                          flight_state_.c_str(), control_mode_);
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool disableOverride() {
        std_msgs::UInt8 msg;
        msg.data = 0;
        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        while (ros::ok()) {
            ros::spinOnce();
            set_control_mode_pub_.publish(msg);
            if (control_mode_ == 0) {
                ROS_INFO("[frame_cross_demo] Override disabled.");
                return true;
            }
            if ((ros::Time::now() - start).toSec() > 2.0) {
                ROS_WARN("[frame_cross_demo] disableOverride timeout.");
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool moveToOverride(const geometry_msgs::PoseStamped& goal,
                        double timeout,
                        const std::string& label) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        const Eigen::Vector3d target = posePosition(goal);
        const double yaw = yawFromPose(goal);
        while (ros::ok()) {
            ros::spinOnce();
            sendPositionCmd(target, yaw);
            const double dist = (odom_pos_ - target).norm();
            ROS_INFO_THROTTLE(0.5, "[frame_cross_demo] override %s dist=%.3f target=(%.2f %.2f %.2f)",
                              label.c_str(), dist, target.x(), target.y(), target.z());
            if (dist < reach_threshold_) {
                ROS_INFO("[frame_cross_demo] override %s reached dist=%.3f", label.c_str(), dist);
                return true;
            }
            if ((ros::Time::now() - start).toSec() > timeout) {
                ROS_ERROR("[frame_cross_demo] override %s timeout dist=%.3f", label.c_str(), dist);
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    void sendPositionCmd(const Eigen::Vector3d& target, double yaw) {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = goal_frame_id_;
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
        override_cmd_pub_.publish(cmd);
    }

    double zHoldSpeed() const {
        const double target_z = odom_pos_.z();
        const double z_err = target_z - odom_pos_.z();
        double vz = clampValue(z_kp_ * z_err, -max_vz_, max_vz_);
        if (std::abs(z_err) < 0.04) vz = 0.0;
        return vz;
    }

    void sendVelocityCmd(double vx, double vy, double vz, double yaw_rate) {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = goal_frame_id_;
        cmd.position.x = odom_pos_.x();
        cmd.position.y = odom_pos_.y();
        cmd.position.z = odom_pos_.z();
        cmd.velocity.x = vx;
        cmd.velocity.y = vy;
        cmd.velocity.z = vz;
        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;
        cmd.yaw = odom_yaw_;
        cmd.yaw_dot = yaw_rate;
        cmd.trajectory_id = 0;
        cmd.trajectory_flag = 0;
        override_cmd_pub_.publish(cmd);
    }

    void holdCurrent(double seconds) {
        if (seconds <= 0.0) return;
        if (control_mode_ != 1) {
            ros::Duration(seconds).sleep();
            return;
        }

        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < seconds) {
            ros::spinOnce();
            sendVelocityCmd(0.0, 0.0, zHoldSpeed(), 0.0);
            rate.sleep();
        }
    }

    void handleFailure(const std::string& reason) {
        ROS_ERROR("[frame_cross_demo] FAIL_LAND reason=%s action=%s", reason.c_str(), failure_action_.c_str());
        if (control_mode_ == 1) {
            disableOverride();
        }
        if (failure_action_ == "land") {
            safeLand();
        } else {
            holdCurrent(hold_after_pass_);
        }
    }

    void safeLand() {
        if (flight_state_ == "IDLE") {
            ROS_INFO("[frame_cross_demo] Already IDLE, skip LAND.");
            return;
        }

        quadrotor_msgs::TakeoffLand msg;
        msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;

        ros::Rate rate(10);
        const ros::Time start = ros::Time::now();
        ros::Time last_pub(0);
        while (ros::ok()) {
            ros::spinOnce();
            if (flight_state_ == "IDLE") {
                ROS_INFO("[frame_cross_demo] Landing complete.");
                return;
            }
            if ((flight_state_ == "HOVER" || flight_state_ == "TRACKING") &&
                (ros::Time::now() - last_pub).toSec() > 0.2) {
                last_pub = ros::Time::now();
                takeoff_land_pub_.publish(msg);
                ROS_INFO_THROTTLE(1.0, "[frame_cross_demo] Publishing LAND subscribers=%u state=%s",
                                  takeoff_land_pub_.getNumSubscribers(), flight_state_.c_str());
            }
            if ((ros::Time::now() - start).toSec() > land_timeout_) {
                ROS_WARN("[frame_cross_demo] Landing timeout state=%s", flight_state_.c_str());
                return;
            }
            rate.sleep();
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber flight_state_sub_;
    ros::Subscriber control_mode_sub_;
    ros::Subscriber reach_status_sub_;
    ros::Subscriber frame_status_sub_;
    ros::Subscriber frame_pre_sub_;
    ros::Subscriber frame_post_sub_;
    ros::Publisher takeoff_land_pub_;
    ros::Publisher set_control_mode_pub_;
    ros::Publisher override_cmd_pub_;
    ros::Publisher planner_goal_pub_;
    ros::Publisher target_point_pub_;

    std::string bridge_ns_;
    std::string odom_topic_;
    std::string control_method_ = "planner";
    std::string failure_action_ = "land";
    std::string goal_frame_id_ = "world";
    std::string ego_use_goal_msg_z_param_ = "/drone_0_ego_planner_node/fsm/use_goal_msg_z";
    double takeoff_timeout_ = 35.0;
    double detect_timeout_ = 20.0;
    double pre_timeout_ = 30.0;
    double post_timeout_ = 35.0;
    double command_publish_timeout_ = 4.0;
    double reach_threshold_ = 0.20;
    double max_goal_distance_ = 5.0;
    double min_goal_z_ = 0.35;
    double max_goal_z_ = 1.8;
    int valid_required_frames_ = 5;
    double valid_max_age_ = 0.6;
    double hover_before_detect_ = 1.0;
    double hold_at_pre_ = 1.0;
    double hold_after_pass_ = 5.0;
    double land_timeout_ = 35.0;
    bool refresh_before_pass_ = true;
    double z_kp_ = 0.9;
    double max_vz_ = 0.15;

    bool have_bridge_ = false;
    bool have_odom_ = false;
    bool have_pre_ = false;
    bool have_post_ = false;
    uint8_t control_mode_ = 0;
    uint8_t reach_status_ = 0;
    std::string flight_state_ = "UNKNOWN";
    std::string frame_status_ = "unknown";
    ros::Time odom_stamp_;
    ros::Time frame_status_stamp_;
    ros::Time frame_pre_stamp_;
    ros::Time frame_post_stamp_;
    Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d odom_vel_ = Eigen::Vector3d::Zero();
    double odom_yaw_ = 0.0;
    geometry_msgs::PoseStamped frame_pre_;
    geometry_msgs::PoseStamped frame_post_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "frame_cross_demo_node");
    FrameCrossDemo demo;
    return demo.run();
}
