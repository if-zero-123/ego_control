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

Eigen::Vector3d posePosition(const geometry_msgs::PoseStamped& pose) {
    return Eigen::Vector3d(pose.pose.position.x,
                           pose.pose.position.y,
                           pose.pose.position.z);
}

double yawFromPose(const geometry_msgs::PoseStamped& pose) {
    const auto& q = pose.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

class TakeoffAlignLandDemo {
public:
    TakeoffAlignLandDemo() : nh_(), pnh_("~") {
        pnh_.param<std::string>("bridge_ns", bridge_ns_, "/ego_bridge");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
        pnh_.param<double>("flight_z", flight_z_, 0.9);
        pnh_.param<double>("takeoff_timeout", takeoff_timeout_, 35.0);
        pnh_.param<double>("detect_timeout", detect_timeout_, 20.0);
        pnh_.param<double>("align_timeout", align_timeout_, 25.0);
        pnh_.param<double>("hold_after_align", hold_after_align_, 3.0);
        pnh_.param<double>("land_timeout", land_timeout_, 35.0);
        pnh_.param<double>("command_publish_timeout", command_publish_timeout_, 4.0);
        pnh_.param<double>("align_threshold", align_threshold_, 0.12);
        pnh_.param<double>("stable_hold_time", stable_hold_time_, 0.8);
        pnh_.param<double>("max_lateral_speed", max_lateral_speed_, 0.10);
        pnh_.param<double>("lateral_kp", lateral_kp_, 0.65);
        pnh_.param<double>("z_kp", z_kp_, 0.9);
        pnh_.param<double>("max_vz", max_vz_, 0.12);
        pnh_.param<double>("yaw_kp", yaw_kp_, 0.6);
        pnh_.param<double>("max_yaw_rate", max_yaw_rate_, 0.25);
        pnh_.param<double>("yaw_threshold", yaw_threshold_, 0.20);
        pnh_.param<double>("valid_max_age", valid_max_age_, 0.5);

        odom_sub_ = nh_.subscribe(odom_topic_, 10, &TakeoffAlignLandDemo::odomCb, this);
        flight_state_sub_ = nh_.subscribe(bridge_ns_ + "/flight_state", 10,
                                          &TakeoffAlignLandDemo::flightStateCb, this);
        control_mode_sub_ = nh_.subscribe(bridge_ns_ + "/control_mode", 10,
                                          &TakeoffAlignLandDemo::controlModeCb, this);
        pillar_status_sub_ = nh_.subscribe("/craic_debug/pillar_status", 10,
                                           &TakeoffAlignLandDemo::pillarStatusCb, this);
        pillar_pre_sub_ = nh_.subscribe("/craic_debug/pillar_pre_goal", 10,
                                        &TakeoffAlignLandDemo::pillarPreCb, this);
        pillar_post_sub_ = nh_.subscribe("/craic_debug/pillar_post_goal", 10,
                                         &TakeoffAlignLandDemo::pillarPostCb, this);

        takeoff_land_pub_ = nh_.advertise<quadrotor_msgs::TakeoffLand>(
            bridge_ns_ + "/takeoff_land", 1);
        set_control_mode_pub_ = nh_.advertise<std_msgs::UInt8>(
            bridge_ns_ + "/set_control_mode", 1);
        override_cmd_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>(
            bridge_ns_ + "/override_cmd", 10);

        ROS_INFO("[align_demo] bridge=%s odom=%s flight_z=%.2f align_thresh=%.2f max_v=%.2f",
                 bridge_ns_.c_str(), odom_topic_.c_str(), flight_z_, align_threshold_, max_lateral_speed_);
    }

    int run() {
        if (!waitForBridgeAndOdom()) return 1;

        ROS_INFO("[align_demo] ===== TAKEOFF =====");
        if (!sendTakeoffAndWait()) return 1;

        ROS_INFO("[align_demo] ===== WAIT_PILLARS =====");
        if (!waitForPillars()) {
            ROS_ERROR("[align_demo] No stable pillar detection. Landing.");
            safeLand();
            return 1;
        }

        ROS_INFO("[align_demo] ===== ALIGN_TO_CENTER_LINE =====");
        if (!enableOverride()) {
            ROS_ERROR("[align_demo] Cannot enter override. Landing.");
            safeLand();
            return 1;
        }

        const bool aligned = alignToPillarCenterLine();
        holdOverride(hold_after_align_);
    disableOverride();

        ROS_INFO("[align_demo] ===== LAND =====");
        safeLand();

        ROS_INFO("[align_demo] ===== DONE aligned=%d =====", aligned);
        return aligned ? 0 : 2;
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

    void pillarStatusCb(const std_msgs::String::ConstPtr& msg) {
        pillar_status_ = msg->data;
        pillar_status_stamp_ = ros::Time::now();
    }

    void pillarPreCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pillar_pre_ = *msg;
        pillar_pre_.pose.position.z = flight_z_;
        have_pre_ = true;
        pillar_goal_stamp_ = ros::Time::now();
    }

    void pillarPostCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pillar_post_ = *msg;
        pillar_post_.pose.position.z = flight_z_;
        have_post_ = true;
        pillar_goal_stamp_ = ros::Time::now();
    }

    bool waitForBridgeAndOdom() {
        ros::Rate rate(20);
        ROS_INFO("[align_demo] Waiting for bridge state and odom...");
        while (ros::ok()) {
            ros::spinOnce();
            if (have_bridge_ && have_odom_) {
                ROS_INFO("[align_demo] Ready. state=%s odom=(%.2f %.2f %.2f)",
                         flight_state_.c_str(), odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
                return true;
            }
            rate.sleep();
        }
        return false;
    }

    bool sendTakeoffAndWait() {
        if (flight_state_ == "HOVER") {
            ROS_WARN("[align_demo] Already in HOVER, skip TAKEOFF command.");
            return true;
        }

        quadrotor_msgs::TakeoffLand msg;
        msg.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;

        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        ros::Time last_pub(0);
        bool command_seen_by_bridge = false;
        while (ros::ok()) {
            ros::spinOnce();
            if (flight_state_ == "HOVER") {
                ROS_INFO("[align_demo] Takeoff complete. odom=(%.2f %.2f %.2f)",
                         odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
                return true;
            }

            // Publish repeatedly until ego_bridge leaves IDLE. A single publish can be
            // lost if the ROS publisher/subscriber connection is still settling.
            if ((flight_state_ == "IDLE" || flight_state_ == "UNKNOWN") &&
                (ros::Time::now() - last_pub).toSec() > 0.2) {
                last_pub = ros::Time::now();
                takeoff_land_pub_.publish(msg);
                ROS_INFO_THROTTLE(1.0,
                                  "[align_demo] Publishing TAKEOFF, subscribers=%u state=%s",
                                  takeoff_land_pub_.getNumSubscribers(),
                                  flight_state_.c_str());
            }

            if (flight_state_ == "PRE_OFFBOARD" || flight_state_ == "TAKEOFF") {
                command_seen_by_bridge = true;
            }

            if (!command_seen_by_bridge &&
                (ros::Time::now() - start).toSec() > command_publish_timeout_) {
                ROS_ERROR("[align_demo] TAKEOFF not accepted after %.1fs. state=%s subscribers=%u",
                          command_publish_timeout_, flight_state_.c_str(),
                          takeoff_land_pub_.getNumSubscribers());
                return false;
            }

            if ((ros::Time::now() - start).toSec() > takeoff_timeout_) {
                ROS_ERROR("[align_demo] Takeoff timeout. state=%s", flight_state_.c_str());
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool waitForPillars() {
        ros::Rate rate(20);
        const ros::Time start = ros::Time::now();
        int valid_count = 0;
        while (ros::ok()) {
            ros::spinOnce();
            if (freshPillars()) {
                ++valid_count;
                if (valid_count >= 5) {
                    ROS_INFO("[align_demo] Pillars stable. pre=(%.2f %.2f %.2f) post=(%.2f %.2f %.2f)",
                             pillar_pre_.pose.position.x, pillar_pre_.pose.position.y, pillar_pre_.pose.position.z,
                             pillar_post_.pose.position.x, pillar_post_.pose.position.y, pillar_post_.pose.position.z);
                    return true;
                }
            } else {
                valid_count = 0;
            }

            if ((ros::Time::now() - start).toSec() > detect_timeout_) {
                ROS_ERROR("[align_demo] Detect timeout. status=%s have_pre=%d have_post=%d",
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
               (now - pillar_status_stamp_).toSec() < valid_max_age_ &&
               (now - pillar_goal_stamp_).toSec() < valid_max_age_;
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
                ROS_INFO("[align_demo] Override enabled.");
                return true;
            }
            if ((ros::Time::now() - start).toSec() > 2.0) {
                ROS_ERROR("[align_demo] enableOverride timeout. state=%s mode=%d",
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
                ROS_INFO("[align_demo] Override disabled.");
                return true;
            }
            if ((ros::Time::now() - start).toSec() > 2.0) {
                ROS_WARN("[align_demo] disableOverride timeout.");
                return false;
            }
            rate.sleep();
        }
        return false;
    }

    bool alignToPillarCenterLine() {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time stable_since;
        bool stable_started = false;
        bool reached = false;

        while (ros::ok()) {
            ros::spinOnce();
            if (!freshPillars()) {
                sendVelocityCmd(0.0, 0.0, zHoldSpeed(), 0.0);
                stable_started = false;
                ROS_WARN_THROTTLE(0.5, "[align_demo] Pillar detection stale while aligning. status=%s",
                                  pillar_status_.c_str());
            } else {
                const Eigen::Vector3d pre = posePosition(pillar_pre_);
                const Eigen::Vector3d post = posePosition(pillar_post_);
                Eigen::Vector2d forward(post.x() - pre.x(), post.y() - pre.y());
                if (forward.norm() < 1e-3) {
                    ROS_ERROR("[align_demo] Invalid pre/post direction.");
                    break;
                }
                forward.normalize();

                const Eigen::Vector2d lateral(-forward.y(), forward.x());
                const Eigen::Vector2d corridor_xy(0.5 * (pre.x() + post.x()),
                                                  0.5 * (pre.y() + post.y()));
                const Eigen::Vector2d pos_xy(odom_pos_.x(), odom_pos_.y());
                const Eigen::Vector2d to_center = corridor_xy - pos_xy;
                const double lateral_err = to_center.dot(lateral);
                const double along_err = to_center.dot(forward);
                const double yaw_target = yawFromPose(pillar_pre_);
                const double yaw_err = normalizeYaw(yaw_target - odom_yaw_);

                double v_lat = clampValue(lateral_kp_ * lateral_err,
                                          -max_lateral_speed_, max_lateral_speed_);
                if (std::abs(lateral_err) < align_threshold_) v_lat = 0.0;

                double yaw_rate = clampValue(yaw_kp_ * yaw_err, -max_yaw_rate_, max_yaw_rate_);
                if (std::abs(yaw_err) < yaw_threshold_) yaw_rate = 0.0;

                sendVelocityCmd(v_lat * lateral.x(),
                                v_lat * lateral.y(),
                                zHoldSpeed(),
                                yaw_rate);

                const bool centered = std::abs(lateral_err) < align_threshold_ &&
                                      std::abs(flight_z_ - odom_pos_.z()) < 0.12;
                if (centered) {
                    if (!stable_started) {
                        stable_started = true;
                        stable_since = ros::Time::now();
                    } else if ((ros::Time::now() - stable_since).toSec() > stable_hold_time_) {
                        reached = true;
                        break;
                    }
                } else {
                    stable_started = false;
                }

                ROS_INFO_THROTTLE(0.4,
                                  "[align_demo] pos=(%.2f %.2f %.2f) lat_err=%.3f along_err=%.3f z_err=%.3f yaw_err=%.3f v_lat=%.3f",
                                  odom_pos_.x(), odom_pos_.y(), odom_pos_.z(),
                                  lateral_err, along_err, flight_z_ - odom_pos_.z(), yaw_err, v_lat);
            }

            if ((ros::Time::now() - start).toSec() > align_timeout_) {
                ROS_WARN("[align_demo] Align timeout.");
                break;
            }
            rate.sleep();
        }

        ROS_INFO("[align_demo] Align result reached=%d odom=(%.2f %.2f %.2f)",
                 reached, odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
        return reached;
    }

    double zHoldSpeed() const {
        const double z_err = flight_z_ - odom_pos_.z();
        double vz = clampValue(z_kp_ * z_err, -max_vz_, max_vz_);
        if (std::abs(z_err) < 0.04) vz = 0.0;
        return vz;
    }

    void sendVelocityCmd(double vx, double vy, double vz, double yaw_rate) {
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
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

    void holdOverride(double seconds) {
        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start).toSec() < seconds) {
            ros::spinOnce();
            sendVelocityCmd(0.0, 0.0, zHoldSpeed(), 0.0);
            rate.sleep();
        }
    }

    void safeLand() {
        if (flight_state_ == "IDLE") {
            ROS_INFO("[align_demo] Already IDLE, skip LAND command.");
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
                ROS_INFO("[align_demo] Landing complete.");
                return;
            }

            if ((flight_state_ == "HOVER" || flight_state_ == "TRACKING") &&
                (ros::Time::now() - last_pub).toSec() > 0.2) {
                last_pub = ros::Time::now();
                takeoff_land_pub_.publish(msg);
                ROS_INFO_THROTTLE(1.0,
                                  "[align_demo] Publishing LAND, subscribers=%u state=%s",
                                  takeoff_land_pub_.getNumSubscribers(),
                                  flight_state_.c_str());
            }

            if ((ros::Time::now() - start).toSec() > land_timeout_) {
                ROS_WARN("[align_demo] Landing timeout. state=%s", flight_state_.c_str());
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
    ros::Subscriber pillar_status_sub_;
    ros::Subscriber pillar_pre_sub_;
    ros::Subscriber pillar_post_sub_;
    ros::Publisher takeoff_land_pub_;
    ros::Publisher set_control_mode_pub_;
    ros::Publisher override_cmd_pub_;

    std::string bridge_ns_;
    std::string odom_topic_;
    double flight_z_ = 0.9;
    double takeoff_timeout_ = 35.0;
    double detect_timeout_ = 20.0;
    double align_timeout_ = 25.0;
    double hold_after_align_ = 3.0;
    double land_timeout_ = 35.0;
    double command_publish_timeout_ = 4.0;
    double align_threshold_ = 0.12;
    double stable_hold_time_ = 0.8;
    double max_lateral_speed_ = 0.10;
    double lateral_kp_ = 0.65;
    double z_kp_ = 0.9;
    double max_vz_ = 0.12;
    double yaw_kp_ = 0.6;
    double max_yaw_rate_ = 0.25;
    double yaw_threshold_ = 0.20;
    double valid_max_age_ = 0.5;

    bool have_bridge_ = false;
    bool have_odom_ = false;
    bool have_pre_ = false;
    bool have_post_ = false;
    uint8_t control_mode_ = 0;
    std::string flight_state_ = "UNKNOWN";
    std::string pillar_status_ = "unknown";
    ros::Time odom_stamp_;
    ros::Time pillar_status_stamp_;
    ros::Time pillar_goal_stamp_;
    Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d odom_vel_ = Eigen::Vector3d::Zero();
    double odom_yaw_ = 0.0;
    geometry_msgs::PoseStamped pillar_pre_;
    geometry_msgs::PoseStamped pillar_post_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "takeoff_align_land_demo");
    TakeoffAlignLandDemo demo;
    return demo.run();
}
