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

// CRAIC 第一版主流程：
// 1. 起飞并记录起飞后悬停点作为击打区返回点。
// 2. 等待 MID360/FAST-LIO 柱子识别节点给出稳定 pre/post goal。
// 3. 用 EGO-Planner 真实飞过柱子通道。
// 4. 触发 override 任务，由 D435 深度图闭环穿框。
// 5. 二维码和气球任务先保留入口，后续替换成真实识别和执行逻辑。
class CraicCompetitionDemo {
public:
    CraicCompetitionDemo()
        : nh_(), pnh_("~"), api_(nh_, getBridgeNs()) {
        pnh_.param<double>("flight_z", flight_z_, 1.1);
        pnh_.param<double>("takeoff_timeout", takeoff_timeout_, 30.0);
        pnh_.param<double>("goal_timeout", goal_timeout_, 60.0);
        pnh_.param<double>("override_timeout", override_timeout_, 120.0);
        pnh_.param<double>("pillar_detect_timeout", pillar_detect_timeout_, 30.0);
        pnh_.param<double>("override_climb_timeout", override_climb_timeout_, 10.0);
        pnh_.param<double>("override_climb_speed", override_climb_speed_, 0.18);
        pnh_.param<double>("override_climb_threshold", override_climb_threshold_, 0.05);
        pnh_.param<double>("pillar_align_timeout", pillar_align_timeout_, 20.0);
        pnh_.param<double>("pillar_align_threshold", pillar_align_threshold_, 0.15);
        pnh_.param<double>("pillar_align_speed", pillar_align_speed_, 0.18);
        pnh_.param<double>("pillar_align_kp", pillar_align_kp_, 0.8);
        pnh_.param<double>("pillar_align_z_kp", pillar_align_z_kp_, 0.9);
        pnh_.param<double>("pillar_align_max_vz", pillar_align_max_vz_, 0.16);
        pnh_.param<double>("pillar_align_yaw_kp", pillar_align_yaw_kp_, 0.8);
        pnh_.param<double>("pillar_align_max_yaw_rate", pillar_align_max_yaw_rate_, 0.35);
        pnh_.param<double>("pillar_align_yaw_threshold", pillar_align_yaw_threshold_, 0.25);
        pnh_.param<double>("pillar_align_hold_time", pillar_align_hold_time_, 0.35);
        pnh_.param<bool>("enable_pillar_active_scan", enable_pillar_active_scan_, true);
        pnh_.param<double>("pillar_active_scan_delay", pillar_active_scan_delay_, 3.0);
        pnh_.param<double>("pillar_scan_yaw_rate", pillar_scan_yaw_rate_, 0.35);
        pnh_.param<double>("pillar_scan_period", pillar_scan_period_, 8.0);
        pnh_.param<double>("pillar_scan_lateral_speed", pillar_scan_lateral_speed_, 0.06);
        pnh_.param<double>("pillar_scan_lateral_after", pillar_scan_lateral_after_, 8.0);
        pnh_.param<double>("pillar_scan_lateral_period", pillar_scan_lateral_period_, 10.0);
        pnh_.param<double>("pillar_scan_max_vz", pillar_scan_max_vz_, 0.12);
        pnh_.param<bool>("climb_to_flight_z_before_detect", climb_to_flight_z_before_detect_, true);
        pnh_.param<bool>("land_after_finish", land_after_finish_, true);

        pre_sub_ = nh_.subscribe("/craic/pillar_pre_goal", 1, &CraicCompetitionDemo::preGoalCb, this);
        post_sub_ = nh_.subscribe("/craic/pillar_post_goal", 1, &CraicCompetitionDemo::postGoalCb, this);
        status_sub_ = nh_.subscribe("/craic/pillar_status", 1, &CraicCompetitionDemo::statusCb, this);
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
        const double yaw_forward = yawFromPose(pillar_pre_);
        const double yaw_back = normalizeYaw(yaw_forward + M_PI);

        ROS_INFO("[craic_demo] ===== PASS_PILLARS_OUTBOUND =====");
        // 第一段不再把 pillar_pre 交给 EGO：先用 OVERRIDE 横向挪到两柱中心线，
        // 避免 EGO 从起点直接规划到贴近柱子的 pre_goal 时进入障碍膨胀区。
        if (!overrideAlignToPillarCenter(yaw_forward)) {
            ROS_ERROR("[craic_demo] Pillar lateral alignment failed. Abort.");
            safeLand();
            return 1;
        }
        // 对中完成后，只给 EGO 一个柱后目标点，让规划轨迹自然穿过两柱中线。
        flyGoal("pillar_post", pillar_post_, yaw_forward);

        ROS_INFO("[craic_demo] ===== PASS_FRAME_OUTBOUND =====");
        runOverrideTask(1, "pass_frame_forward");

        ROS_INFO("[craic_demo] ===== QR_PLACEHOLDER =====");
        runOverrideTask(2, "qr_placeholder");

        ROS_INFO("[craic_demo] ===== PASS_FRAME_BACKWARD =====");
        runOverrideTask(3, "pass_frame_backward");

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
        while (ros::ok()) {
            ros::spinOnce();
            // 状态和目标点分开订阅，防止只收到 status 却还没拿到最新 goal。
            if (pillar_status_ == "valid" && have_pre_ && have_post_) {
                if (scan_active) {
                    stopPillarActiveScan();
                }
                ROS_INFO("[craic_demo] Pillars valid. pre=(%.2f %.2f %.2f), post=(%.2f %.2f %.2f)",
                         pillar_pre_.pose.position.x, pillar_pre_.pose.position.y, pillar_pre_.pose.position.z,
                         pillar_post_.pose.position.x, pillar_post_.pose.position.y, pillar_post_.pose.position.z);
                return true;
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

    void flyGoal(const std::string& label, const geometry_msgs::PoseStamped& pose, double yaw) {
        ROS_INFO("[craic_demo] Goal %s: (%.2f, %.2f, %.2f), yaw=%.2f",
                 label.c_str(),
                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw);
        const bool reached = api_.sendGoalWithYaw(pose.pose.position.x,
                                                  pose.pose.position.y,
                                                  pose.pose.position.z,
                                                  yaw,
                                                  goal_timeout_);
        if (!reached) {
            ROS_WARN("[craic_demo] Goal %s timeout, continue mission.", label.c_str());
        }
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
        const Eigen::Vector3d pre = posePosition(pillar_pre_);
        const Eigen::Vector3d post = posePosition(pillar_post_);
        const Eigen::Vector3d corridor = 0.5 * (pre + post);
        Eigen::Vector2d forward(post.x() - pre.x(), post.y() - pre.y());
        if (forward.norm() < 1e-3) {
            ROS_ERROR("[craic_demo] invalid pillar pre/post, cannot compute forward direction.");
            return false;
        }
        forward.normalize();

        // lateral 是两柱连线方向；只沿这个方向移动，就能从起点横向挪到通道中心线。
        const Eigen::Vector2d lateral(-forward.y(), forward.x());
        const Eigen::Vector2d corridor_xy(corridor.x(), corridor.y());

        ROS_INFO("[craic_demo] OVERRIDE lateral align to pillar center line: corridor=(%.2f %.2f), forward=(%.2f %.2f), lateral=(%.2f %.2f)",
                 corridor.x(), corridor.y(), forward.x(), forward.y(), lateral.x(), lateral.y());

        if (!api_.enableOverride()) return false;

        ros::Rate rate(50);
        const ros::Time start = ros::Time::now();
        ros::Time stable_since;
        bool stable_started = false;
        bool reached = false;

        while (ros::ok()) {
            ros::spinOnce();
            const Eigen::Vector3d pos = api_.getOdomPosition();
            const Eigen::Vector2d pos_xy(pos.x(), pos.y());
            const Eigen::Vector2d to_center = corridor_xy - pos_xy;

            const double lateral_err = to_center.dot(lateral);
            const double along_err = to_center.dot(forward);
            const double z_err = flight_z_ - pos.z();
            const double yaw_err = normalizeYaw(yaw_forward - api_.getOdomYaw());

            double v_lat = clampValue(pillar_align_kp_ * lateral_err,
                                      -pillar_align_speed_, pillar_align_speed_);
            if (std::abs(lateral_err) < pillar_align_threshold_) v_lat = 0.0;

            double vz = clampValue(pillar_align_z_kp_ * z_err,
                                   -pillar_align_max_vz_, pillar_align_max_vz_);
            if (std::abs(z_err) < 0.05) vz = 0.0;

            double yaw_rate = clampValue(pillar_align_yaw_kp_ * yaw_err,
                                         -pillar_align_max_yaw_rate_, pillar_align_max_yaw_rate_);
            if (std::abs(yaw_err) < pillar_align_yaw_threshold_) yaw_rate = 0.0;

            api_.sendVelocityCmd(v_lat * lateral.x(),
                                 v_lat * lateral.y(),
                                 vz,
                                 yaw_rate);

            // 这里的“对中”只看横向误差和高度误差；yaw 只顺手慢速修正，
            // 不作为硬门槛，避免因为 yaw 控制响应慢导致一直悬停。
            const bool centered = std::abs(lateral_err) < pillar_align_threshold_ &&
                                  std::abs(z_err) < 0.10;
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

            ROS_INFO_THROTTLE(0.5,
                              "[craic_demo] pillar align pos=(%.2f %.2f %.2f) lat_err=%.2f along_err=%.2f z_err=%.2f yaw_err=%.2f v_lat=%.2f",
                              pos.x(), pos.y(), pos.z(), lateral_err, along_err, z_err, yaw_err, v_lat);

            if ((ros::Time::now() - start).toSec() > pillar_align_timeout_) {
                ROS_WARN("[craic_demo] pillar align timeout lat_err=%.2f along_err=%.2f z_err=%.2f yaw_err=%.2f",
                         lateral_err, along_err, z_err, yaw_err);
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
    }

    void postGoalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pillar_post_ = *msg;
        pillar_post_.pose.position.z = flight_z_;
        have_post_ = true;
    }

    void statusCb(const std_msgs::String::ConstPtr& msg) {
        pillar_status_ = msg->data;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    EgoApi api_;
    ros::Subscriber pre_sub_;
    ros::Subscriber post_sub_;
    ros::Subscriber status_sub_;

    double flight_z_ = 1.1;
    double takeoff_timeout_ = 30.0;
    double goal_timeout_ = 60.0;
    double override_timeout_ = 120.0;
    double pillar_detect_timeout_ = 30.0;
    double override_climb_timeout_ = 10.0;
    double override_climb_speed_ = 0.18;
    double override_climb_threshold_ = 0.05;
    double pillar_align_timeout_ = 20.0;
    double pillar_align_threshold_ = 0.15;
    double pillar_align_speed_ = 0.18;
    double pillar_align_kp_ = 0.8;
    double pillar_align_z_kp_ = 0.9;
    double pillar_align_max_vz_ = 0.16;
    double pillar_align_yaw_kp_ = 0.8;
    double pillar_align_max_yaw_rate_ = 0.35;
    double pillar_align_yaw_threshold_ = 0.25;
    double pillar_align_hold_time_ = 0.35;
    bool enable_pillar_active_scan_ = true;
    double pillar_active_scan_delay_ = 3.0;
    double pillar_scan_yaw_rate_ = 0.35;
    double pillar_scan_period_ = 8.0;
    double pillar_scan_lateral_speed_ = 0.06;
    double pillar_scan_lateral_after_ = 8.0;
    double pillar_scan_lateral_period_ = 10.0;
    double pillar_scan_max_vz_ = 0.12;
    bool land_after_finish_ = true;
    bool climb_to_flight_z_before_detect_ = true;

    bool have_pre_ = false;
    bool have_post_ = false;
    std::string pillar_status_ = "unknown";
    geometry_msgs::PoseStamped pillar_pre_;
    geometry_msgs::PoseStamped pillar_post_;
    Eigen::Vector3d home_hover_ = Eigen::Vector3d::Zero();
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "craic_competition_demo");
    CraicCompetitionDemo demo;
    return demo.run();
}
