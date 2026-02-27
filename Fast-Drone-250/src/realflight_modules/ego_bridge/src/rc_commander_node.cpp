/**
 * @file rc_commander_node.cpp
 * @brief RC channel → TakeoffLand / EmergencyStop commands.
 *
 * Reads RC channels from MAVROS, performs edge detection with debounce,
 * and publishes takeoff/land/emergency_stop commands.
 *
 * Triggers:
 *   - ARM channel low→high AND MODE channel low  →  TAKEOFF
 *   - ARM channel high AND MODE channel low→high  →  LAND
 *   - ARM channel high→low                        →  EMERGENCY STOP
 */

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Empty.h>

class RcCommanderNode {
public:
    RcCommanderNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        // Load parameters
        pnh_.param("rc/channel_arm",    ch_arm_,    4);
        pnh_.param("rc/channel_mode",   ch_mode_,   5);
        pnh_.param("rc/threshold",      threshold_, 0.75);
        pnh_.param("rc/debounce_time",  debounce_,  2.0);

        sub_rc_ = nh_.subscribe("/mavros/rc/in", 10, &RcCommanderNode::rcCb, this);
        pub_takeoff_land_   = pnh_.advertise<quadrotor_msgs::TakeoffLand>("takeoff_land", 10);
        pub_emergency_stop_ = pnh_.advertise<std_msgs::Empty>("emergency_stop", 10);

        ROS_INFO("[rc_commander] Initialized. arm_ch=%d, mode_ch=%d, threshold=%.2f, debounce=%.1fs",
                 ch_arm_, ch_mode_, threshold_, debounce_);
    }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_rc_;
    ros::Publisher pub_takeoff_land_, pub_emergency_stop_;

    int    ch_arm_   = 4;
    int    ch_mode_  = 5;
    double threshold_= 0.75;
    double debounce_ = 2.0;

    // Normalized channel state (0~1): true = high
    bool prev_arm_high_  = false;
    bool prev_mode_high_ = false;
    bool first_msg_      = true;

    ros::Time last_action_time_ = ros::Time(0);

    /** Normalize RC PWM (1000–2000) to 0~1. */
    double normalize(uint16_t pwm) {
        return std::max(0.0, std::min(1.0, (pwm - 1000.0) / 1000.0));
    }

    void rcCb(const mavros_msgs::RCIn::ConstPtr& msg) {
        if ((int)msg->channels.size() <= std::max(ch_arm_, ch_mode_)) {
            ROS_WARN_THROTTLE(5.0, "[rc_commander] Not enough RC channels (%zu)",
                              msg->channels.size());
            return;
        }

        double arm_val  = normalize(msg->channels[ch_arm_]);
        double mode_val = normalize(msg->channels[ch_mode_]);

        bool arm_high  = arm_val  > threshold_;
        bool mode_high = mode_val > threshold_;

        ros::Time now = ros::Time::now();

        if (first_msg_) {
            prev_arm_high_  = arm_high;
            prev_mode_high_ = mode_high;
            first_msg_ = false;
            return;
        }

        // Debounce: skip if too soon after last action
        if ((now - last_action_time_).toSec() < debounce_) {
            prev_arm_high_  = arm_high;
            prev_mode_high_ = mode_high;
            return;
        }

        // ── TAKEOFF: ARM low→high AND MODE is low ──
        if (!prev_arm_high_ && arm_high && !mode_high) {
            ROS_INFO("[rc_commander] TAKEOFF triggered (arm low→high, mode low)");
            quadrotor_msgs::TakeoffLand cmd;
            cmd.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
            pub_takeoff_land_.publish(cmd);
            last_action_time_ = now;
        }

        // ── LAND: ARM stays high AND MODE low→high ──
        if (arm_high && prev_arm_high_ && !prev_mode_high_ && mode_high) {
            ROS_INFO("[rc_commander] LAND triggered (arm high, mode low→high)");
            quadrotor_msgs::TakeoffLand cmd;
            cmd.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
            pub_takeoff_land_.publish(cmd);
            last_action_time_ = now;
        }

        // ── EMERGENCY STOP: ARM high→low ──
        if (prev_arm_high_ && !arm_high) {
            ROS_WARN("[rc_commander] EMERGENCY STOP triggered (arm high→low)");
            std_msgs::Empty empty;
            pub_emergency_stop_.publish(empty);
            last_action_time_ = now;
        }

        prev_arm_high_  = arm_high;
        prev_mode_high_ = mode_high;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rc_commander_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    RcCommanderNode node(nh, pnh);
    ros::spin();
    return 0;
}
