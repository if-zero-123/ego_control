#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <json/json.h>
#include <mavros_msgs/ActuatorControl.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include "d_task_uav_control/AprilTagRange.h"
#include "d_task_uav_control/PlatformDetection.h"
#include "d_task_uav_control/PlatformEstimate.h"
#include "d_task_uav_control/mission_controller.h"
#include "d_task_uav_control/payload_pulse.h"
#include "d_task_uav_control/pixel_servo.h"
#include "ego_api/ego_api.h"

namespace d_task_uav_control {
namespace {

bool parseJsonObject(const std::string& raw, Json::Value& value,
                     std::string& error) {
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(raw.data(), raw.data() + raw.size(), &value, &error)) {
        return false;
    }
    if (!value.isObject()) {
        error = "root must be an object";
        return false;
    }
    return true;
}

std::string writeJson(const Json::Value& value) {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    return Json::writeString(builder, value);
}

bool parseMode(const std::string& raw, MissionMode& mode) {
    if (raw == "DROP") {
        mode = MissionMode::DROP;
        return true;
    }
    if (raw == "DYNAMIC_LANDING") {
        mode = MissionMode::DYNAMIC_LANDING;
        return true;
    }
    return false;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
    return std::atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z));
}

double stampOrNow(const ros::Time& stamp) {
    return stamp.isZero() ? ros::Time::now().toSec() : stamp.toSec();
}

class PayloadActuator {
public:
    PayloadActuator(ros::NodeHandle& node, ros::NodeHandle& private_node)
        : pulse_(loadPulseDuration(private_node)) {
        private_node.param("payload/enabled", enabled_, false);
        private_node.param("payload/backend", backend_, std::string("mavros"));
        private_node.param("payload/group_mix", group_mix_, 2);
        private_node.param("payload/channel", channel_, 0);
        private_node.param("payload/release_value", release_value_, 1.0);
        private_node.param("payload/neutral_value", neutral_value_, 0.0);
        private_node.param("payload/gpio_wiringpi_pin", gpio_wiringpi_pin_, 9);
        private_node.param("payload/gpio_active_high", gpio_active_high_, true);
        const std::string topic = private_node.param<std::string>(
            "payload/topic", "/mavros/actuator_control");
        if (backend_ == "mavros") {
            if (channel_ < 0 || channel_ >= 8) {
                throw std::runtime_error("payload/channel must be in [0, 7]");
            }
            publisher_ = node.advertise<mavros_msgs::ActuatorControl>(topic, 5);
        } else if (backend_ == "gpio") {
            if (gpio_wiringpi_pin_ < 0) {
                throw std::runtime_error("payload/gpio_wiringpi_pin must be non-negative");
            }
            if (!setGpioOutputMode() || !writeGpio(false, true)) {
                throw std::runtime_error("failed to initialize payload GPIO through /usr/bin/gpio");
            }
        } else {
            throw std::runtime_error("payload/backend must be mavros or gpio");
        }
    }

    ~PayloadActuator() {
        if (backend_ == "gpio") {
            writeGpio(false, true);
        }
    }

    bool trigger(double now_s) {
        return pulse_.trigger(now_s);
    }

    void update(double now_s) {
        const PayloadPulseCommand command = pulse_.update(now_s);
        if (!enabled_ || command == PayloadPulseCommand::NONE) {
            return;
        }
        if (command == PayloadPulseCommand::RELEASE) {
            writeRelease();
        } else if (command == PayloadPulseCommand::NEUTRAL) {
            writeNeutral();
        }
    }

    void reset() {
        const PayloadPulseCommand command = pulse_.reset();
        if (backend_ == "gpio") {
            if (!writeGpio(false, true)) {
                ROS_ERROR("[d_task_mission] failed to reset payload GPIO to low");
            }
        } else if (enabled_ && command == PayloadPulseCommand::NEUTRAL) {
            publish(neutral_value_);
        }
    }

    bool enabled() const { return enabled_; }

private:
    static double loadPulseDuration(ros::NodeHandle& private_node) {
        double pulse_duration_s = 0.50;
        private_node.param("payload/pulse_duration_s", pulse_duration_s, 0.50);
        if (pulse_duration_s < 0.0) {
            throw std::runtime_error("payload/pulse_duration_s cannot be negative");
        }
        return pulse_duration_s;
    }

    void writeRelease() {
        if (backend_ == "gpio") {
            if (!writeGpio(true)) {
                ROS_ERROR("[d_task_mission] failed to set payload GPIO high");
            }
        } else {
            publish(release_value_);
        }
    }

    void writeNeutral() {
        if (backend_ == "gpio") {
            if (!writeGpio(false)) {
                ROS_ERROR("[d_task_mission] failed to set payload GPIO low");
            }
        } else {
            publish(neutral_value_);
        }
    }

    bool setGpioOutputMode() const {
        const std::string pin = std::to_string(gpio_wiringpi_pin_);
        return runGpioCommand("mode", pin, "out");
    }

    bool writeGpio(bool release, bool force = false) {
        const bool high_level = gpio_active_high_ ? release : !release;
        if (!force && gpio_level_known_ && gpio_level_high_ == high_level) {
            return true;
        }
        const std::string pin = std::to_string(gpio_wiringpi_pin_);
        if (!runGpioCommand("write", pin, high_level ? "1" : "0")) {
            return false;
        }
        gpio_level_known_ = true;
        gpio_level_high_ = high_level;
        return true;
    }

    static bool runGpioCommand(const char* command, const std::string& pin,
                               const char* value) {
        const pid_t child = fork();
        if (child < 0) {
            return false;
        }
        if (child == 0) {
            execl("/usr/bin/gpio", "gpio", command, pin.c_str(), value,
                  static_cast<char*>(nullptr));
            _exit(127);
        }

        int status = 0;
        pid_t result = -1;
        do {
            result = waitpid(child, &status, 0);
        } while (result == -1 && errno == EINTR);
        return result == child && WIFEXITED(status) && WEXITSTATUS(status) == 0;
    }

    void publish(double channel_value) {
        mavros_msgs::ActuatorControl message;
        message.header.stamp = ros::Time::now();
        message.group_mix = static_cast<uint8_t>(group_mix_);
        for (std::size_t index = 0; index < message.controls.size(); ++index) {
            message.controls[index] = static_cast<float>(neutral_value_);
        }
        message.controls[static_cast<std::size_t>(channel_)] =
            static_cast<float>(channel_value);
        publisher_.publish(message);
    }

    ros::Publisher publisher_;
    bool enabled_ = false;
    std::string backend_ = "mavros";
    int group_mix_ = 2;
    int channel_ = 0;
    double release_value_ = 1.0;
    double neutral_value_ = 0.0;
    int gpio_wiringpi_pin_ = 9;
    bool gpio_active_high_ = true;
    bool gpio_level_known_ = false;
    bool gpio_level_high_ = false;
    PayloadPulse pulse_;
};

class DTaskMissionNode {
public:
    DTaskMissionNode()
        : private_node_("~"),
          controller_config_(loadControllerConfig()),
          controller_(controller_config_),
          pixel_servo_config_(loadPixelServoConfig()),
          pixel_servo_(pixel_servo_config_),
          payload_actuator_(node_, private_node_) {
        const std::string bridge_namespace = private_node_.param<std::string>(
            "mission/bridge_namespace", "/ego_bridge");
        ego_api_.reset(new EgoApi(node_, bridge_namespace));
        private_node_.param("mission/odom_timeout_s", odom_timeout_s_, 0.30);
        private_node_.param("mission/platform_estimate_timeout_s",
                            platform_estimate_timeout_s_, 0.35);
        private_node_.param("mission/apriltag_range_timeout_s",
                            apriltag_range_timeout_s_, 0.35);
        private_node_.param("mission/control_rate_hz", control_rate_hz_, 30.0);
        private_node_.param("diagnostics/log_rate_hz", diagnostic_log_rate_hz_,
                            1.0);
        private_node_.param("diagnostics/source_timeout_s",
                            diagnostic_source_timeout_s_, 1.0);
        if (control_rate_hz_ < 10.0 || !std::isfinite(control_rate_hz_)
            || apriltag_range_timeout_s_ <= 0.0
            || !std::isfinite(apriltag_range_timeout_s_)
            || diagnostic_log_rate_hz_ < 0.0
            || !std::isfinite(diagnostic_log_rate_hz_)
            || diagnostic_source_timeout_s_ <= 0.0
            || !std::isfinite(diagnostic_source_timeout_s_)) {
            throw std::runtime_error(
                "mission control rate, AprilTag timeout, or diagnostic rate is invalid");
        }

        subscribeTopics();
        advertiseTopics();
        control_timer_ = node_.createTimer(
            ros::Duration(1.0 / control_rate_hz_),
            &DTaskMissionNode::controlTimerCallback, this);
        ROS_INFO("[d_task_mission] ready, payload actuator %s",
                 payload_actuator_.enabled() ? "ENABLED" : "dry-run");
    }

private:
    MissionControllerConfig loadControllerConfig() {
        MissionControllerConfig config;
#define LOAD_MISSION_PARAM(name, field) \
        private_node_.param("mission/" name, config.field, config.field)
        LOAD_MISSION_PARAM("cruise_height_m", cruise_height_m);
        LOAD_MISSION_PARAM("hover_time_s", hover_time_s);
        LOAD_MISSION_PARAM("vision_lock_time_s", vision_lock_time_s);
        LOAD_MISSION_PARAM("follow_stable_time_s", follow_stable_time_s);
        LOAD_MISSION_PARAM("phase_stable_time_s", phase_stable_time_s);
        LOAD_MISSION_PARAM("release_settle_time_s", release_settle_time_s);
        LOAD_MISSION_PARAM("platform_hold_time_s", platform_hold_time_s);
        LOAD_MISSION_PARAM("search_start_x_m", search_start_x_m);
        LOAD_MISSION_PARAM("search_start_y_m", search_start_y_m);
        LOAD_MISSION_PARAM("search_forward_distance_m",
                           search_forward_distance_m);
        LOAD_MISSION_PARAM("drop_height_m", drop_height_m);
        LOAD_MISSION_PARAM("drop_apriltag_distance_m",
                           drop_apriltag_distance_m);
        LOAD_MISSION_PARAM("high_descent_height_m", high_descent_height_m);
        LOAD_MISSION_PARAM("low_descent_height_m", low_descent_height_m);
        LOAD_MISSION_PARAM("platform_press_depth_m", platform_press_depth_m);
        LOAD_MISSION_PARAM("follow_lead_time_s", follow_lead_time_s);
        LOAD_MISSION_PARAM("follow_xy_kp", follow_xy_kp);
        LOAD_MISSION_PARAM("follow_xy_kd", follow_xy_kd);
        LOAD_MISSION_PARAM("follow_position_deadband_m",
                           follow_position_deadband_m);
        LOAD_MISSION_PARAM("follow_max_correction_mps",
                           follow_max_correction_mps);
        LOAD_MISSION_PARAM("follow_max_total_speed_mps",
                           follow_max_total_speed_mps);
        LOAD_MISSION_PARAM("follow_max_accel_mps2",
                           follow_max_accel_mps2);
        LOAD_MISSION_PARAM("vision_trim_max_speed_mps",
                           vision_trim_max_speed_mps);
        LOAD_MISSION_PARAM("xy_tolerance_m", xy_tolerance_m);
        LOAD_MISSION_PARAM("relative_speed_tolerance_mps",
                           relative_speed_tolerance_mps);
        LOAD_MISSION_PARAM("height_tolerance_m", height_tolerance_m);
        LOAD_MISSION_PARAM("home_xy_tolerance_m", home_xy_tolerance_m);
        LOAD_MISSION_PARAM("home_height_tolerance_m",
                           home_height_tolerance_m);
        LOAD_MISSION_PARAM("high_descent_speed_mps", high_descent_speed_mps);
        LOAD_MISSION_PARAM("low_descent_speed_mps", low_descent_speed_mps);
        LOAD_MISSION_PARAM("contact_descent_speed_mps",
                           contact_descent_speed_mps);
        LOAD_MISSION_PARAM("climb_speed_mps", climb_speed_mps);
        LOAD_MISSION_PARAM("takeoff_timeout_s", takeoff_timeout_s);
        LOAD_MISSION_PARAM("search_timeout_s", search_timeout_s);
        LOAD_MISSION_PARAM("tracking_loss_timeout_s",
                           tracking_loss_timeout_s);
        LOAD_MISSION_PARAM("descent_timeout_s", descent_timeout_s);
        LOAD_MISSION_PARAM("platform_contact_timeout_s",
                           platform_contact_timeout_s);
        LOAD_MISSION_PARAM("landing_prediction_timeout_s",
                           landing_prediction_timeout_s);
        LOAD_MISSION_PARAM("landing_visual_handoff_s",
                           landing_visual_handoff_s);
        LOAD_MISSION_PARAM("platform_takeoff_timeout_s",
                           platform_takeoff_timeout_s);
        LOAD_MISSION_PARAM("return_timeout_s", return_timeout_s);
        LOAD_MISSION_PARAM("home_land_timeout_s", home_land_timeout_s);
        LOAD_MISSION_PARAM("total_timeout_s", total_timeout_s);
        LOAD_MISSION_PARAM("request_retry_s", request_retry_s);
        LOAD_MISSION_PARAM("drop_force_descent_distance_to_d_m",
                           drop_force_descent_distance_to_d_m);
        LOAD_MISSION_PARAM("drop_abort_distance_to_d_m",
                           drop_abort_distance_to_d_m);
        private_node_.param("mission/max_dynamic_landing_retries",
                            config.max_dynamic_landing_retries,
                            config.max_dynamic_landing_retries);
#undef LOAD_MISSION_PARAM
        validateControllerConfig(config);
        return config;
    }

    static void validateControllerConfig(const MissionControllerConfig& config) {
        if (config.cruise_height_m <= 0.3
            || !std::isfinite(config.search_start_x_m)
            || !std::isfinite(config.search_start_y_m)
            || !std::isfinite(config.search_forward_distance_m)
            || config.search_forward_distance_m <= 0.0
            || config.high_descent_height_m <= config.low_descent_height_m
            || config.low_descent_height_m <= 0.0
            || config.drop_height_m <= 0.0
            || config.drop_apriltag_distance_m <= 0.0
            || config.platform_hold_time_s < 0.0
            || config.follow_xy_kp < 0.0
            || config.follow_xy_kd < 0.0
            || config.follow_position_deadband_m < 0.0
            || config.follow_max_correction_mps <= 0.0
            || config.follow_max_total_speed_mps <= 0.0
            || config.follow_max_accel_mps2 < 0.0
            || config.vision_trim_max_speed_mps < 0.0
            || config.landing_prediction_timeout_s <= 0.0
            || config.landing_visual_handoff_s < 0.0
            || config.max_dynamic_landing_retries < 0) {
            throw std::runtime_error(
                "invalid mission height, follow-control, hold, or retry parameters");
        }
    }

    PixelServoConfig loadPixelServoConfig() {
        PixelServoConfig config;
#define LOAD_PIXEL_PARAM(name, field) \
        private_node_.param("pixel_servo/" name, config.field, config.field)
        LOAD_PIXEL_PARAM("filter_alpha", filter_alpha);
        LOAD_PIXEL_PARAM("filter_beta", filter_beta);
        LOAD_PIXEL_PARAM("source_timeout_s", source_timeout_s);
        LOAD_PIXEL_PARAM("min_confidence", min_confidence);
        LOAD_PIXEL_PARAM("deadband", deadband);
        LOAD_PIXEL_PARAM("gain_mps", gain_mps);
        LOAD_PIXEL_PARAM("max_speed_mps", max_speed_mps);
        LOAD_PIXEL_PARAM("body_x_from_u", body_x_from_u);
        LOAD_PIXEL_PARAM("body_x_from_v", body_x_from_v);
        LOAD_PIXEL_PARAM("body_y_from_u", body_y_from_u);
        LOAD_PIXEL_PARAM("body_y_from_v", body_y_from_v);
#undef LOAD_PIXEL_PARAM
        if (!pixelServoConfigValid(config)) {
            throw std::runtime_error("invalid pixel_servo configuration");
        }
        return config;
    }

    void subscribeTopics() {
        mission_config_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/mission_config", "/uav_protocol/mission_config"),
            5, &DTaskMissionNode::missionConfigCallback, this);
        mission_start_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/mission_start", "/uav_protocol/mission_start"),
            5, &DTaskMissionNode::missionStartCallback, this);
        positioning_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/positioning_status", "/d_task/positioning/status"),
            5, &DTaskMissionNode::positioningStatusCallback, this);
        platform_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/platform_estimate", "/d_task/tracking/platform_estimate"),
            10, &DTaskMissionNode::platformEstimateCallback, this);
        detection_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/platform_detection", "/d_task/vision/platform_detection"),
            10, &DTaskMissionNode::platformDetectionCallback, this);
        apriltag_range_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/apriltag_range", "/d_task/vision/apriltag_range"),
            10, &DTaskMissionNode::aprilTagRangeCallback, this);
        odom_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/px4_odom", "/mavros/local_position/odom"),
            10, &DTaskMissionNode::odomCallback, this);
        bridge_state_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/bridge_state", "/ego_bridge/flight_state"),
            10, &DTaskMissionNode::bridgeStateCallback, this);
        control_mode_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/control_mode", "/ego_bridge/control_mode"),
            10, &DTaskMissionNode::controlModeCallback, this);
        descent_allowed_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/dynamic_descent_allowed",
                "/uav_protocol/dynamic_descent_allowed"),
            10, &DTaskMissionNode::descentAllowedCallback, this);
        safety_hold_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/safety_hold", "/uav_protocol/safety_hold"),
            10, &DTaskMissionNode::safetyHoldCallback, this);
        car_pose_meta_subscriber_ = node_.subscribe(
            private_node_.param<std::string>(
                "topics/car_pose_meta", "/uav_protocol/car/pose_meta"),
            10, &DTaskMissionNode::carPoseMetaCallback, this);
    }

    void advertiseTopics() {
        task_state_publisher_ = node_.advertise<std_msgs::String>(
            private_node_.param<std::string>(
                "topics/task_state", "/uav_protocol/task_state"),
            10, true);
        event_publisher_ = node_.advertise<std_msgs::String>(
            private_node_.param<std::string>(
                "topics/local_event", "/uav_protocol/local_event"),
            10);
        fault_publisher_ = node_.advertise<std_msgs::String>(
            private_node_.param<std::string>(
                "topics/local_fault", "/uav_protocol/local_fault"),
            10);
        health_publisher_ = node_.advertise<std_msgs::String>(
            private_node_.param<std::string>(
                "topics/local_health", "/uav_protocol/local_health"),
            5);
        mission_reset_publisher_ = node_.advertise<std_msgs::String>(
            private_node_.param<std::string>(
                "topics/mission_reset", "/uav_protocol/mission_reset"),
            2);
    }

    void missionConfigCallback(const std_msgs::String::ConstPtr& message) {
        if (message->data.empty()) {
            return;
        }
        Json::Value root;
        std::string error;
        if (!parseJsonObject(message->data, root, error)
            || root.get("type", "").asString() != "mission_config"
            || root.get("sender", "").asString() != "car"
            || !root["payload"].isObject()) {
            publishFault(2301, "invalid_local_mission_config", error);
            return;
        }
        MissionMode mode;
        if (!parseMode(root["payload"].get("mode", "").asString(), mode)) {
            publishFault(2301, "invalid_local_mission_mode", "");
            return;
        }
        const std::string mission_id = root.get("mission_id", "").asString();
        const MissionState previous_state = controller_.state();
        controller_.reset();
        pixel_servo_.reset();
        payload_actuator_.reset();
        terminal_published_ = false;
        last_fault_key_.clear();
        if (!controller_.configure(mission_id, mode)) {
            publishFault(2301, "mission_config_rejected_by_controller", "");
            return;
        }
        logStateTransition(previous_state, "mission_config");
        publishEvent("MISSION_CONTROLLER_CONFIGURED");
        publishTaskState();
    }

    void positioningStatusCallback(const std_msgs::String::ConstPtr& message) {
        Json::Value root;
        std::string error;
        if (!parseJsonObject(message->data, root, error)
            || !root.get("ready", false).asBool()) {
            return;
        }
        const std::string mission_id = root.get("mission_id", "").asString();
        HomePosition home;
        home.x = root.get("home_x", 0.0).asDouble();
        home.y = root.get("home_y", 0.0).asDouble();
        home.z = root.get("home_z", 0.0).asDouble();
        if (has_odom_) {
            home.yaw = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        }
        const MissionState previous_state = controller_.state();
        if (controller_.markPositioningReady(mission_id, home)) {
            logStateTransition(previous_state, "positioning_ready");
            publishEvent("MISSION_CONTROLLER_READY");
            publishTaskState();
        }
    }

    void missionStartCallback(const std_msgs::String::ConstPtr& message) {
        if (message->data.empty()) {
            return;
        }
        Json::Value root;
        std::string error;
        if (!parseJsonObject(message->data, root, error)
            || root.get("type", "").asString() != "mission_start"
            || root.get("sender", "").asString() != "car"
            || !root["payload"].isObject()) {
            publishFault(2302, "invalid_local_mission_start", error);
            return;
        }
        MissionMode mode;
        if (!parseMode(root["payload"].get("mode", "").asString(), mode)) {
            publishFault(2302, "invalid_local_start_mode", "");
            return;
        }
        const MissionState previous_state = controller_.state();
        if (!controller_.start(
                root.get("mission_id", "").asString(), mode,
                root["payload"].get("start_reason", "").asString(),
                ros::Time::now().toSec())) {
            publishFault(2303, "mission_start_rejected_by_controller", "");
            return;
        }
        logStateTransition(previous_state, "mission_start");
        publishEvent("MISSION_CONTROLLER_STARTED");
        publishTaskState();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
        latest_odom_ = *message;
        has_odom_ = true;
        last_odom_received_s_ = ros::Time::now().toSec();
    }

    void platformEstimateCallback(const PlatformEstimate::ConstPtr& message) {
        latest_platform_ = *message;
        has_platform_ = true;
        last_platform_received_s_ = ros::Time::now().toSec();
    }

    void platformDetectionCallback(const PlatformDetection::ConstPtr& message) {
        latest_detection_found_ = message->found;
        latest_detection_predicted_ = message->predicted;
        latest_detection_confidence_ = message->confidence;
        latest_detection_measurement_age_s_ = message->measurement_age_s;
        last_detection_received_s_ = ros::Time::now().toSec();
        if (!message->found || message->predicted) {
            return;
        }
        pixel_servo_.update(PixelMeasurement{
            stampOrNow(message->header.stamp), message->center_u, message->center_v,
            message->image_width, message->image_height, message->confidence});
    }

    void aprilTagRangeCallback(const AprilTagRange::ConstPtr& message) {
        latest_apriltag_detected_ = message->detected;
        latest_apriltag_pose_valid_ = message->pose_valid;
        latest_apriltag_visible_ids_ = message->visible_tag_ids;
        latest_apriltag_used_ids_ = message->used_tag_ids;
        last_apriltag_message_received_s_ = ros::Time::now().toSec();
        has_apriltag_range_ = message->detected && message->pose_valid
            && std::isfinite(message->plane_distance_m)
            && message->plane_distance_m > 0.0;
        if (has_apriltag_range_) {
            apriltag_plane_distance_m_ = message->plane_distance_m;
            last_apriltag_range_received_s_ = ros::Time::now().toSec();
        }
        apriltag_center_tag_visible_ =
            message->detected && message->center_tag_visible;
        last_apriltag_center_received_s_ = ros::Time::now().toSec();
    }

    void bridgeStateCallback(const std_msgs::String::ConstPtr& message) {
        bridge_state_ = message->data;
        last_bridge_state_received_s_ = ros::Time::now().toSec();
    }

    void controlModeCallback(const std_msgs::UInt8::ConstPtr& message) {
        control_mode_ = message->data;
        last_control_mode_received_s_ = ros::Time::now().toSec();
    }

    void descentAllowedCallback(const std_msgs::Bool::ConstPtr& message) {
        descent_allowed_ = message->data;
        last_descent_allowed_received_s_ = ros::Time::now().toSec();
    }

    void safetyHoldCallback(const std_msgs::Bool::ConstPtr& message) {
        safety_hold_ = message->data;
        last_safety_hold_received_s_ = ros::Time::now().toSec();
    }

    void carPoseMetaCallback(const std_msgs::String::ConstPtr& message) {
        Json::Value root;
        std::string error;
        if (!parseJsonObject(message->data, root, error)
            || !root["payload"].isObject()
            || root.get("mission_id", "").asString() != controller_.missionId()) {
            return;
        }
        const Json::Value& distance = root["payload"]["distance_to_d_m"];
        if (distance.isNumeric()) {
            distance_to_d_m_ = distance.asDouble();
        }
        last_car_pose_meta_received_s_ = ros::Time::now().toSec();
    }

    MissionInput buildInput(double now_s) const {
        MissionInput input;
        input.now_s = now_s;
        input.uav_valid = has_odom_
            && now_s - last_odom_received_s_ <= odom_timeout_s_;
        if (has_odom_) {
            input.uav_x = latest_odom_.pose.pose.position.x;
            input.uav_y = latest_odom_.pose.pose.position.y;
            input.uav_z = latest_odom_.pose.pose.position.z;
            input.uav_vx = latest_odom_.twist.twist.linear.x;
            input.uav_vy = latest_odom_.twist.twist.linear.y;
            input.uav_vz = latest_odom_.twist.twist.linear.z;
        }
        input.bridge_state = bridge_state_;
        input.control_mode = control_mode_;
        input.platform_valid = has_platform_ && latest_platform_.valid
            && now_s - last_platform_received_s_ <= platform_estimate_timeout_s_;
        if (has_platform_) {
            input.platform_vision_detected = latest_platform_.vision_detected;
            input.platform_x = latest_platform_.x;
            input.platform_y = latest_platform_.y;
            input.platform_z = latest_platform_.z;
            input.platform_vx = latest_platform_.vx;
            input.platform_vy = latest_platform_.vy;
        }
        const double yaw = has_odom_
            ? yawFromQuaternion(latest_odom_.pose.pose.orientation) : 0.0;
        const PixelServoState pixel_state = pixel_servo_.stateAt(now_s, yaw);
        input.pixel_valid = pixel_state.valid;
        input.pixel_aligned = pixel_state.valid
            && std::abs(pixel_state.error_u) <= 1e-9
            && std::abs(pixel_state.error_v) <= 1e-9;
        input.pixel_world_vx = pixel_state.world_vx;
        input.pixel_world_vy = pixel_state.world_vy;
        input.apriltag_range_valid = has_apriltag_range_
            && now_s - last_apriltag_range_received_s_
                <= apriltag_range_timeout_s_;
        input.apriltag_center_tag_visible = apriltag_center_tag_visible_
            && now_s - last_apriltag_center_received_s_
                <= apriltag_range_timeout_s_;
        input.apriltag_plane_distance_m = apriltag_plane_distance_m_;
        input.descent_allowed = descent_allowed_;
        input.safety_hold = safety_hold_;
        input.distance_to_d_m = distance_to_d_m_;
        return input;
    }

    void controlTimerCallback(const ros::TimerEvent&) {
        const double now_s = ros::Time::now().toSec();
        const MissionState previous_state = controller_.state();
        const MissionInput input = buildInput(now_s);
        const MissionCommand command = controller_.update(input);
        if (command.fault_code == 0) {
            last_fault_key_.clear();
        }
        executeCommand(command, now_s);
        payload_actuator_.update(now_s);

        if (controller_.state() != previous_state) {
            Json::Value details;
            details["previous_state"] = missionStateName(previous_state);
            details["state"] = missionStateName(controller_.state());
            details["retry_count"] = controller_.retryCount();
            publishEvent("MISSION_STATE_CHANGED", details);
            logStateTransition(previous_state, "control_update");
        }
        logDiagnosticSnapshot(input, command, now_s);
        publishTaskState();
        if (now_s - last_health_publish_s_ >= 1.0) {
            publishHealth(now_s);
            last_health_publish_s_ = now_s;
        }
        if ((command.complete || command.abort) && !terminal_published_) {
            publishTerminal(command.abort ? "ABORT" : "COMPLETE");
            terminal_published_ = true;
        }
    }

    void executeCommand(const MissionCommand& command, double now_s) {
        if (command.request_takeoff) {
            ego_api_->requestTakeoffTo(command.target_z);
        }
        if (command.override_mode_request >= 0) {
            ego_api_->requestOverrideMode(command.override_mode_request == 1);
        }
        if (command.setpoint_valid) {
            ego_api_->sendPositionVelocityCmd(
                command.target_x, command.target_y, command.target_z,
                command.target_vx, command.target_vy, command.target_vz,
                command.target_yaw);
        }
        if (command.request_platform_land) {
            ego_api_->requestPlatformDisarm();
        }
        if (command.request_platform_cancel) {
            ego_api_->requestPlatformLandingCancel();
        }
        if (command.request_platform_takeoff) {
            ego_api_->requestPlatformTakeoff(
                command.target_x, command.target_y, command.target_z,
                command.target_yaw);
        }
        if (command.request_home_land) {
            ego_api_->requestLand();
        }
        if (command.release_payload && payload_actuator_.trigger(now_s)) {
            Json::Value details;
            details["hardware_enabled"] = payload_actuator_.enabled();
            publishEvent(payload_actuator_.enabled()
                             ? "PAYLOAD_RELEASE_TRIGGERED"
                             : "PAYLOAD_RELEASE_DRY_RUN",
                         details);
        }
        if (command.fault_code != 0) {
            publishFault(command.fault_code, command.fault_text, "");
        }
    }

    static const char* targetName(MissionState state) {
        switch (state) {
            case MissionState::MOVE_TO_SEARCH_START: return "SEARCH_START_A";
            case MissionState::FORWARD_SEARCH: return "SEARCH_END";
            case MissionState::LOCK_CAR:
            case MissionState::FOLLOW_CAR:
            case MissionState::DROP_DESCEND:
            case MissionState::DESCEND_HIGH:
            case MissionState::DESCEND_LOW:
            case MissionState::LAND_ON_PLATFORM: return "PLATFORM";
            case MissionState::RETURN_HOME:
            case MissionState::LAND_HOME: return "HOME";
            default: return "NONE";
        }
    }

    void logStateTransition(MissionState previous_state, const char* reason) {
        if (controller_.state() == previous_state) {
            return;
        }
        ROS_INFO("[d_task_mission][STATE] mission=%s mode=%s %s -> %s retry=%d reason=%s",
                 controller_.missionId().c_str(),
                 missionModeName(controller_.mode()),
                 missionStateName(previous_state),
                 missionStateName(controller_.state()),
                 controller_.retryCount(), reason);
    }

    static double diagnosticNumber(double value) {
        return std::isfinite(value) ? value : -1.0;
    }

    static double sourceAge(double now_s, double last_received_s) {
        return last_received_s >= 0.0 ? now_s - last_received_s : -1.0;
    }

    bool sourceFresh(double now_s, double last_received_s) const {
        const double age_s = sourceAge(now_s, last_received_s);
        return age_s >= 0.0 && age_s <= diagnostic_source_timeout_s_;
    }

    static Json::Value idArray(const std::vector<int32_t>& ids) {
        Json::Value value(Json::arrayValue);
        for (const int32_t id : ids) {
            value.append(id);
        }
        return value;
    }

    void logDiagnosticSnapshot(const MissionInput& input,
                               const MissionCommand& command,
                               double now_s) {
        if (diagnostic_log_rate_hz_ <= 0.0
            || now_s - last_diagnostic_log_s_
                < 1.0 / diagnostic_log_rate_hz_) {
            return;
        }
        last_diagnostic_log_s_ = now_s;

        Json::Value value;
        value["mission_id"] = controller_.missionId();
        value["mode"] = missionModeName(controller_.mode());
        value["state"] = missionStateName(controller_.state());
        value["state_before"] = missionStateName(command.state);
        value["state_after"] = missionStateName(controller_.state());
        value["state_target"] = targetName(command.state);
        value["bridge_state"] = input.bridge_state;
        value["bridge_state_age_s"] =
            sourceAge(now_s, last_bridge_state_received_s_);
        value["bridge_state_fresh"] =
            sourceFresh(now_s, last_bridge_state_received_s_);
        value["control_mode"] = static_cast<unsigned int>(input.control_mode);
        value["control_mode_age_s"] =
            sourceAge(now_s, last_control_mode_received_s_);
        value["control_mode_fresh"] =
            sourceFresh(now_s, last_control_mode_received_s_);

        value["uav_valid"] = input.uav_valid;
        value["uav_odom_age_s"] = sourceAge(now_s, last_odom_received_s_);
        value["uav_x"] = input.uav_x;
        value["uav_y"] = input.uav_y;
        value["uav_z"] = input.uav_z;
        value["uav_vx"] = input.uav_vx;
        value["uav_vy"] = input.uav_vy;
        value["uav_vz"] = input.uav_vz;

        value["search_start_error_m"] = std::hypot(
            input.uav_x - controller_config_.search_start_x_m,
            input.uav_y - controller_config_.search_start_y_m);
        value["search_end_error_m"] = std::hypot(
            input.uav_x - controller_config_.search_start_x_m
                - controller_config_.search_forward_distance_m,
            input.uav_y - controller_config_.search_start_y_m);

        value["platform_valid"] = input.platform_valid;
        value["platform_receive_age_s"] =
            sourceAge(now_s, last_platform_received_s_);
        value["platform_filter_mode"] =
            static_cast<unsigned int>(latest_platform_.filter_mode);
        value["platform_measurement_age_s"] =
            static_cast<double>(latest_platform_.measurement_age_ms) / 1000.0;
        value["platform_confidence"] = latest_platform_.confidence;
        value["platform_vision"] = input.platform_vision_detected;
        value["platform_x"] = input.platform_x;
        value["platform_y"] = input.platform_y;
        value["platform_z"] = input.platform_z;
        value["platform_vx"] = input.platform_vx;
        value["platform_vy"] = input.platform_vy;
        value["raw_detection_found"] = latest_detection_found_;
        value["raw_detection_predicted"] = latest_detection_predicted_;
        value["raw_detection_confidence"] = latest_detection_confidence_;
        value["raw_detection_receive_age_s"] =
            sourceAge(now_s, last_detection_received_s_);
        value["raw_detection_measurement_age_s"] =
            latest_detection_measurement_age_s_;
        value["raw_detection_fresh"] =
            sourceFresh(now_s, last_detection_received_s_);

        value["pixel_valid"] = input.pixel_valid;
        value["pixel_aligned"] = input.pixel_aligned;
        value["pixel_world_vx"] = input.pixel_world_vx;
        value["pixel_world_vy"] = input.pixel_world_vy;
        value["apriltag_detected"] = latest_apriltag_detected_;
        value["apriltag_age_s"] =
            sourceAge(now_s, last_apriltag_message_received_s_);
        value["apriltag_fresh"] =
            sourceFresh(now_s, last_apriltag_message_received_s_);
        value["apriltag_pose_valid"] = latest_apriltag_pose_valid_;
        value["apriltag_range_valid"] = input.apriltag_range_valid;
        value["apriltag_center"] = input.apriltag_center_tag_visible;
        value["apriltag_plane_distance_m"] =
            diagnosticNumber(input.apriltag_plane_distance_m);
        value["apriltag_visible_ids"] = idArray(latest_apriltag_visible_ids_);
        value["apriltag_used_ids"] = idArray(latest_apriltag_used_ids_);

        value["descent_allowed"] = input.descent_allowed;
        value["descent_allowed_age_s"] =
            sourceAge(now_s, last_descent_allowed_received_s_);
        value["descent_allowed_fresh"] =
            sourceFresh(now_s, last_descent_allowed_received_s_);
        value["safety_hold"] = input.safety_hold;
        value["safety_hold_age_s"] =
            sourceAge(now_s, last_safety_hold_received_s_);
        value["safety_hold_fresh"] =
            sourceFresh(now_s, last_safety_hold_received_s_);
        value["distance_to_d_m"] = diagnosticNumber(input.distance_to_d_m);
        value["car_pose_meta_age_s"] =
            sourceAge(now_s, last_car_pose_meta_received_s_);
        value["car_pose_meta_fresh"] =
            sourceFresh(now_s, last_car_pose_meta_received_s_);
        value["setpoint_valid"] = command.setpoint_valid;
        value["target_x"] = command.target_x;
        value["target_y"] = command.target_y;
        value["target_z"] = command.target_z;
        value["target_vx"] = command.target_vx;
        value["target_vy"] = command.target_vy;
        value["target_vz"] = command.target_vz;
        value["override_request"] = command.override_mode_request;
        value["fault_code"] = command.fault_code;
        value["fault_text"] = command.fault_text;
        ROS_INFO_STREAM("[d_task_mission][DIAG] " << writeJson(value));
    }

    void publishTaskState() {
        Json::Value state;
        state["mission_id"] = controller_.missionId();
        state["mode"] = missionModeName(controller_.mode());
        state["state"] = missionStateName(controller_.state());
        state["retry_count"] = controller_.retryCount();
        state["payload_hardware_enabled"] = payload_actuator_.enabled();
        std_msgs::String message;
        message.data = writeJson(state);
        task_state_publisher_.publish(message);
    }

    void publishEvent(const std::string& event,
                      const Json::Value& details = Json::Value(Json::objectValue)) {
        Json::Value value = details;
        value["event"] = event;
        value["mission_id"] = controller_.missionId();
        std_msgs::String message;
        message.data = writeJson(value);
        event_publisher_.publish(message);
        ROS_INFO_STREAM("[d_task_mission][EVENT] " << message.data);
    }

    void publishFault(int code, const std::string& text,
                      const std::string& detail) {
        const std::string key = std::to_string(code) + ":" + text + ":" + detail;
        if (key == last_fault_key_) {
            return;
        }
        last_fault_key_ = key;
        Json::Value value;
        value["fault_code"] = code;
        value["severity"] = "ERROR";
        value["fault_text"] = text;
        if (!detail.empty()) {
            value["detail"] = detail;
        }
        std_msgs::String message;
        message.data = writeJson(value);
        fault_publisher_.publish(message);
        ROS_ERROR_STREAM("[d_task_mission][FAULT] " << message.data);
    }

    void publishHealth(double now_s) {
        Json::Value value;
        value["task_node_ok"] = true;
        value["bridge_connected"] = ego_api_->isConnected();
        value["mission_odom_fresh"] = has_odom_
            && now_s - last_odom_received_s_ <= odom_timeout_s_;
        value["platform_estimate_fresh"] = has_platform_
            && now_s - last_platform_received_s_
                <= platform_estimate_timeout_s_;
        value["apriltag_range_fresh"] = has_apriltag_range_
            && now_s - last_apriltag_range_received_s_
                <= apriltag_range_timeout_s_;
        value["apriltag_plane_distance_m"] =
            has_apriltag_range_ ? apriltag_plane_distance_m_ : -1.0;
        value["payload_hardware_enabled"] = payload_actuator_.enabled();
        std_msgs::String message;
        message.data = writeJson(value);
        health_publisher_.publish(message);
    }

    void publishTerminal(const std::string& final_state) {
        Json::Value reset;
        reset["mission_id"] = controller_.missionId();
        reset["final_state"] = final_state;
        std_msgs::String message;
        message.data = writeJson(reset);
        mission_reset_publisher_.publish(message);
        Json::Value details;
        details["final_state"] = final_state;
        publishEvent("MISSION_CONTROLLER_FINISHED", details);
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    MissionControllerConfig controller_config_;
    MissionController controller_;
    PixelServoConfig pixel_servo_config_;
    PixelServo pixel_servo_;
    PayloadActuator payload_actuator_;
    std::unique_ptr<EgoApi> ego_api_;

    ros::Subscriber mission_config_subscriber_;
    ros::Subscriber mission_start_subscriber_;
    ros::Subscriber positioning_subscriber_;
    ros::Subscriber platform_subscriber_;
    ros::Subscriber detection_subscriber_;
    ros::Subscriber apriltag_range_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber bridge_state_subscriber_;
    ros::Subscriber control_mode_subscriber_;
    ros::Subscriber descent_allowed_subscriber_;
    ros::Subscriber safety_hold_subscriber_;
    ros::Subscriber car_pose_meta_subscriber_;
    ros::Publisher task_state_publisher_;
    ros::Publisher event_publisher_;
    ros::Publisher fault_publisher_;
    ros::Publisher health_publisher_;
    ros::Publisher mission_reset_publisher_;
    ros::Timer control_timer_;

    nav_msgs::Odometry latest_odom_;
    PlatformEstimate latest_platform_;
    bool has_odom_ = false;
    bool has_platform_ = false;
    bool has_apriltag_range_ = false;
    bool latest_detection_found_ = false;
    bool latest_detection_predicted_ = false;
    float latest_detection_confidence_ = 0.0F;
    float latest_detection_measurement_age_s_ = 0.0F;
    double last_detection_received_s_ = -1.0;
    bool latest_apriltag_detected_ = false;
    bool latest_apriltag_pose_valid_ = false;
    std::vector<int32_t> latest_apriltag_visible_ids_;
    std::vector<int32_t> latest_apriltag_used_ids_;
    double last_apriltag_message_received_s_ = -1.0;
    double last_odom_received_s_ = -1.0;
    double last_platform_received_s_ = -1.0;
    double last_apriltag_range_received_s_ = -1.0;
    double last_apriltag_center_received_s_ = -1.0;
    double apriltag_plane_distance_m_ =
        std::numeric_limits<double>::infinity();
    bool apriltag_center_tag_visible_ = false;
    std::string bridge_state_;
    double last_bridge_state_received_s_ = -1.0;
    uint8_t control_mode_ = 0;
    double last_control_mode_received_s_ = -1.0;
    bool descent_allowed_ = false;
    double last_descent_allowed_received_s_ = -1.0;
    bool safety_hold_ = false;
    double last_safety_hold_received_s_ = -1.0;
    double distance_to_d_m_ = std::numeric_limits<double>::infinity();
    double last_car_pose_meta_received_s_ = -1.0;
    double odom_timeout_s_ = 0.30;
    double platform_estimate_timeout_s_ = 0.35;
    double apriltag_range_timeout_s_ = 0.35;
    double control_rate_hz_ = 30.0;
    double diagnostic_log_rate_hz_ = 1.0;
    double diagnostic_source_timeout_s_ = 1.0;
    double last_diagnostic_log_s_ = -1e9;
    double last_health_publish_s_ = -1e9;
    bool terminal_published_ = false;
    std::string last_fault_key_;
};

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "d_task_mission");
    try {
        d_task_uav_control::DTaskMissionNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("[d_task_mission] startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
