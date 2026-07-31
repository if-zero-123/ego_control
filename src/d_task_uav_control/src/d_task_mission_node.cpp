#include <algorithm>
#include <cerrno>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <json/json.h>
#include <mavros_msgs/ActuatorControl.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include "ego_api/ego_api.h"
#include "d_task_uav_control/fixed_height_drop_flow.h"
#include "d_task_uav_control/payload_pulse.h"

namespace d_task_uav_control {
namespace {

double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

constexpr double kPi = 3.14159265358979323846;

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
    return std::atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z));
}

bool parseJson(const std::string& raw, Json::Value& value) {
    Json::CharReaderBuilder builder;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    std::string error;
    return reader->parse(raw.data(), raw.data() + raw.size(), &value, &error)
        && value.isObject();
}

std::string writeJson(const Json::Value& value) {
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    return Json::writeString(builder, value);
}

class PayloadActuator {
public:
    PayloadActuator(ros::NodeHandle& node, ros::NodeHandle& private_node)
        : pulse_(loadPulseDuration(private_node)) {
        private_node.param("payload/enabled", enabled_, false);
        private_node.param("payload/backend", backend_, std::string("gpio"));
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
                throw std::runtime_error(
                    "payload/gpio_wiringpi_pin must be non-negative");
            }
            if (!setGpioOutputMode() || !writeGpio(false, true)) {
                throw std::runtime_error(
                    "failed to initialize payload GPIO through /usr/bin/gpio");
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

    bool trigger(double now_s) { return pulse_.trigger(now_s); }

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
                ROS_ERROR("[tag0_follow] failed to reset payload GPIO");
            }
        } else if (enabled_ && command == PayloadPulseCommand::NEUTRAL) {
            publish(neutral_value_);
        }
    }

    bool enabled() const { return enabled_; }

private:
    static double loadPulseDuration(ros::NodeHandle& private_node) {
        double duration_s = 5.0;
        private_node.param("payload/pulse_duration_s", duration_s, 5.0);
        if (!std::isfinite(duration_s) || duration_s < 0.0) {
            throw std::runtime_error(
                "payload/pulse_duration_s must be finite and non-negative");
        }
        return duration_s;
    }

    void writeRelease() {
        if (backend_ == "gpio") {
            if (!writeGpio(true)) {
                ROS_ERROR("[tag0_follow] failed to set payload GPIO high");
            }
        } else {
            publish(release_value_);
        }
    }

    void writeNeutral() {
        if (backend_ == "gpio") {
            if (!writeGpio(false)) {
                ROS_ERROR("[tag0_follow] failed to set payload GPIO low");
            }
        } else {
            publish(neutral_value_);
        }
    }

    bool setGpioOutputMode() const {
        return runGpioCommand(
            "mode", std::to_string(gpio_wiringpi_pin_), "out");
    }

    bool writeGpio(bool release, bool force = false) {
        const bool high_level = gpio_active_high_ ? release : !release;
        if (!force && gpio_level_known_ && gpio_level_high_ == high_level) {
            return true;
        }
        if (!runGpioCommand("write", std::to_string(gpio_wiringpi_pin_),
                            high_level ? "1" : "0")) {
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
    PayloadPulse pulse_;
    bool enabled_ = false;
    std::string backend_ = "gpio";
    int group_mix_ = 2;
    int channel_ = 0;
    double release_value_ = 1.0;
    double neutral_value_ = 0.0;
    int gpio_wiringpi_pin_ = 9;
    bool gpio_active_high_ = true;
    bool gpio_level_known_ = false;
    bool gpio_level_high_ = false;
};

class Tag0FixedHeightFollowNode {
public:
    Tag0FixedHeightFollowNode()
        : private_node_("~"), image_transport_(node_),
          flow_(loadFlowConfig()), payload_actuator_(node_, private_node_) {
        loadParameters();
        dictionary_ = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_APRILTAG_36h11);
        detector_parameters_ = cv::aruco::DetectorParameters::create();
        detector_parameters_->cornerRefinementMethod =
            cv::aruco::CORNER_REFINE_SUBPIX;
        detector_parameters_->minMarkerPerimeterRate = 0.02;
        detector_parameters_->aprilTagQuadDecimate = 1.0F;

        ego_api_.reset(new EgoApi(node_, bridge_namespace_));
        subscribeTopics();
        advertiseTopics();
        control_timer_ = node_.createTimer(
            ros::Duration(1.0 / control_rate_hz_),
            &Tag0FixedHeightFollowNode::controlTimerCallback, this);
        ROS_INFO("[tag0_follow] ready: tag=%d fixed_height=%.2fm PID=(%.3f, %.3f, %.3f)",
                 tag_id_, flight_height_m_, kp_, ki_, kd_);
    }

private:
    FixedHeightDropFlowConfig loadFlowConfig() {
        FixedHeightDropFlowConfig config;
        private_node_.param("simple_follow/drop_alignment_stable_s",
                            config.alignment_stable_s, 1.0);
        private_node_.param("payload/pulse_duration_s",
                            config.release_duration_s, 5.0);
        private_node_.param("simple_follow/release_settle_s",
                            config.release_settle_s, 0.25);
        private_node_.param("simple_follow/home_stable_s",
                            config.home_stable_s, 0.50);
        return config;
    }

    void loadParameters() {
        private_node_.param<std::string>("mission/bridge_namespace",
                                         bridge_namespace_, "/ego_bridge");
        private_node_.param("simple_follow/flight_height_m", flight_height_m_, 1.50);
        private_node_.param("simple_follow/control_rate_hz", control_rate_hz_, 30.0);
        private_node_.param("simple_follow/odom_timeout_s", odom_timeout_s_, 0.30);
        private_node_.param("simple_follow/tag_timeout_s", tag_timeout_s_, 0.20);
        private_node_.param("simple_follow/takeoff_retry_s", takeoff_retry_s_, 0.50);
        private_node_.param("simple_follow/landing_retry_s", landing_retry_s_, 0.50);
        private_node_.param("simple_follow/min_tag_side_px", min_tag_side_px_, 8.0);
        private_node_.param("simple_follow/tag_id", tag_id_, 0);
        private_node_.param("simple_follow/target_offset_u_px",
                            target_offset_u_px_, 0.0);
        private_node_.param("simple_follow/target_offset_v_px",
                            target_offset_v_px_, 0.0);
        private_node_.param("simple_follow/initial_offset_distance_m",
                            initial_offset_distance_m_, 0.50);
        private_node_.param("simple_follow/initial_offset_clockwise_deg",
                            initial_offset_clockwise_deg_, 30.0);
        private_node_.param("simple_follow/initial_offset_reach_radius_m",
                            initial_offset_reach_radius_m_, 0.08);
        private_node_.param("simple_follow/forward_search_distance_m",
                            forward_search_distance_m_, 1.00);
        private_node_.param("simple_follow/forward_search_reach_radius_m",
                            forward_search_reach_radius_m_, 0.08);
        private_node_.param("simple_follow/drop_alignment_tolerance",
                            drop_alignment_tolerance_, 0.08);
        private_node_.param("simple_follow/height_tolerance_m",
                            height_tolerance_m_, 0.10);
        private_node_.param("simple_follow/home_xy_tolerance_m",
                            home_xy_tolerance_m_, 0.20);
        private_node_.param("simple_follow/pid/kp", kp_, 0.35);
        private_node_.param("simple_follow/pid/ki", ki_, 0.0);
        private_node_.param("simple_follow/pid/kd", kd_, 0.0);
        private_node_.param("simple_follow/pid/deadband", deadband_, 0.04);
        private_node_.param("simple_follow/pid/integral_limit", integral_limit_, 0.50);
        private_node_.param("simple_follow/pid/max_speed_mps", max_speed_mps_, 0.35);
        if (flight_height_m_ <= 0.0 || control_rate_hz_ < 2.0
            || odom_timeout_s_ <= 0.0 || tag_timeout_s_ <= 0.0
            || takeoff_retry_s_ <= 0.0 || landing_retry_s_ <= 0.0
            || min_tag_side_px_ <= 0.0 || initial_offset_distance_m_ < 0.0
            || initial_offset_reach_radius_m_ <= 0.0
            || forward_search_distance_m_ < 0.0
            || forward_search_reach_radius_m_ <= 0.0
            || drop_alignment_tolerance_ < 0.0
            || height_tolerance_m_ <= 0.0
            || home_xy_tolerance_m_ <= 0.0
            || !std::isfinite(target_offset_u_px_)
            || !std::isfinite(target_offset_v_px_)
            || tag_id_ < 0 || kp_ < 0.0 || ki_ < 0.0 || kd_ < 0.0
            || deadband_ < 0.0 || integral_limit_ < 0.0
            || max_speed_mps_ <= 0.0) {
            throw std::runtime_error("invalid simple_follow configuration");
        }
    }

    std::string topic(const std::string& key, const std::string& fallback) {
        return private_node_.param<std::string>("topics/" + key, fallback);
    }

    void subscribeTopics() {
        mission_config_subscriber_ = node_.subscribe(
            topic("mission_config", "/uav_protocol/mission_config"), 2,
            &Tag0FixedHeightFollowNode::missionConfigCallback, this);
        mission_start_subscriber_ = node_.subscribe(
            topic("mission_start", "/uav_protocol/mission_start"), 2,
            &Tag0FixedHeightFollowNode::missionStartCallback, this);
        car_event_subscriber_ = node_.subscribe(
            topic("car_event", "/uav_protocol/car/event"), 5,
            &Tag0FixedHeightFollowNode::carEventCallback, this);
        positioning_subscriber_ = node_.subscribe(
            topic("positioning_status", "/d_task/positioning/status"), 2,
            &Tag0FixedHeightFollowNode::positioningStatusCallback, this);
        odom_subscriber_ = node_.subscribe(
            topic("px4_odom", "/mavros/local_position/odom"), 5,
            &Tag0FixedHeightFollowNode::odomCallback, this);
        bridge_state_subscriber_ = node_.subscribe(
            topic("bridge_state", "/ego_bridge/flight_state"), 5,
            &Tag0FixedHeightFollowNode::bridgeStateCallback, this);
        control_mode_subscriber_ = node_.subscribe(
            topic("control_mode", "/ego_bridge/control_mode"), 5,
            &Tag0FixedHeightFollowNode::controlModeCallback, this);
        image_subscriber_ = image_transport_.subscribe(
            topic("image", "/usb_camera_vision/usb_cam/image_raw"), 1,
            &Tag0FixedHeightFollowNode::imageCallback, this);
        camera_info_subscriber_ = node_.subscribe(
            topic("camera_info", "/usb_camera_vision/usb_cam/camera_info"), 1,
            &Tag0FixedHeightFollowNode::cameraInfoCallback, this);
    }

    void advertiseTopics() {
        task_state_publisher_ = node_.advertise<std_msgs::String>(
            topic("task_state", "/uav_protocol/task_state"), 2, true);
        mission_reset_publisher_ = node_.advertise<std_msgs::String>(
            topic("mission_reset", "/uav_protocol/mission_reset"), 2);
        event_publisher_ = node_.advertise<std_msgs::String>(
            topic("local_event", "/uav_protocol/local_event"), 5);
        fault_publisher_ = node_.advertise<std_msgs::String>(
            topic("local_fault", "/uav_protocol/local_fault"), 5);
        health_publisher_ = node_.advertise<std_msgs::String>(
            topic("local_health", "/uav_protocol/local_health"), 2);
        vision_health_publisher_ = node_.advertise<std_msgs::Bool>(
            topic("vision_health", "/d_task/vision/health"), 1, true);
        tag_center_publisher_ = node_.advertise<geometry_msgs::PointStamped>(
            topic("tag_center", "/d_task/vision/tag0_center"), 2);
        debug_publisher_ = node_.advertise<std_msgs::String>(
            topic("debug", "/d_task/simple_follow/debug"), 2);
        tracking_publisher_ = node_.advertise<std_msgs::String>(
            topic("local_tracking", "/uav_protocol/local_tracking"), 2);
    }

    void missionConfigCallback(const std_msgs::String::ConstPtr& message) {
        if (message->data.empty()) {
            mission_id_.clear();
            mode_ = "DROP";
            positioning_ready_ = false;
            started_ = false;
            abort_requested_ = false;
            terminal_published_ = false;
            flow_.reset();
            payload_actuator_.reset();
            resetPid();
            publishTaskState();
            return;
        }
        Json::Value root;
        if (!parseJson(message->data, root)
            || root.get("type", "").asString() != "mission_config"
            || root.get("sender", "").asString() != "car"
            || root.get("mission_id", "").asString().empty()
            || !root["payload"].isObject()
            || root["payload"].get("mode", "").asString() != "DROP") {
            publishFault(2301, "invalid_mission_config");
            return;
        }
        mission_id_ = root.get("mission_id", "").asString();
        mode_ = "DROP";
        positioning_ready_ = false;
        started_ = false;
        abort_requested_ = false;
        terminal_published_ = false;
        last_land_request_s_ = -1e9;
        flow_.configure();
        payload_actuator_.reset();
        resetPid();
        publishEvent("MISSION_CONTROLLER_CONFIGURED");
        publishTaskState();
    }

    void positioningStatusCallback(const std_msgs::String::ConstPtr& message) {
        Json::Value root;
        if (!parseJson(message->data, root)
            || !root.get("ready", false).asBool()
            || root.get("mission_id", "").asString() != mission_id_) {
            return;
        }
        home_x_ = root.get("home_x", 0.0).asDouble();
        home_y_ = root.get("home_y", 0.0).asDouble();
        home_z_ = root.get("home_z", 0.0).asDouble();
        if (has_odom_) {
            home_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        }
        if (!flow_.markPositioningReady()) {
            return;
        }
        positioning_ready_ = true;
        publishEvent("MISSION_CONTROLLER_READY");
        publishTaskState();
    }

    void missionStartCallback(const std_msgs::String::ConstPtr& message) {
        Json::Value root;
        const bool valid = parseJson(message->data, root)
            && root.get("type", "").asString() == "mission_start"
            && root.get("sender", "").asString() == "car"
            && root.get("mission_id", "").asString() == mission_id_
            && root["payload"].isObject();
        const std::string reason = valid
            ? root["payload"].get("start_reason", "").asString() : "";
        if (!valid || !positioning_ready_ || started_
            || (reason != "car_button" && reason != "ground_web")) {
            publishFault(2302, "mission_start_rejected");
            return;
        }
        const double now_s = ros::Time::now().toSec();
        if (!flow_.start(now_s)) {
            publishFault(2302, "mission_start_rejected");
            return;
        }
        started_ = true;
        abort_requested_ = false;
        terminal_published_ = false;
        last_takeoff_request_s_ = -1e9;
        publishEvent("MISSION_CONTROLLER_STARTED");
        publishTaskState();
    }

    void carEventCallback(const std_msgs::String::ConstPtr& message) {
        Json::Value root;
        if (!parseJson(message->data, root)
            || root.get("type", "").asString() != "event"
            || root.get("sender", "").asString() != "car"
            || root.get("mission_id", "").asString() != mission_id_
            || !root["payload"].isObject()
            || root["payload"].get("event", "").asString()
                != "MISSION_ABORT") {
            return;
        }
        abort_requested_ = true;
        publishEvent("MISSION_ABORT_RECEIVED");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
        latest_odom_ = *message;
        has_odom_ = true;
        last_odom_s_ = ros::Time::now().toSec();
    }

    void bridgeStateCallback(const std_msgs::String::ConstPtr& message) {
        bridge_state_ = message->data;
    }

    void controlModeCallback(const std_msgs::UInt8::ConstPtr& message) {
        control_mode_ = message->data;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& message) {
        const double fx = message->K[0];
        const double fy = message->K[4];
        const double cx = message->K[2];
        const double cy = message->K[5];
        if (!std::isfinite(fx) || !std::isfinite(fy)
            || !std::isfinite(cx) || !std::isfinite(cy)
            || fx <= 0.0 || fy <= 0.0
            || message->width == 0U || message->height == 0U
            || !std::all_of(message->D.begin(), message->D.end(),
                            [](double value) { return std::isfinite(value); })) {
            ROS_WARN_THROTTLE(2.0, "[tag0_follow] invalid CameraInfo; using image centre");
            return;
        }
        camera_matrix_ = (cv::Mat_<double>(3, 3)
            << fx, 0.0, cx,
               0.0, fy, cy,
               0.0, 0.0, 1.0);
        distortion_coefficients_ = message->D.empty()
            ? cv::Mat() : cv::Mat(message->D).clone();
        camera_info_width_ = message->width;
        camera_info_height_ = message->height;
        camera_info_ready_ = true;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& message) {
        try {
            const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(
                message, sensor_msgs::image_encodings::BGR8);
            cv::Mat gray;
            cv::cvtColor(image->image, gray, cv::COLOR_BGR2GRAY);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<std::vector<cv::Point2f>> rejected;
            cv::aruco::detectMarkers(
                gray, dictionary_, corners, ids, detector_parameters_, rejected);

            tag_found_ = false;
            for (std::size_t index = 0; index < ids.size(); ++index) {
                if (ids[index] != tag_id_ || corners[index].size() != 4U) {
                    continue;
                }
                double minimum_side = std::numeric_limits<double>::infinity();
                cv::Point2f centre(0.0F, 0.0F);
                for (std::size_t corner = 0; corner < 4U; ++corner) {
                    centre += corners[index][corner];
                    minimum_side = std::min(
                        minimum_side,
                        cv::norm(corners[index][(corner + 1U) % 4U]
                                 - corners[index][corner]));
                }
                if (minimum_side < min_tag_side_px_) {
                    continue;
                }
                tag_found_ = true;
                tag_center_ = centre * 0.25F;
                image_width_ = message->width;
                image_height_ = message->height;
                servoPixels(servo_observed_u_, servo_observed_v_,
                            servo_target_u_, servo_target_v_);
                tag_confidence_ = std::min(1.0, minimum_side / 40.0);
                last_tag_s_ = ros::Time::now().toSec();
                geometry_msgs::PointStamped output;
                output.header = message->header;
                output.point.x = tag_center_.x;
                output.point.y = tag_center_.y;
                output.point.z = tag_confidence_;
                tag_center_publisher_.publish(output);
                break;
            }
        } catch (const cv_bridge::Exception& error) {
            ROS_WARN_THROTTLE(2.0, "[tag0_follow] image conversion failed: %s", error.what());
            tag_found_ = false;
        }
    }

    bool odomFresh(double now_s) const {
        return has_odom_ && now_s - last_odom_s_ <= odom_timeout_s_;
    }

    bool tagFresh(double now_s) const {
        return tag_found_ && image_width_ > 0U && image_height_ > 0U
            && now_s - last_tag_s_ <= tag_timeout_s_;
    }

    static double deadband(double value, double width) {
        return std::abs(value) <= width ? 0.0 : value;
    }

    void servoPixels(double& observed_u, double& observed_v,
                     double& target_u, double& target_v) const {
        observed_u = tag_center_.x;
        observed_v = tag_center_.y;
        target_u = 0.5 * static_cast<double>(image_width_) + target_offset_u_px_;
        target_v = 0.5 * static_cast<double>(image_height_) + target_offset_v_px_;
        if (!camera_info_ready_ || camera_info_width_ == 0U
            || camera_info_height_ == 0U) {
            return;
        }

        const double scale_x = static_cast<double>(image_width_)
            / static_cast<double>(camera_info_width_);
        const double scale_y = static_cast<double>(image_height_)
            / static_cast<double>(camera_info_height_);
        cv::Mat scaled_camera_matrix = camera_matrix_.clone();
        scaled_camera_matrix.at<double>(0, 0) *= scale_x;
        scaled_camera_matrix.at<double>(0, 2) *= scale_x;
        scaled_camera_matrix.at<double>(1, 1) *= scale_y;
        scaled_camera_matrix.at<double>(1, 2) *= scale_y;

        std::vector<cv::Point2f> source{tag_center_};
        std::vector<cv::Point2f> rectified;
        try {
            cv::undistortPoints(source, rectified, scaled_camera_matrix,
                                distortion_coefficients_, cv::noArray(),
                                scaled_camera_matrix);
        } catch (const cv::Exception& error) {
            ROS_WARN_THROTTLE(2.0, "[tag0_follow] undistort failed: %s", error.what());
            return;
        }
        if (rectified.size() == 1U) {
            observed_u = rectified.front().x;
            observed_v = rectified.front().y;
            target_u = scaled_camera_matrix.at<double>(0, 2) + target_offset_u_px_;
            target_v = scaled_camera_matrix.at<double>(1, 2) + target_offset_v_px_;
        }
    }

    void resetPid() {
        integral_forward_ = 0.0;
        integral_left_ = 0.0;
        previous_forward_ = 0.0;
        previous_left_ = 0.0;
        previous_pid_s_ = -1.0;
    }

    void computeVelocity(double now_s, double yaw,
                         double& world_vx, double& world_vy,
                         double& forward_error, double& left_error) {
        world_vx = 0.0;
        world_vy = 0.0;
        forward_error = 0.0;
        left_error = 0.0;
        if (!tagFresh(now_s)) {
            resetPid();
            return;
        }

        servoPixels(servo_observed_u_, servo_observed_v_,
                    servo_target_u_, servo_target_v_);
        const double normal_u = 2.0 * (servo_observed_u_ - servo_target_u_)
            / static_cast<double>(image_width_);
        const double normal_v = 2.0 * (servo_observed_v_ - servo_target_v_)
            / static_cast<double>(image_height_);
        forward_error = deadband(-normal_v, deadband_);
        left_error = deadband(-normal_u, deadband_);
        const double dt = previous_pid_s_ < 0.0
            ? 1.0 / control_rate_hz_
            : clamp(now_s - previous_pid_s_, 1e-3, 0.20);

        integral_forward_ = clamp(
            integral_forward_ + forward_error * dt,
            -integral_limit_, integral_limit_);
        integral_left_ = clamp(
            integral_left_ + left_error * dt,
            -integral_limit_, integral_limit_);
        const double derivative_forward =
            (forward_error - previous_forward_) / dt;
        const double derivative_left = (left_error - previous_left_) / dt;
        double body_vx = kp_ * forward_error + ki_ * integral_forward_
            + kd_ * derivative_forward;
        double body_vy = kp_ * left_error + ki_ * integral_left_
            + kd_ * derivative_left;
        const double speed = std::hypot(body_vx, body_vy);
        if (speed > max_speed_mps_) {
            const double scale = max_speed_mps_ / speed;
            body_vx *= scale;
            body_vy *= scale;
        }
        const double cosine = std::cos(yaw);
        const double sine = std::sin(yaw);
        world_vx = cosine * body_vx - sine * body_vy;
        world_vy = sine * body_vx + cosine * body_vy;
        previous_forward_ = forward_error;
        previous_left_ = left_error;
        previous_pid_s_ = now_s;
    }

    bool requestOverrideIfNeeded() {
        if (control_mode_ == 1U) {
            return true;
        }
        ego_api_->requestOverrideMode(true);
        return false;
    }

    void sendFixedHeightCommand(double x, double y, double yaw,
                                double vx = 0.0, double vy = 0.0) {
        ego_api_->sendPositionVelocityCmd(
            x, y, home_z_ + flight_height_m_, vx, vy, 0.0, yaw);
    }

    void prepareInitialOffset() {
        home_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        const double heading = home_yaw_
            - initial_offset_clockwise_deg_ * kPi / 180.0;
        offset_target_x_ = latest_odom_.pose.pose.position.x
            + initial_offset_distance_m_ * std::cos(heading);
        offset_target_y_ = latest_odom_.pose.pose.position.y
            + initial_offset_distance_m_ * std::sin(heading);
        resetPid();
    }

    void prepareForwardSearch() {
        forward_target_x_ = offset_target_x_
            + forward_search_distance_m_ * std::cos(home_yaw_);
        forward_target_y_ = offset_target_y_
            + forward_search_distance_m_ * std::sin(home_yaw_);
        resetPid();
    }

    void prepareReleaseHold() {
        release_x_ = latest_odom_.pose.pose.position.x;
        release_y_ = latest_odom_.pose.pose.position.y;
        release_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        resetPid();
    }

    void requestLanding(double now_s) {
        ego_api_->requestLand();
        last_land_request_s_ = now_s;
    }

    bool alignedForDrop(double now_s, double forward_error,
                        double left_error) const {
        return tagFresh(now_s)
            && std::abs(forward_error) <= drop_alignment_tolerance_
            && std::abs(left_error) <= drop_alignment_tolerance_
            && std::abs(latest_odom_.pose.pose.position.z
                        - (home_z_ + flight_height_m_))
                <= height_tolerance_m_;
    }

    void handleStateChange(const FixedHeightDropFlowOutput& output,
                           double now_s) {
        if (!output.state_changed) {
            return;
        }
        Json::Value details;
        details["previous_state"] = fixedHeightDropStateName(output.previous_state);
        details["state"] = fixedHeightDropStateName(output.state);
        publishEvent("MISSION_STATE_CHANGED", details);
        switch (output.state) {
            case FixedHeightDropState::MOVE_TO_SEARCH_START:
                prepareInitialOffset();
                publishEvent("INITIAL_OFFSET_STARTED");
                break;
            case FixedHeightDropState::FORWARD_SEARCH:
                prepareForwardSearch();
                publishEvent("FORWARD_SEARCH_STARTED");
                break;
            case FixedHeightDropState::FOLLOW_CAR:
                resetPid();
                publishEvent("TAG_FOLLOW_STARTED");
                break;
            case FixedHeightDropState::RELEASE: {
                prepareReleaseHold();
                Json::Value release;
                release["hardware_enabled"] = payload_actuator_.enabled();
                if (output.trigger_payload && payload_actuator_.trigger(now_s)) {
                    publishEvent(payload_actuator_.enabled()
                                     ? "PAYLOAD_RELEASE_TRIGGERED"
                                     : "PAYLOAD_RELEASE_DRY_RUN",
                                 release);
                }
                break;
            }
            case FixedHeightDropState::RETURN_HOME:
                resetPid();
                if (flow_.finalAbort()) {
                    payload_actuator_.reset();
                } else if (output.previous_state
                           == FixedHeightDropState::RELEASE) {
                    publishEvent("PAYLOAD_RELEASE_COMPLETE");
                    publishEvent("CAR_SPEEDUP_REQUESTED");
                }
                publishEvent("RETURN_HOME_STARTED");
                break;
            case FixedHeightDropState::LAND_HOME:
                publishEvent("LAND_HOME_REQUESTED");
                requestLanding(now_s);
                break;
            case FixedHeightDropState::COMPLETE:
                publishEvent("MISSION_CONTROLLER_COMPLETE");
                break;
            case FixedHeightDropState::ABORT:
                publishEvent("MISSION_CONTROLLER_ABORT");
                break;
            case FixedHeightDropState::NOT_READY:
            case FixedHeightDropState::POSITIONING_INIT:
            case FixedHeightDropState::WAIT_START:
            case FixedHeightDropState::TAKEOFF:
                break;
        }
    }

    void controlTimerCallback(const ros::TimerEvent&) {
        const double now_s = ros::Time::now().toSec();
        double command_vx = 0.0;
        double command_vy = 0.0;
        double forward_error = 0.0;
        double left_error = 0.0;
        const bool odom_fresh = odomFresh(now_s);
        const FixedHeightDropState previous_state = flow_.state();
        if (started_ && previous_state == FixedHeightDropState::FOLLOW_CAR
            && odom_fresh) {
            const double yaw = yawFromQuaternion(latest_odom_.pose.pose.orientation);
            computeVelocity(now_s, yaw, command_vx, command_vy,
                            forward_error, left_error);
        }

        FixedHeightDropFlowInput input;
        input.now_s = now_s;
        input.takeoff_complete = bridge_state_ == "HOVER" && odom_fresh;
        input.tag_detected = tagFresh(now_s);
        input.aligned = odom_fresh
            && alignedForDrop(now_s, forward_error, left_error);
        input.landed = bridge_state_ == "IDLE";
        input.abort_requested = abort_requested_;
        if (odom_fresh) {
            input.offset_reached = std::hypot(
                offset_target_x_ - latest_odom_.pose.pose.position.x,
                offset_target_y_ - latest_odom_.pose.pose.position.y)
                <= initial_offset_reach_radius_m_;
            input.search_endpoint_reached = std::hypot(
                forward_target_x_ - latest_odom_.pose.pose.position.x,
                forward_target_y_ - latest_odom_.pose.pose.position.y)
                <= forward_search_reach_radius_m_;
            input.home_reached = std::hypot(
                home_x_ - latest_odom_.pose.pose.position.x,
                home_y_ - latest_odom_.pose.pose.position.y)
                <= home_xy_tolerance_m_
                && std::abs(latest_odom_.pose.pose.position.z
                            - (home_z_ + flight_height_m_))
                    <= height_tolerance_m_;
        }

        const FixedHeightDropFlowOutput output = flow_.update(input);
        if (output.fault_code != 0) {
            publishFault(output.fault_code, output.fault_text);
        }
        handleStateChange(output, now_s);
        abort_requested_ = false;

        const FixedHeightDropState state = flow_.state();
        if (started_ && state == FixedHeightDropState::TAKEOFF) {
            if (now_s - last_takeoff_request_s_ >= takeoff_retry_s_) {
                ego_api_->requestTakeoffTo(home_z_ + flight_height_m_);
                last_takeoff_request_s_ = now_s;
            }
        } else if (started_ && !odom_fresh
                   && state != FixedHeightDropState::LAND_HOME
                   && state != FixedHeightDropState::COMPLETE
                   && state != FixedHeightDropState::ABORT) {
            publishFault(2101, "uav_odometry_stale");
        } else if (started_ && odom_fresh
                   && state == FixedHeightDropState::MOVE_TO_SEARCH_START
                   && requestOverrideIfNeeded()) {
            sendFixedHeightCommand(offset_target_x_, offset_target_y_, home_yaw_);
        } else if (started_ && odom_fresh
                   && state == FixedHeightDropState::FORWARD_SEARCH
                   && requestOverrideIfNeeded()) {
            sendFixedHeightCommand(forward_target_x_, forward_target_y_, home_yaw_);
        } else if (started_ && odom_fresh
                   && state == FixedHeightDropState::FOLLOW_CAR
                   && requestOverrideIfNeeded()) {
            const double yaw = yawFromQuaternion(latest_odom_.pose.pose.orientation);
            sendFixedHeightCommand(latest_odom_.pose.pose.position.x,
                                   latest_odom_.pose.pose.position.y,
                                   yaw, command_vx, command_vy);
        } else if (started_ && odom_fresh
                   && state == FixedHeightDropState::RELEASE
                   && requestOverrideIfNeeded()) {
            sendFixedHeightCommand(release_x_, release_y_, release_yaw_);
        } else if (started_ && odom_fresh
                   && state == FixedHeightDropState::RETURN_HOME
                   && requestOverrideIfNeeded()) {
            sendFixedHeightCommand(home_x_, home_y_, home_yaw_);
        } else if (started_ && state == FixedHeightDropState::LAND_HOME
                   && bridge_state_ != "IDLE"
                   && now_s - last_land_request_s_ >= landing_retry_s_) {
            requestLanding(now_s);
        }

        payload_actuator_.update(now_s);

        publishTaskState();
        publishHealth(now_s);
        publishTracking(now_s, forward_error, left_error);
        publishDebug(now_s, command_vx, command_vy, forward_error, left_error);
        if (output.terminal && !terminal_published_) {
            publishTerminal(output.aborted ? "ABORT" : "COMPLETE");
            terminal_published_ = true;
            started_ = false;
        }
    }

    void publishTaskState() {
        Json::Value state;
        state["mission_id"] = mission_id_;
        state["mode"] = mode_;
        state["state"] = fixedHeightDropStateName(flow_.state());
        state["payload_hardware_enabled"] = payload_actuator_.enabled();
        std_msgs::String message;
        message.data = writeJson(state);
        task_state_publisher_.publish(message);
    }

    void publishEvent(
        const std::string& event,
        const Json::Value& details = Json::Value(Json::objectValue)) {
        Json::Value value = details;
        value["event"] = event;
        value["mission_id"] = mission_id_;
        std_msgs::String message;
        message.data = writeJson(value);
        event_publisher_.publish(message);
    }

    void publishTerminal(const std::string& final_state) {
        Json::Value reset;
        reset["mission_id"] = mission_id_;
        reset["final_state"] = final_state;
        std_msgs::String message;
        message.data = writeJson(reset);
        mission_reset_publisher_.publish(message);
        Json::Value details;
        details["final_state"] = final_state;
        publishEvent("MISSION_CONTROLLER_FINISHED", details);
    }

    void publishFault(int code, const std::string& text) {
        const std::string key = std::to_string(code) + ":" + text;
        if (key == last_fault_key_) {
            return;
        }
        last_fault_key_ = key;
        Json::Value value;
        value["fault_code"] = code;
        value["severity"] = "ERROR";
        value["fault_text"] = text;
        std_msgs::String message;
        message.data = writeJson(value);
        fault_publisher_.publish(message);
        ROS_ERROR_STREAM("[tag0_follow] " << message.data);
    }

    void publishHealth(double now_s) {
        const bool vision_ok = tagFresh(now_s);
        std_msgs::Bool vision;
        vision.data = vision_ok;
        vision_health_publisher_.publish(vision);
        Json::Value value;
        value["task_node_ok"] = true;
        value["bridge_connected"] = ego_api_->isConnected();
        value["mission_odom_fresh"] = odomFresh(now_s);
        value["tag0_fresh"] = vision_ok;
        value["payload_hardware_enabled"] = payload_actuator_.enabled();
        value["mission_configured"] = !mission_id_.empty();
        std_msgs::String message;
        message.data = writeJson(value);
        health_publisher_.publish(message);
    }

    void publishTracking(double now_s, double forward_error,
                         double left_error) {
        const bool detected = tagFresh(now_s);
        Json::Value value;
        value["track_state"] = detected ? "MEASURED" : "INVALID";
        value["detected"] = detected;
        value["confidence"] = detected ? tag_confidence_ : 0.0;
        value["pixel_center"]["u"] = detected ? tag_center_.x : 0.0;
        value["pixel_center"]["v"] = detected ? tag_center_.y : 0.0;
        value["relative_position"]["x"] = 0.0;
        value["relative_position"]["y"] = 0.0;
        value["relative_position"]["z"] = 0.0;
        value["relative_velocity"]["x"] = 0.0;
        value["relative_velocity"]["y"] = 0.0;
        value["vision_age_ms"] = detected
            ? static_cast<Json::Int64>(std::max(0.0, now_s - last_tag_s_)
                                      * 1000.0)
            : static_cast<Json::Int64>(2147483647);
        value["filter_mode"] = detected ? "MEASURED" : "INVALID";
        value["release_gate"] =
            flow_.state() == FixedHeightDropState::FOLLOW_CAR
            && alignedForDrop(now_s, forward_error, left_error);
        value["landing_gate"] = false;
        std_msgs::String message;
        message.data = writeJson(value);
        tracking_publisher_.publish(message);
    }

    void publishDebug(double now_s, double command_vx, double command_vy,
                      double forward_error, double left_error) {
        Json::Value value;
        value["state"] = fixedHeightDropStateName(flow_.state());
        value["tag0_found"] = tagFresh(now_s);
        value["tag0_u"] = tag_center_.x;
        value["tag0_v"] = tag_center_.y;
        value["servo_observed_u"] = servo_observed_u_;
        value["servo_observed_v"] = servo_observed_v_;
        value["servo_target_u"] = servo_target_u_;
        value["servo_target_v"] = servo_target_v_;
        value["camera_info_ready"] = camera_info_ready_;
        value["forward_error"] = forward_error;
        value["left_error"] = left_error;
        value["command_vx"] = command_vx;
        value["command_vy"] = command_vy;
        value["fixed_height_m"] = home_z_ + flight_height_m_;
        value["initial_offset_target_x"] = offset_target_x_;
        value["initial_offset_target_y"] = offset_target_y_;
        value["forward_search_target_x"] = forward_target_x_;
        value["forward_search_target_y"] = forward_target_y_;
        value["home_x"] = home_x_;
        value["home_y"] = home_y_;
        value["home_z"] = home_z_;
        value["payload_hardware_enabled"] = payload_actuator_.enabled();
        std_msgs::String message;
        message.data = writeJson(value);
        debug_publisher_.publish(message);
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_subscriber_;
    std::unique_ptr<EgoApi> ego_api_;
    FixedHeightDropFlow flow_;
    PayloadActuator payload_actuator_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;

    ros::Subscriber mission_config_subscriber_;
    ros::Subscriber mission_start_subscriber_;
    ros::Subscriber car_event_subscriber_;
    ros::Subscriber positioning_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber bridge_state_subscriber_;
    ros::Subscriber control_mode_subscriber_;
    ros::Subscriber camera_info_subscriber_;
    ros::Publisher task_state_publisher_;
    ros::Publisher mission_reset_publisher_;
    ros::Publisher event_publisher_;
    ros::Publisher fault_publisher_;
    ros::Publisher health_publisher_;
    ros::Publisher vision_health_publisher_;
    ros::Publisher tag_center_publisher_;
    ros::Publisher debug_publisher_;
    ros::Publisher tracking_publisher_;
    ros::Timer control_timer_;

    std::string bridge_namespace_;
    std::string mission_id_;
    std::string mode_ = "DROP";
    std::string bridge_state_;
    std::string last_fault_key_;
    nav_msgs::Odometry latest_odom_;
    cv::Point2f tag_center_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coefficients_;
    bool has_odom_ = false;
    bool tag_found_ = false;
    bool positioning_ready_ = false;
    bool started_ = false;
    bool abort_requested_ = false;
    bool terminal_published_ = false;
    bool camera_info_ready_ = false;
    uint8_t control_mode_ = 0;
    unsigned int image_width_ = 0;
    unsigned int image_height_ = 0;
    unsigned int camera_info_width_ = 0;
    unsigned int camera_info_height_ = 0;
    int tag_id_ = 0;
    double last_odom_s_ = -1.0;
    double last_tag_s_ = -1.0;
    double last_takeoff_request_s_ = -1e9;
    double last_land_request_s_ = -1e9;
    double home_x_ = 0.0;
    double home_y_ = 0.0;
    double home_z_ = 0.0;
    double home_yaw_ = 0.0;
    double offset_target_x_ = 0.0;
    double offset_target_y_ = 0.0;
    double forward_target_x_ = 0.0;
    double forward_target_y_ = 0.0;
    double release_x_ = 0.0;
    double release_y_ = 0.0;
    double release_yaw_ = 0.0;
    double tag_confidence_ = 0.0;
    double target_offset_u_px_ = 0.0;
    double target_offset_v_px_ = 0.0;
    double servo_observed_u_ = 0.0;
    double servo_observed_v_ = 0.0;
    double servo_target_u_ = 0.0;
    double servo_target_v_ = 0.0;

    double flight_height_m_ = 1.50;
    double control_rate_hz_ = 30.0;
    double odom_timeout_s_ = 0.30;
    double tag_timeout_s_ = 0.20;
    double takeoff_retry_s_ = 0.50;
    double landing_retry_s_ = 0.50;
    double min_tag_side_px_ = 8.0;
    double initial_offset_distance_m_ = 0.50;
    double initial_offset_clockwise_deg_ = 30.0;
    double initial_offset_reach_radius_m_ = 0.08;
    double forward_search_distance_m_ = 1.00;
    double forward_search_reach_radius_m_ = 0.08;
    double drop_alignment_tolerance_ = 0.08;
    double height_tolerance_m_ = 0.10;
    double home_xy_tolerance_m_ = 0.20;
    double kp_ = 0.35;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double deadband_ = 0.04;
    double integral_limit_ = 0.50;
    double max_speed_mps_ = 0.35;
    double integral_forward_ = 0.0;
    double integral_left_ = 0.0;
    double previous_forward_ = 0.0;
    double previous_left_ = 0.0;
    double previous_pid_s_ = -1.0;
};

}  // namespace
}  // namespace d_task_uav_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "d_task_mission");
    try {
        d_task_uav_control::Tag0FixedHeightFollowNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("[tag0_follow] startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
