#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <json/json.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include "ego_api/ego_api.h"

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

class Tag0FixedHeightFollowNode {
public:
    Tag0FixedHeightFollowNode()
        : private_node_("~"), image_transport_(node_) {
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
    void loadParameters() {
        private_node_.param<std::string>("mission/bridge_namespace",
                                         bridge_namespace_, "/ego_bridge");
        private_node_.param("simple_follow/flight_height_m", flight_height_m_, 1.50);
        private_node_.param("simple_follow/control_rate_hz", control_rate_hz_, 30.0);
        private_node_.param("simple_follow/odom_timeout_s", odom_timeout_s_, 0.30);
        private_node_.param("simple_follow/tag_timeout_s", tag_timeout_s_, 0.20);
        private_node_.param("simple_follow/takeoff_retry_s", takeoff_retry_s_, 0.50);
        private_node_.param("simple_follow/landing_retry_s", landing_retry_s_, 0.50);
        private_node_.param("simple_follow/platform_request_retry_s",
                            platform_request_retry_s_, 0.50);
        private_node_.param("simple_follow/min_tag_side_px", min_tag_side_px_, 8.0);
        private_node_.param("simple_follow/tag_id", tag_id_, 0);
        private_node_.param("simple_follow/initial_offset_distance_m",
                            initial_offset_distance_m_, 0.50);
        private_node_.param("simple_follow/initial_offset_clockwise_deg",
                            initial_offset_clockwise_deg_, 30.0);
        private_node_.param("simple_follow/initial_offset_reach_radius_m",
                            initial_offset_reach_radius_m_, 0.08);
        private_node_.param("simple_follow/hover_time_s", hover_time_s_, 3.0);
        private_node_.param("simple_follow/mission_timeout_s", mission_timeout_s_, 80.0);
        private_node_.param("simple_follow/vision_acquire_timeout_s",
                            vision_acquire_timeout_s_, 20.0);
        private_node_.param("simple_follow/home_reach_radius_m",
                            home_reach_radius_m_, 0.15);
        private_node_.param("simple_follow/state_settle_time_s",
                            state_settle_time_s_, 0.50);
        private_node_.param("simple_follow/alignment_tolerance_norm",
                            alignment_tolerance_norm_, 0.10);
        private_node_.param("simple_follow/alignment_hold_s",
                            alignment_hold_s_, 0.50);
        private_node_.param("simple_follow/drop_min_follow_s",
                            drop_min_follow_s_, 3.0);
        private_node_.param("simple_follow/payload_release_pulse_s",
                            payload_release_pulse_s_, 0.70);
        private_node_.param("simple_follow/platform_height_offset_m",
                            platform_height_offset_m_, 0.0);
        private_node_.param("simple_follow/landing_approach_height_m",
                            landing_approach_height_m_, 0.25);
        private_node_.param("simple_follow/landing_press_depth_m",
                            landing_press_depth_m_, 0.07);
        private_node_.param("simple_follow/landing_vertical_speed_mps",
                            landing_vertical_speed_mps_, 0.12);
        private_node_.param("simple_follow/landing_height_tolerance_m",
                            landing_height_tolerance_m_, 0.05);
        private_node_.param("simple_follow/platform_hold_s", platform_hold_s_, 5.0);
        private_node_.param("simple_follow/platform_contact_timeout_s",
                            platform_contact_timeout_s_, 12.0);
        private_node_.param("simple_follow/platform_takeoff_timeout_s",
                            platform_takeoff_timeout_s_, 12.0);
        private_node_.param("simple_follow/pid/kp", kp_, 0.35);
        private_node_.param("simple_follow/pid/ki", ki_, 0.0);
        private_node_.param("simple_follow/pid/kd", kd_, 0.0);
        private_node_.param("simple_follow/pid/deadband", deadband_, 0.04);
        private_node_.param("simple_follow/pid/integral_limit", integral_limit_, 0.50);
        private_node_.param("simple_follow/pid/max_speed_mps", max_speed_mps_, 0.35);
        if (flight_height_m_ <= 0.0 || control_rate_hz_ < 2.0
            || odom_timeout_s_ <= 0.0 || tag_timeout_s_ <= 0.0
            || takeoff_retry_s_ <= 0.0 || landing_retry_s_ <= 0.0
            || platform_request_retry_s_ <= 0.0
            || min_tag_side_px_ <= 0.0 || initial_offset_distance_m_ < 0.0
            || initial_offset_reach_radius_m_ <= 0.0 || hover_time_s_ < 3.0
            || mission_timeout_s_ <= hover_time_s_ || vision_acquire_timeout_s_ <= 0.0
            || home_reach_radius_m_ <= 0.0 || state_settle_time_s_ < 0.0
            || alignment_tolerance_norm_ <= 0.0 || alignment_tolerance_norm_ > 1.0
            || alignment_hold_s_ < 0.0 || drop_min_follow_s_ < 0.0
            || payload_release_pulse_s_ <= 0.0 || landing_approach_height_m_ <= 0.0
            || landing_press_depth_m_ < 0.0 || landing_vertical_speed_mps_ <= 0.0
            || landing_height_tolerance_m_ <= 0.0 || platform_hold_s_ < 5.0
            || platform_contact_timeout_s_ <= 0.0 || platform_takeoff_timeout_s_ <= 0.0
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
    }

    void advertiseTopics() {
        task_state_publisher_ = node_.advertise<std_msgs::String>(
            topic("task_state", "/uav_protocol/task_state"), 2, true);
        event_publisher_ = node_.advertise<std_msgs::String>(
            topic("local_event", "/uav_protocol/local_event"), 5);
        fault_publisher_ = node_.advertise<std_msgs::String>(
            topic("local_fault", "/uav_protocol/local_fault"), 5);
        health_publisher_ = node_.advertise<std_msgs::String>(
            topic("local_health", "/uav_protocol/local_health"), 2);
        tracking_publisher_ = node_.advertise<std_msgs::String>(
            topic("local_tracking", "/uav_protocol/local_tracking"), 2);
        vision_health_publisher_ = node_.advertise<std_msgs::Bool>(
            topic("vision_health", "/d_task/vision/health"), 1, true);
        tag_center_publisher_ = node_.advertise<geometry_msgs::PointStamped>(
            topic("tag_center", "/d_task/vision/tag0_center"), 2);
        payload_release_publisher_ = node_.advertise<std_msgs::Bool>(
            topic("payload_release", "/d_task/payload/release"), 1, true);
        debug_publisher_ = node_.advertise<std_msgs::String>(
            topic("debug", "/d_task/simple_follow/debug"), 2);
    }

    void missionConfigCallback(const std_msgs::String::ConstPtr& message) {
        Json::Value root;
        if (!parseJson(message->data, root)
            || root.get("type", "").asString() != "mission_config"
            || root.get("sender", "").asString() != "car"
            || !root["payload"].isObject()) {
            publishFault(2301, "invalid_mission_config");
            return;
        }
        mission_id_ = root.get("mission_id", "").asString();
        mode_ = root["payload"].get("mode", "DROP").asString();
        state_ = "POSITIONING_INIT";
        positioning_ready_ = false;
        started_ = false;
        mission_start_s_ = -1.0;
        state_enter_s_ = -1.0;
        follow_start_s_ = -1.0;
        aligned_since_s_ = -1.0;
        release_start_s_ = -1.0;
        platform_hold_start_s_ = -1.0;
        platform_landing_start_s_ = -1.0;
        platform_takeoff_start_s_ = -1.0;
        last_land_request_s_ = -1e9;
        last_platform_request_s_ = -1e9;
        payload_released_ = false;
        payload_release_active_ = false;
        initial_offset_pending_ = true;
        mission_failed_ = false;
        resetPid();
        publishPayloadRelease(false);
        publishEvent("MISSION_CONFIGURED");
        publishTaskState();
    }

    void positioningStatusCallback(const std_msgs::String::ConstPtr& message) {
        Json::Value root;
        if (!parseJson(message->data, root)
            || !root.get("ready", false).asBool()
            || root.get("mission_id", "").asString() != mission_id_) {
            return;
        }
        home_z_ = root.get("home_z", 0.0).asDouble();
        if (has_odom_) {
            home_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        }
        positioning_ready_ = true;
        state_ = "WAIT_START";
        publishEvent("MISSION_READY");
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
        started_ = true;
        state_ = "TAKEOFF";
        mission_start_s_ = ros::Time::now().toSec();
        state_enter_s_ = mission_start_s_;
        last_takeoff_request_s_ = -1e9;
        publishEvent("MISSION_STARTED");
        publishTaskState();
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

        const double normal_u = 2.0 * tag_center_.x
            / static_cast<double>(image_width_) - 1.0;
        const double normal_v = 2.0 * tag_center_.y
            / static_cast<double>(image_height_) - 1.0;
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

    double cruiseZ() const { return home_z_ + flight_height_m_; }

    double platformZ() const { return home_z_ + platform_height_offset_m_; }

    bool isDropMode() const { return mode_ == "DROP"; }

    void setState(const std::string& next, double now_s,
                  const std::string& event) {
        state_ = next;
        state_enter_s_ = now_s;
        aligned_since_s_ = -1.0;
        if (!event.empty()) {
            publishEvent(event);
        }
    }

    void sendPositionCommand(double x, double y, double z, double yaw,
                             double vx = 0.0, double vy = 0.0,
                             double vz = 0.0) {
        ego_api_->sendPositionVelocityCmd(x, y, z, vx, vy, vz, yaw);
    }

    void holdAtCurrent(double z) {
        sendPositionCommand(
            latest_odom_.pose.pose.position.x,
            latest_odom_.pose.pose.position.y,
            z,
            yawFromQuaternion(latest_odom_.pose.pose.orientation));
    }

    bool tagAligned() const {
        if (!tag_found_ || image_width_ == 0U || image_height_ == 0U) {
            return false;
        }
        const double normal_u = 2.0 * tag_center_.x
            / static_cast<double>(image_width_) - 1.0;
        const double normal_v = 2.0 * tag_center_.y
            / static_cast<double>(image_height_) - 1.0;
        return std::abs(normal_u) <= alignment_tolerance_norm_
            && std::abs(normal_v) <= alignment_tolerance_norm_;
    }

    bool alignedFor(double now_s, double duration_s) {
        if (!tagFresh(now_s) || !tagAligned()) {
            aligned_since_s_ = -1.0;
            return false;
        }
        if (aligned_since_s_ < 0.0) {
            aligned_since_s_ = now_s;
        }
        return now_s - aligned_since_s_ + 1e-6 >= duration_s;
    }

    bool followTagAtHeight(double now_s, double z, double vz,
                           double& command_vx, double& command_vy,
                           double& forward_error, double& left_error) {
        if (!tagFresh(now_s)) {
            resetPid();
            return false;
        }
        const double yaw = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        computeVelocity(now_s, yaw, command_vx, command_vy,
                        forward_error, left_error);
        sendPositionCommand(
            latest_odom_.pose.pose.position.x,
            latest_odom_.pose.pose.position.y,
            z, yaw, command_vx, command_vy, vz);
        return true;
    }

    void beginSearch(double now_s) {
        home_x_ = latest_odom_.pose.pose.position.x;
        home_y_ = latest_odom_.pose.pose.position.y;
        home_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        const double heading = home_yaw_
            - initial_offset_clockwise_deg_ * kPi / 180.0;
        offset_target_x_ = home_x_
            + initial_offset_distance_m_ * std::cos(heading);
        offset_target_y_ = home_y_
            + initial_offset_distance_m_ * std::sin(heading);
        initial_offset_pending_ = true;
        resetPid();
        setState("SEARCH_CAR", now_s, "SEARCH_CAR_STARTED");
    }

    void beginReturnHome(double now_s, const std::string& event,
                         bool failed = false) {
        mission_failed_ = mission_failed_ || failed;
        resetPid();
        setState("RETURN_HOME", now_s, event);
    }

    void beginRecoveryClimb(double now_s, const std::string& event) {
        recovery_x_ = latest_odom_.pose.pose.position.x;
        recovery_y_ = latest_odom_.pose.pose.position.y;
        recovery_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        resetPid();
        setState("CLIMB_TO_CRUISE", now_s, event);
    }

    void requestLanding(double now_s) {
        ego_api_->requestLand();
        last_land_request_s_ = now_s;
    }

    void requestPlatformLanding(double now_s) {
        if (now_s - last_platform_request_s_ < platform_request_retry_s_) {
            return;
        }
        ego_api_->requestPlatformDisarm();
        last_platform_request_s_ = now_s;
    }

    void requestPlatformTakeoff(double now_s) {
        if (now_s - last_platform_request_s_ < platform_request_retry_s_) {
            return;
        }
        ego_api_->requestPlatformTakeoff(
            latest_odom_.pose.pose.position.x,
            latest_odom_.pose.pose.position.y,
            cruiseZ(),
            yawFromQuaternion(latest_odom_.pose.pose.orientation));
        last_platform_request_s_ = now_s;
    }

    void publishPayloadRelease(bool release) {
        std_msgs::Bool message;
        message.data = release;
        payload_release_publisher_.publish(message);
        payload_release_active_ = release;
    }

    void abortOnStaleOdom(double now_s) {
        publishFault(2101, "uav_odometry_stale");
        if (state_ == "LAND_ON_PLATFORM") {
            ego_api_->requestPlatformLandingCancel();
        }
        beginReturnHome(now_s, "UAV_ODOMETRY_STALE_RETURN", true);
    }

    void controlTimerCallback(const ros::TimerEvent&) {
        const double now_s = ros::Time::now().toSec();
        double command_vx = 0.0;
        double command_vy = 0.0;
        double forward_error = 0.0;
        double left_error = 0.0;

        const bool active_flight_state = state_ == "SEARCH_CAR"
            || state_ == "FOLLOW_CAR" || state_ == "DESCEND_HIGH"
            || state_ == "LAND_ON_PLATFORM" || state_ == "CLIMB_TO_CRUISE";
        if (started_ && mission_start_s_ >= 0.0
            && now_s - mission_start_s_ > mission_timeout_s_
            && active_flight_state) {
            publishFault(2102, "mission_timeout_return_home");
            if (state_ == "LAND_ON_PLATFORM") {
                ego_api_->requestPlatformLandingCancel();
            }
            beginReturnHome(now_s, "MISSION_TIMEOUT_RETURN", true);
        }

        if (started_ && state_ == "TAKEOFF") {
            if (now_s - last_takeoff_request_s_ >= takeoff_retry_s_) {
                ego_api_->requestTakeoffTo(cruiseZ());
                last_takeoff_request_s_ = now_s;
            }
            if (bridge_state_ == "HOVER") {
                if (!odomFresh(now_s)) {
                    publishFault(2101, "uav_odometry_stale");
                } else {
                    home_x_ = latest_odom_.pose.pose.position.x;
                    home_y_ = latest_odom_.pose.pose.position.y;
                    home_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
                    setState("HOVER_3S", now_s, "CRUISE_HOVER_STARTED");
                }
            }
        } else if (started_ && state_ == "HOVER_3S") {
            if (!odomFresh(now_s)) {
                abortOnStaleOdom(now_s);
            } else if (now_s - state_enter_s_ >= hover_time_s_) {
                ego_api_->requestOverrideMode(true);
                beginSearch(now_s);
            }
        } else if (started_ && state_ == "SEARCH_CAR") {
            if (!odomFresh(now_s)) {
                abortOnStaleOdom(now_s);
            } else if (!requestOverrideIfNeeded()) {
                // Wait for the bridge to acknowledge OVERRIDE before sending a
                // competition command to the moving platform.
            } else if (initial_offset_pending_) {
                sendPositionCommand(offset_target_x_, offset_target_y_, cruiseZ(), home_yaw_);
                const double dx = offset_target_x_ - latest_odom_.pose.pose.position.x;
                const double dy = offset_target_y_ - latest_odom_.pose.pose.position.y;
                if (std::hypot(dx, dy) <= initial_offset_reach_radius_m_) {
                    initial_offset_pending_ = false;
                    state_enter_s_ = now_s;
                    publishEvent("SEARCH_OFFSET_REACHED");
                }
            } else if (tagFresh(now_s)) {
                follow_start_s_ = now_s;
                resetPid();
                setState("FOLLOW_CAR", now_s, "FOLLOW_CAR_ESTABLISHED");
            } else {
                holdAtCurrent(cruiseZ());
                if (now_s - state_enter_s_ > vision_acquire_timeout_s_) {
                    publishFault(2104, "platform_tag_not_found");
                    beginReturnHome(now_s, "SEARCH_TIMEOUT_RETURN", true);
                }
            }
        } else if (started_ && state_ == "FOLLOW_CAR") {
            if (!odomFresh(now_s)) {
                abortOnStaleOdom(now_s);
            } else if (!requestOverrideIfNeeded()) {
                // The current bridge setpoint holds the drone until OVERRIDE returns.
            } else if (!followTagAtHeight(now_s, cruiseZ(), 0.0,
                                           command_vx, command_vy,
                                           forward_error, left_error)) {
                initial_offset_pending_ = false;
                setState("SEARCH_CAR", now_s, "PLATFORM_TAG_LOST_SEARCHING");
            } else if (isDropMode()) {
                if (now_s - follow_start_s_ >= drop_min_follow_s_
                    && alignedFor(now_s, alignment_hold_s_)) {
                    payload_released_ = true;
                    release_start_s_ = now_s;
                    publishPayloadRelease(true);
                    setState("RELEASE", now_s, "PAYLOAD_RELEASE_STARTED");
                }
            } else if (alignedFor(now_s, alignment_hold_s_)) {
                setState("DESCEND_HIGH", now_s, "DYNAMIC_DESCENT_STARTED");
            }
        } else if (started_ && state_ == "RELEASE") {
            if (!odomFresh(now_s)) {
                publishPayloadRelease(false);
                abortOnStaleOdom(now_s);
            } else {
                if (requestOverrideIfNeeded()) {
                    followTagAtHeight(now_s, cruiseZ(), 0.0,
                                      command_vx, command_vy,
                                      forward_error, left_error);
                }
                if (now_s - release_start_s_ >= payload_release_pulse_s_) {
                    publishPayloadRelease(false);
                    // The car must not accelerate at RELEASE: that state begins
                    // the GPIO pulse.  This one-shot event is sent only after
                    // the configured pulse has elapsed and the output is low.
                    publishEvent("PAYLOAD_RELEASED_SPEED_UP");
                    beginReturnHome(now_s, "PAYLOAD_RELEASED_RETURN_HOME");
                }
            }
        } else if (started_ && state_ == "DESCEND_HIGH") {
            if (!odomFresh(now_s)) {
                abortOnStaleOdom(now_s);
            } else if (!requestOverrideIfNeeded()) {
                // Wait for OVERRIDE.
            } else if (!followTagAtHeight(now_s,
                                           platformZ() + landing_approach_height_m_,
                                           -landing_vertical_speed_mps_,
                                           command_vx, command_vy,
                                           forward_error, left_error)) {
                beginRecoveryClimb(now_s, "DESCENT_TAG_LOST_RECOVERY");
            } else if (std::abs(latest_odom_.pose.pose.position.z
                                - (platformZ() + landing_approach_height_m_))
                           <= landing_height_tolerance_m_
                       && alignedFor(now_s, state_settle_time_s_)) {
                platform_landing_start_s_ = now_s;
                last_platform_request_s_ = -1e9;
                setState("LAND_ON_PLATFORM", now_s, "PLATFORM_CONTACT_STARTED");
            }
        } else if (started_ && state_ == "LAND_ON_PLATFORM") {
            if (bridge_state_ == "PLATFORM_LANDED") {
                platform_hold_start_s_ = now_s;
                setState("PLATFORM_HOLD", now_s, "PLATFORM_LANDED");
            } else if (!odomFresh(now_s)) {
                abortOnStaleOdom(now_s);
            } else if (now_s - platform_landing_start_s_ > platform_contact_timeout_s_) {
                ego_api_->requestPlatformLandingCancel();
                publishFault(2201, "platform_contact_timeout");
                beginRecoveryClimb(now_s, "PLATFORM_CONTACT_TIMEOUT_RECOVERY");
            } else if (!requestOverrideIfNeeded()) {
                // Wait for OVERRIDE before asking the bridge to enter contact mode.
            } else if (!followTagAtHeight(now_s,
                                           platformZ() - landing_press_depth_m_,
                                           -landing_vertical_speed_mps_,
                                           command_vx, command_vy,
                                           forward_error, left_error)) {
                ego_api_->requestPlatformLandingCancel();
                beginRecoveryClimb(now_s, "PLATFORM_TAG_LOST_RECOVERY");
            } else {
                requestPlatformLanding(now_s);
            }
        } else if (started_ && state_ == "PLATFORM_HOLD") {
            if (bridge_state_ != "PLATFORM_LANDED") {
                mission_failed_ = true;
                setState("ABORT", now_s, "PLATFORM_HOLD_INTERRUPTED");
            } else if (now_s - platform_hold_start_s_ >= platform_hold_s_) {
                platform_takeoff_start_s_ = now_s;
                last_platform_request_s_ = -1e9;
                setState("PLATFORM_TAKEOFF", now_s, "PLATFORM_TAKEOFF_STARTED");
                requestPlatformTakeoff(now_s);
            }
        } else if (started_ && state_ == "PLATFORM_TAKEOFF") {
            if (bridge_state_ == "HOVER") {
                beginReturnHome(now_s, "PLATFORM_TAKEOFF_COMPLETE_RETURN_HOME");
            } else if (now_s - platform_takeoff_start_s_ > platform_takeoff_timeout_s_) {
                mission_failed_ = true;
                publishFault(2204, "platform_takeoff_timeout");
                setState("ABORT", now_s, "PLATFORM_TAKEOFF_ABORT");
            } else {
                requestPlatformTakeoff(now_s);
            }
        } else if (started_ && state_ == "CLIMB_TO_CRUISE") {
            if (!odomFresh(now_s)) {
                abortOnStaleOdom(now_s);
            } else if (!requestOverrideIfNeeded()) {
                // Wait for OVERRIDE.
            } else {
                sendPositionCommand(recovery_x_, recovery_y_, cruiseZ(), recovery_yaw_);
                const double dx = recovery_x_ - latest_odom_.pose.pose.position.x;
                const double dy = recovery_y_ - latest_odom_.pose.pose.position.y;
                if (std::hypot(dx, dy) <= home_reach_radius_m_
                    && std::abs(latest_odom_.pose.pose.position.z - cruiseZ())
                        <= landing_height_tolerance_m_) {
                    initial_offset_pending_ = false;
                    setState("SEARCH_CAR", now_s, "RECOVERY_SEARCH_RESUMED");
                }
            }
        } else if (started_ && state_ == "RETURN_HOME") {
            if (!odomFresh(now_s)) {
                mission_failed_ = true;
                setState("ABORT", now_s, "RETURN_HOME_ODOMETRY_LOST");
            } else if (!requestOverrideIfNeeded()) {
                // Wait for OVERRIDE.
            } else {
                sendPositionCommand(home_x_, home_y_, cruiseZ(), home_yaw_);
                const double dx = home_x_ - latest_odom_.pose.pose.position.x;
                const double dy = home_y_ - latest_odom_.pose.pose.position.y;
                if (std::hypot(dx, dy) <= home_reach_radius_m_
                    && std::abs(latest_odom_.pose.pose.position.z - cruiseZ())
                        <= landing_height_tolerance_m_) {
                    setState("LAND_HOME", now_s, "HOME_REACHED_LANDING");
                    requestLanding(now_s);
                }
            }
        } else if (started_ && state_ == "LAND_HOME") {
            if (bridge_state_ == "IDLE") {
                setState(mission_failed_ ? "ABORT" : "COMPLETE", now_s,
                         mission_failed_ ? "MISSION_ABORTED" : "MISSION_COMPLETE");
            } else if (bridge_state_ == "HOVER"
                       && now_s - last_land_request_s_ >= landing_retry_s_) {
                requestLanding(now_s);
            }
        }

        publishTaskState();
        publishHealth(now_s);
        publishTracking(now_s, command_vx, command_vy,
                        forward_error, left_error);
        publishDebug(now_s, command_vx, command_vy, forward_error, left_error);
    }

    void publishTaskState() {
        Json::Value state;
        state["mission_id"] = mission_id_;
        state["mode"] = mode_;
        state["state"] = state_;
        state["vision_locked"] = tagFresh(ros::Time::now().toSec());
        state["payload_released"] = payload_released_;
        state["payload_output_active"] = payload_release_active_;
        state["mission_elapsed_ms"] = mission_start_s_ < 0.0
            ? 0 : static_cast<Json::Int64>(std::max(0.0,
                ros::Time::now().toSec() - mission_start_s_) * 1000.0);
        std_msgs::String message;
        message.data = writeJson(state);
        task_state_publisher_.publish(message);
    }

    void publishEvent(const std::string& event) {
        Json::Value value;
        value["event"] = event;
        value["mission_id"] = mission_id_;
        std_msgs::String message;
        message.data = writeJson(value);
        event_publisher_.publish(message);
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
        value["state"] = state_;
        value["mode"] = mode_;
        value["payload_released"] = payload_released_;
        std_msgs::String message;
        message.data = writeJson(value);
        health_publisher_.publish(message);
    }

    void publishTracking(double now_s, double command_vx, double command_vy,
                         double forward_error, double left_error) {
        const bool detected = tagFresh(now_s);
        Json::Value value;
        value["track_state"] = detected ? "LOCKED" : "INVALID";
        value["detected"] = detected;
        value["confidence"] = detected ? tag_confidence_ : 0.0;
        value["pixel_center"]["u"] = tag_center_.x;
        value["pixel_center"]["v"] = tag_center_.y;
        // This simple controller deliberately has no world-space target
        // estimate.  Keep the required fields zero and publish the measured
        // normalised pixel error explicitly for the ground station.
        value["relative_position"]["x"] = 0.0;
        value["relative_position"]["y"] = 0.0;
        value["relative_position"]["z"] = 0.0;
        value["relative_velocity"]["x"] = 0.0;
        value["relative_velocity"]["y"] = 0.0;
        value["command_velocity"]["x"] = command_vx;
        value["command_velocity"]["y"] = command_vy;
        value["pixel_error_norm"]["forward"] = forward_error;
        value["pixel_error_norm"]["left"] = left_error;
        value["vision_age_ms"] = last_tag_s_ < 0.0
            ? Json::UInt64(0x7fffffffU)
            : static_cast<Json::UInt64>(std::max(0.0, now_s - last_tag_s_) * 1000.0);
        value["filter_mode"] = detected ? "MEASURED" : "INVALID";
        value["release_gate"] = isDropMode() && state_ == "FOLLOW_CAR"
            && detected && tagAligned();
        value["landing_gate"] = !isDropMode()
            && (state_ == "DESCEND_HIGH" || state_ == "LAND_ON_PLATFORM")
            && detected && tagAligned();
        std_msgs::String message;
        message.data = writeJson(value);
        tracking_publisher_.publish(message);
    }

    void publishDebug(double now_s, double command_vx, double command_vy,
                      double forward_error, double left_error) {
        Json::Value value;
        value["state"] = state_;
        value["tag0_found"] = tagFresh(now_s);
        value["tag0_u"] = tag_center_.x;
        value["tag0_v"] = tag_center_.y;
        value["forward_error"] = forward_error;
        value["left_error"] = left_error;
        value["command_vx"] = command_vx;
        value["command_vy"] = command_vy;
        value["fixed_height_m"] = home_z_ + flight_height_m_;
        value["initial_offset_target_x"] = offset_target_x_;
        value["initial_offset_target_y"] = offset_target_y_;
        value["airborne_elapsed_s"] = mission_start_s_ < 0.0
            ? 0.0 : now_s - mission_start_s_;
        value["payload_released"] = payload_released_;
        value["payload_output_active"] = payload_release_active_;
        value["bridge_state"] = bridge_state_;
        std_msgs::String message;
        message.data = writeJson(value);
        debug_publisher_.publish(message);
    }

    ros::NodeHandle node_;
    ros::NodeHandle private_node_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_subscriber_;
    std::unique_ptr<EgoApi> ego_api_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;

    ros::Subscriber mission_config_subscriber_;
    ros::Subscriber mission_start_subscriber_;
    ros::Subscriber positioning_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber bridge_state_subscriber_;
    ros::Subscriber control_mode_subscriber_;
    ros::Publisher task_state_publisher_;
    ros::Publisher event_publisher_;
    ros::Publisher fault_publisher_;
    ros::Publisher health_publisher_;
    ros::Publisher tracking_publisher_;
    ros::Publisher vision_health_publisher_;
    ros::Publisher tag_center_publisher_;
    ros::Publisher payload_release_publisher_;
    ros::Publisher debug_publisher_;
    ros::Timer control_timer_;

    std::string bridge_namespace_;
    std::string mission_id_;
    std::string mode_ = "DROP";
    std::string state_ = "NOT_READY";
    std::string bridge_state_;
    std::string last_fault_key_;
    nav_msgs::Odometry latest_odom_;
    cv::Point2f tag_center_;
    bool has_odom_ = false;
    bool tag_found_ = false;
    bool positioning_ready_ = false;
    bool started_ = false;
    bool initial_offset_pending_ = true;
    bool payload_released_ = false;
    bool payload_release_active_ = false;
    bool mission_failed_ = false;
    uint8_t control_mode_ = 0;
    unsigned int image_width_ = 0;
    unsigned int image_height_ = 0;
    int tag_id_ = 0;
    double last_odom_s_ = -1.0;
    double last_tag_s_ = -1.0;
    double last_takeoff_request_s_ = -1e9;
    double last_land_request_s_ = -1e9;
    double last_platform_request_s_ = -1e9;
    double mission_start_s_ = -1.0;
    double state_enter_s_ = -1.0;
    double follow_start_s_ = -1.0;
    double aligned_since_s_ = -1.0;
    double release_start_s_ = -1.0;
    double platform_hold_start_s_ = -1.0;
    double platform_landing_start_s_ = -1.0;
    double platform_takeoff_start_s_ = -1.0;
    double home_z_ = 0.0;
    double home_x_ = 0.0;
    double home_y_ = 0.0;
    double home_yaw_ = 0.0;
    double offset_target_x_ = 0.0;
    double offset_target_y_ = 0.0;
    double recovery_x_ = 0.0;
    double recovery_y_ = 0.0;
    double recovery_yaw_ = 0.0;
    double tag_confidence_ = 0.0;

    double flight_height_m_ = 1.50;
    double control_rate_hz_ = 30.0;
    double odom_timeout_s_ = 0.30;
    double tag_timeout_s_ = 0.20;
    double takeoff_retry_s_ = 0.50;
    double landing_retry_s_ = 0.50;
    double platform_request_retry_s_ = 0.50;
    double min_tag_side_px_ = 8.0;
    double initial_offset_distance_m_ = 0.50;
    double initial_offset_clockwise_deg_ = 30.0;
    double initial_offset_reach_radius_m_ = 0.08;
    double hover_time_s_ = 3.0;
    double mission_timeout_s_ = 80.0;
    double vision_acquire_timeout_s_ = 20.0;
    double home_reach_radius_m_ = 0.15;
    double state_settle_time_s_ = 0.50;
    double alignment_tolerance_norm_ = 0.10;
    double alignment_hold_s_ = 0.50;
    double drop_min_follow_s_ = 3.0;
    double payload_release_pulse_s_ = 0.70;
    double platform_height_offset_m_ = 0.0;
    double landing_approach_height_m_ = 0.25;
    double landing_press_depth_m_ = 0.07;
    double landing_vertical_speed_mps_ = 0.12;
    double landing_height_tolerance_m_ = 0.05;
    double platform_hold_s_ = 5.0;
    double platform_contact_timeout_s_ = 12.0;
    double platform_takeoff_timeout_s_ = 12.0;
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
