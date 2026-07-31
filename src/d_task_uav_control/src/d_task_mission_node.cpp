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
        private_node_.param("simple_follow/min_tag_side_px", min_tag_side_px_, 8.0);
        private_node_.param("simple_follow/tag_id", tag_id_, 0);
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
        private_node_.param("simple_follow/search_timeout_s", search_timeout_s_, 20.0);
        private_node_.param("simple_follow/follow_duration_s", follow_duration_s_, 30.0);
        private_node_.param("simple_follow/pre_land_hover_s", pre_land_hover_s_, 7.0);
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
            || initial_offset_reach_radius_m_ <= 0.0 || search_timeout_s_ <= 0.0
            || forward_search_distance_m_ < 0.0
            || forward_search_reach_radius_m_ <= 0.0 || follow_duration_s_ <= 0.0
            || pre_land_hover_s_ < 0.0
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
        vision_health_publisher_ = node_.advertise<std_msgs::Bool>(
            topic("vision_health", "/d_task/vision/health"), 1, true);
        tag_center_publisher_ = node_.advertise<geometry_msgs::PointStamped>(
            topic("tag_center", "/d_task/vision/tag0_center"), 2);
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
        airborne_start_s_ = -1.0;
        follow_start_s_ = -1.0;
        hold_start_s_ = -1.0;
        last_land_request_s_ = -1e9;
        resetPid();
        publishEvent("SIMPLE_FOLLOW_CONFIGURED");
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
        publishEvent("SIMPLE_FOLLOW_READY");
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
        last_takeoff_request_s_ = -1e9;
        publishEvent("SIMPLE_FOLLOW_STARTED");
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

    void sendFixedHeightCommand(double x, double y, double yaw,
                                double vx = 0.0, double vy = 0.0) {
        ego_api_->sendPositionVelocityCmd(
            x, y, home_z_ + flight_height_m_, vx, vy, 0.0, yaw);
    }

    void beginInitialOffset(double now_s) {
        home_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        const double heading = home_yaw_
            - initial_offset_clockwise_deg_ * kPi / 180.0;
        offset_target_x_ = latest_odom_.pose.pose.position.x
            + initial_offset_distance_m_ * std::cos(heading);
        offset_target_y_ = latest_odom_.pose.pose.position.y
            + initial_offset_distance_m_ * std::sin(heading);
        airborne_start_s_ = now_s;
        state_ = "INITIAL_OFFSET";
        resetPid();
        publishEvent("INITIAL_OFFSET_STARTED");
    }

    void beginFollowTag(double now_s) {
        state_ = "FOLLOW_TAG";
        follow_start_s_ = now_s;
        resetPid();
        publishEvent("TAG_FOLLOW_STARTED");
        ROS_INFO("[tag0_follow] tag acquired after %.2fs airborne",
                 now_s - airborne_start_s_);
    }

    void beginForwardSearch() {
        forward_target_x_ = offset_target_x_
            + forward_search_distance_m_ * std::cos(home_yaw_);
        forward_target_y_ = offset_target_y_
            + forward_search_distance_m_ * std::sin(home_yaw_);
        state_ = "FORWARD_SEARCH";
        resetPid();
        publishEvent("FORWARD_SEARCH_STARTED");
    }

    void beginPreLandHover(double now_s) {
        hold_x_ = latest_odom_.pose.pose.position.x;
        hold_y_ = latest_odom_.pose.pose.position.y;
        hold_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        hold_start_s_ = now_s;
        state_ = "HOLD_BEFORE_LAND";
        resetPid();
        publishEvent("PRE_LAND_HOVER_STARTED");
    }

    void requestLanding(double now_s) {
        ego_api_->requestLand();
        last_land_request_s_ = now_s;
    }

    void controlTimerCallback(const ros::TimerEvent&) {
        const double now_s = ros::Time::now().toSec();
        if (started_ && state_ == "TAKEOFF") {
            if (now_s - last_takeoff_request_s_ >= takeoff_retry_s_) {
                ego_api_->requestTakeoffTo(home_z_ + flight_height_m_);
                last_takeoff_request_s_ = now_s;
            }
            if (bridge_state_ == "HOVER") {
                ego_api_->requestOverrideMode(true);
                if (!odomFresh(now_s)) {
                    publishFault(2101, "uav_odometry_stale");
                } else {
                    beginInitialOffset(now_s);
                }
            }
        }

        double command_vx = 0.0;
        double command_vy = 0.0;
        double forward_error = 0.0;
        double left_error = 0.0;
        if (started_ && state_ == "INITIAL_OFFSET") {
            if (!odomFresh(now_s)) {
                publishFault(2101, "uav_odometry_stale");
            } else if (now_s - airborne_start_s_ >= search_timeout_s_) {
                beginPreLandHover(now_s);
            } else if (requestOverrideIfNeeded()) {
                const double dx = offset_target_x_ - latest_odom_.pose.pose.position.x;
                const double dy = offset_target_y_ - latest_odom_.pose.pose.position.y;
                sendFixedHeightCommand(offset_target_x_, offset_target_y_, home_yaw_);
                if (std::hypot(dx, dy) <= initial_offset_reach_radius_m_) {
                    beginForwardSearch();
                }
            }
        } else if (started_ && state_ == "FORWARD_SEARCH") {
            if (!odomFresh(now_s)) {
                publishFault(2101, "uav_odometry_stale");
            } else if (now_s - airborne_start_s_ >= search_timeout_s_) {
                beginPreLandHover(now_s);
            } else if (tagFresh(now_s)) {
                beginFollowTag(now_s);
            } else if (requestOverrideIfNeeded()) {
                const double dx = forward_target_x_
                    - latest_odom_.pose.pose.position.x;
                const double dy = forward_target_y_
                    - latest_odom_.pose.pose.position.y;
                sendFixedHeightCommand(forward_target_x_, forward_target_y_, home_yaw_);
                if (std::hypot(dx, dy) <= forward_search_reach_radius_m_) {
                    beginPreLandHover(now_s);
                }
            }
        } else if (started_ && state_ == "FOLLOW_TAG") {
            if (!odomFresh(now_s)) {
                publishFault(2101, "uav_odometry_stale");
            } else if (now_s - follow_start_s_ >= follow_duration_s_) {
                beginPreLandHover(now_s);
            } else {
                if (requestOverrideIfNeeded()) {
                    const double yaw = yawFromQuaternion(latest_odom_.pose.pose.orientation);
                    computeVelocity(now_s, yaw, command_vx, command_vy,
                                    forward_error, left_error);
                    sendFixedHeightCommand(
                        latest_odom_.pose.pose.position.x,
                        latest_odom_.pose.pose.position.y,
                        yaw, command_vx, command_vy);
                }
            }
        } else if (started_ && state_ == "HOLD_BEFORE_LAND") {
            if (!odomFresh(now_s)) {
                publishFault(2101, "uav_odometry_stale");
            } else if (now_s - hold_start_s_ >= pre_land_hover_s_) {
                state_ = "LANDING";
                publishEvent("LANDING_REQUESTED");
                requestLanding(now_s);
            } else if (requestOverrideIfNeeded()) {
                sendFixedHeightCommand(hold_x_, hold_y_, hold_yaw_);
            }
        } else if (started_ && state_ == "LANDING") {
            if (bridge_state_ == "IDLE") {
                state_ = "COMPLETE";
                publishEvent("SIMPLE_FOLLOW_COMPLETE");
            } else if (bridge_state_ == "HOVER"
                       && now_s - last_land_request_s_ >= landing_retry_s_) {
                requestLanding(now_s);
            }
        }

        publishTaskState();
        publishHealth(now_s);
        publishDebug(now_s, command_vx, command_vy, forward_error, left_error);
    }

    void publishTaskState() {
        Json::Value state;
        state["mission_id"] = mission_id_;
        state["mode"] = mode_;
        state["state"] = state_;
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
        std_msgs::String message;
        message.data = writeJson(value);
        health_publisher_.publish(message);
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
        value["forward_search_target_x"] = forward_target_x_;
        value["forward_search_target_y"] = forward_target_y_;
        value["airborne_elapsed_s"] = airborne_start_s_ < 0.0
            ? 0.0 : now_s - airborne_start_s_;
        value["follow_elapsed_s"] = follow_start_s_ < 0.0
            ? 0.0 : now_s - follow_start_s_;
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
    ros::Publisher vision_health_publisher_;
    ros::Publisher tag_center_publisher_;
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
    uint8_t control_mode_ = 0;
    unsigned int image_width_ = 0;
    unsigned int image_height_ = 0;
    int tag_id_ = 0;
    double last_odom_s_ = -1.0;
    double last_tag_s_ = -1.0;
    double last_takeoff_request_s_ = -1e9;
    double last_land_request_s_ = -1e9;
    double airborne_start_s_ = -1.0;
    double follow_start_s_ = -1.0;
    double hold_start_s_ = -1.0;
    double home_z_ = 0.0;
    double home_yaw_ = 0.0;
    double offset_target_x_ = 0.0;
    double offset_target_y_ = 0.0;
    double forward_target_x_ = 0.0;
    double forward_target_y_ = 0.0;
    double hold_x_ = 0.0;
    double hold_y_ = 0.0;
    double hold_yaw_ = 0.0;
    double tag_confidence_ = 0.0;

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
    double search_timeout_s_ = 20.0;
    double follow_duration_s_ = 30.0;
    double pre_land_hover_s_ = 7.0;
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
