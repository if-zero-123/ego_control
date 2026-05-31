#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

geometry_msgs::Quaternion yawToQuat(double yaw) {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

double yawFromQuat(const geometry_msgs::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

struct FrameDetection {
    bool found = false;
    int u0 = 0;
    int u1 = 0;
    int v0 = 0;
    int v1 = 0;
    int area = 0;
    double err_y = 0.0;
    double err_z = 0.0;
    double dist = std::numeric_limits<double>::quiet_NaN();
};

struct FrameGoals {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d pre = Eigen::Vector3d::Zero();
    Eigen::Vector3d post = Eigen::Vector3d::Zero();
    double yaw = 0.0;
};

class FrameDebugNode {
public:
    FrameDebugNode() : nh_(), pnh_("~"), it_(nh_) {
        pnh_.param<std::string>("depth_topic", depth_topic_, "/camera/depth/image_rect_raw");
        pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/depth/camera_info");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
        pnh_.param<std::string>("frame_id", configured_frame_id_, "");

        pnh_.param<double>("depth_min", depth_min_, 0.35);
        pnh_.param<double>("depth_near_threshold", depth_near_threshold_, 1.8);
        pnh_.param<double>("depth_max", depth_max_, 3.0);
        pnh_.param<int>("min_open_width_px", min_open_width_px_, 70);
        pnh_.param<int>("min_open_height_px", min_open_height_px_, 70);
        pnh_.param<double>("roi_x_margin_ratio", roi_x_margin_ratio_, 0.20);
        pnh_.param<double>("roi_y_top_ratio", roi_y_top_ratio_, 0.20);
        pnh_.param<double>("roi_y_bottom_ratio", roi_y_bottom_ratio_, 0.10);
        pnh_.param<double>("row_open_ratio", row_open_ratio_, 0.65);
        pnh_.param<double>("pre_offset", pre_offset_, 0.60);
        pnh_.param<double>("post_offset", post_offset_, 0.90);
        pnh_.param<double>("debug_log_period", debug_log_period_, 1.0);
        pnh_.param<double>("depth_stale_timeout", depth_stale_timeout_, 1.0);
        pnh_.param<bool>("publish_debug_image", publish_debug_image_, true);

        depth_sub_ = it_.subscribe(depth_topic_, 1, &FrameDebugNode::depthCb, this);
        info_sub_ = nh_.subscribe(camera_info_topic_, 1, &FrameDebugNode::infoCb, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &FrameDebugNode::odomCb, this);

        status_pub_ = nh_.advertise<std_msgs::String>("/craic_debug/frame_status", 1, true);
        center_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/craic_debug/frame_center", 1, true);
        pre_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic_debug/frame_pre_goal", 1, true);
        post_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic_debug/frame_post_goal", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/craic_debug/frame_markers", 1, true);
        debug_image_pub_ = it_.advertise("/craic_debug/frame_debug_image", 1);

        health_timer_ = nh_.createTimer(ros::Duration(0.5), &FrameDebugNode::healthTimerCb, this);

        ROS_INFO("[frame_debug] depth=%s camera_info=%s odom=%s depth=[%.2f %.2f %.2f] open_px=%dx%d offsets=%.2f/%.2f",
                 depth_topic_.c_str(), camera_info_topic_.c_str(), odom_topic_.c_str(),
                 depth_min_, depth_near_threshold_, depth_max_,
                 min_open_width_px_, min_open_height_px_, pre_offset_, post_offset_);
    }

private:
    void infoCb(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        fx_ = msg->K[0];
        fy_ = msg->K[4];
        cx_ = msg->K[2];
        cy_ = msg->K[5];
        have_info_ = fx_ > 1.0 && fy_ > 1.0;
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        latest_odom_ = *msg;
        have_odom_ = true;
    }

    void depthCb(const sensor_msgs::ImageConstPtr& msg) {
        have_depth_ = true;
        last_depth_stamp_ = ros::Time::now();

        if (!have_info_) {
            publishStatus("waiting_camera_info");
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg);
        } catch (const cv_bridge::Exception& e) {
            publishStatus("no_depth");
            ROS_WARN_THROTTLE(debug_log_period_, "[frame_debug] cv_bridge error: %s", e.what());
            return;
        }

        if (cv_ptr->image.empty()) {
            publishStatus("no_depth");
            return;
        }

        FrameDetection detection;
        const bool found = processDepth(cv_ptr->image, detection);
        if (publish_debug_image_) {
            publishDebugImage(cv_ptr->image, detection, msg->header);
        }

        if (!found) {
            publishStatus("no_opening");
            clearMarkers(outputFrame(), msg->header.stamp);
            ROS_WARN_THROTTLE(debug_log_period_,
                              "[frame_debug] no_opening frame=%s size=%dx%d thresholds=[%.2f %.2f %.2f]",
                              msg->header.frame_id.c_str(), cv_ptr->image.cols, cv_ptr->image.rows,
                              depth_min_, depth_near_threshold_, depth_max_);
            return;
        }

        if (!have_odom_) {
            publishStatus("waiting_odom");
            ROS_INFO_THROTTLE(debug_log_period_,
                              "[frame_debug] opening detected but waiting odom err_y=%.3f err_z=%.3f dist=%.3f bbox=(%d,%d)-(%d,%d)",
                              detection.err_y, detection.err_z, detection.dist,
                              detection.u0, detection.v0, detection.u1, detection.v1);
            return;
        }

        const std::string frame_id = outputFrame();
        const FrameGoals goals = makeGoals(detection);
        publishOutputs(goals, frame_id, msg->header.stamp);
        publishMarkers(goals, frame_id, msg->header.stamp);
        publishStatus("valid");

        ROS_INFO_THROTTLE(debug_log_period_,
                          "[frame_debug] valid err_y=%.3f err_z=%.3f dist=%.3f bbox=(%d,%d)-(%d,%d) center=(%.2f %.2f %.2f) pre=(%.2f %.2f %.2f) post=(%.2f %.2f %.2f)",
                          detection.err_y, detection.err_z, detection.dist,
                          detection.u0, detection.v0, detection.u1, detection.v1,
                          goals.center.x(), goals.center.y(), goals.center.z(),
                          goals.pre.x(), goals.pre.y(), goals.pre.z(),
                          goals.post.x(), goals.post.y(), goals.post.z());
    }

    bool processDepth(const cv::Mat& depth, FrameDetection& det) const {
        if (depth.empty()) return false;

        const int width = depth.cols;
        const int height = depth.rows;
        const int x_margin = clampInt(static_cast<int>(width * roi_x_margin_ratio_), 0, width / 2 - 1);
        const int y_top = clampInt(static_cast<int>(height * roi_y_top_ratio_), 0, height - 1);
        const int y_bottom_margin = clampInt(static_cast<int>(height * roi_y_bottom_ratio_), 0, height - 1);
        const int x0 = x_margin;
        const int x1 = width - x_margin;
        const int y0 = y_top;
        const int y1 = height - y_bottom_margin;

        FrameDetection best;
        for (int v = y0; v < y1; ++v) {
            int run_start = -1;
            for (int u = x0; u <= x1; ++u) {
                const bool at_end = (u == x1);
                const bool open = !at_end && isOpenDepth(depth, u, v);
                if (open && run_start < 0) {
                    run_start = u;
                }
                if ((!open || at_end) && run_start >= 0) {
                    const int run_end = u - 1;
                    const int run_width = run_end - run_start + 1;
                    if (run_width >= min_open_width_px_) {
                        int vv0 = v;
                        int vv1 = v;
                        growVertical(depth, run_start, run_end, y0, y1, vv0, vv1);
                        const int run_height = vv1 - vv0 + 1;
                        const int area = run_width * run_height;
                        if (run_height >= min_open_height_px_ && area > best.area) {
                            best.found = true;
                            best.u0 = run_start;
                            best.u1 = run_end;
                            best.v0 = vv0;
                            best.v1 = vv1;
                            best.area = area;
                        }
                    }
                    run_start = -1;
                }
            }
        }

        if (!best.found) return false;

        const double dist = estimateFrameDistance(depth, best.u0, best.u1, best.v0, best.v1);
        if (!std::isfinite(dist)) return false;

        const double u_center = 0.5 * (best.u0 + best.u1);
        const double v_center = 0.5 * (best.v0 + best.v1);
        best.err_y = (u_center - cx_) / fx_ * dist;
        best.err_z = -(v_center - cy_) / fy_ * dist;
        best.dist = dist;

        det = best;
        return true;
    }

    int clampInt(int value, int min_value, int max_value) const {
        if (max_value < min_value) return min_value;
        return std::max(min_value, std::min(value, max_value));
    }

    bool isOpenDepth(const cv::Mat& depth, int u, int v) const {
        const double d = readDepthMeters(depth, u, v);
        if (!std::isfinite(d)) return true;
        return d > depth_near_threshold_ && d < depth_max_;
    }

    double readDepthMeters(const cv::Mat& depth, int u, int v) const {
        if (u < 0 || v < 0 || u >= depth.cols || v >= depth.rows) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (depth.type() == CV_16UC1) {
            const uint16_t raw = depth.at<uint16_t>(v, u);
            if (raw == 0) return std::numeric_limits<double>::quiet_NaN();
            return static_cast<double>(raw) * 0.001;
        }
        if (depth.type() == CV_32FC1) {
            const float raw = depth.at<float>(v, u);
            if (!std::isfinite(raw) || raw <= 0.0f) return std::numeric_limits<double>::quiet_NaN();
            return static_cast<double>(raw);
        }
        return std::numeric_limits<double>::quiet_NaN();
    }

    void growVertical(const cv::Mat& depth,
                      int u0,
                      int u1,
                      int y_min,
                      int y_max,
                      int& v0,
                      int& v1) const {
        const int step = std::max(1, (u1 - u0 + 1) / 12);
        while (v0 > y_min && rowOpenRatio(depth, u0, u1, v0 - 1, step) > row_open_ratio_) --v0;
        while (v1 < y_max - 1 && rowOpenRatio(depth, u0, u1, v1 + 1, step) > row_open_ratio_) ++v1;
    }

    double rowOpenRatio(const cv::Mat& depth, int u0, int u1, int v, int step) const {
        int open = 0;
        int total = 0;
        for (int u = u0; u <= u1; u += step) {
            ++total;
            if (isOpenDepth(depth, u, v)) ++open;
        }
        return total > 0 ? static_cast<double>(open) / total : 0.0;
    }

    double estimateFrameDistance(const cv::Mat& depth,
                                 int u0,
                                 int u1,
                                 int v0,
                                 int v1) const {
        std::vector<double> samples;
        const int margin = 8;
        const int step = 4;
        const int left0 = std::max(0, u0 - margin * 3);
        const int left1 = std::max(0, u0 - margin);
        const int right0 = std::min(depth.cols - 1, u1 + margin);
        const int right1 = std::min(depth.cols - 1, u1 + margin * 3);
        for (int v = v0; v <= v1; v += step) {
            for (int u = left0; u <= left1; u += step) collectNearDepth(depth, u, v, samples);
            for (int u = right0; u <= right1; u += step) collectNearDepth(depth, u, v, samples);
        }
        if (samples.empty()) return std::numeric_limits<double>::quiet_NaN();
        std::nth_element(samples.begin(), samples.begin() + samples.size() / 2, samples.end());
        return samples[samples.size() / 2];
    }

    void collectNearDepth(const cv::Mat& depth, int u, int v, std::vector<double>& samples) const {
        const double d = readDepthMeters(depth, u, v);
        if (std::isfinite(d) && d > depth_min_ && d < depth_near_threshold_) {
            samples.push_back(d);
        }
    }

    FrameGoals makeGoals(const FrameDetection& det) const {
        FrameGoals goals;
        const auto& pose = latest_odom_.pose.pose;
        const Eigen::Vector3d odom_pos(pose.position.x, pose.position.y, pose.position.z);
        goals.yaw = yawFromQuat(pose.orientation);

        const Eigen::Vector3d forward(std::cos(goals.yaw), std::sin(goals.yaw), 0.0);
        const Eigen::Vector3d right(std::sin(goals.yaw), -std::cos(goals.yaw), 0.0);

        goals.center = odom_pos + det.dist * forward + det.err_y * right;
        goals.center.z() = odom_pos.z() + det.err_z;
        goals.pre = goals.center - pre_offset_ * forward;
        goals.post = goals.center + post_offset_ * forward;
        return goals;
    }

    std::string outputFrame() const {
        if (!configured_frame_id_.empty()) return configured_frame_id_;
        if (have_odom_ && !latest_odom_.header.frame_id.empty()) return latest_odom_.header.frame_id;
        return "world";
    }

    geometry_msgs::PoseStamped makePose(const Eigen::Vector3d& p,
                                        double yaw,
                                        const std::string& frame_id,
                                        const ros::Time& stamp) const {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = p.z();
        pose.pose.orientation = yawToQuat(yaw);
        return pose;
    }

    void publishOutputs(const FrameGoals& goals, const std::string& frame_id, const ros::Time& stamp) {
        geometry_msgs::PointStamped center;
        center.header.stamp = stamp;
        center.header.frame_id = frame_id;
        center.point.x = goals.center.x();
        center.point.y = goals.center.y();
        center.point.z = goals.center.z();
        center_pub_.publish(center);
        pre_pub_.publish(makePose(goals.pre, goals.yaw, frame_id, stamp));
        post_pub_.publish(makePose(goals.post, goals.yaw, frame_id, stamp));
    }

    visualization_msgs::Marker makeSphereMarker(int id,
                                                const Eigen::Vector3d& p,
                                                const std::string& frame_id,
                                                const ros::Time& stamp,
                                                float r,
                                                float g,
                                                float b,
                                                double scale,
                                                const std::string& ns) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = p.x();
        marker.pose.position.y = p.y();
        marker.pose.position.z = p.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.9f;
        return marker;
    }

    visualization_msgs::Marker makeLineMarker(int id,
                                              const Eigen::Vector3d& a,
                                              const Eigen::Vector3d& b,
                                              const std::string& frame_id,
                                              const ros::Time& stamp) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = "frame_debug_line";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.035;
        marker.color.r = 0.1f;
        marker.color.g = 1.0f;
        marker.color.b = 0.3f;
        marker.color.a = 0.9f;
        marker.points.push_back(toPoint(a));
        marker.points.push_back(toPoint(b));
        return marker;
    }

    geometry_msgs::Point toPoint(const Eigen::Vector3d& p) const {
        geometry_msgs::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        return point;
    }

    void publishMarkers(const FrameGoals& goals, const std::string& frame_id, const ros::Time& stamp) {
        visualization_msgs::MarkerArray arr;
        addDeleteMarker(arr, frame_id, stamp);
        arr.markers.push_back(makeSphereMarker(1, goals.center, frame_id, stamp, 0.1f, 0.9f, 1.0f, 0.18, "frame_center"));
        arr.markers.push_back(makeSphereMarker(2, goals.pre, frame_id, stamp, 0.2f, 0.6f, 1.0f, 0.16, "frame_goals"));
        arr.markers.push_back(makeSphereMarker(3, goals.post, frame_id, stamp, 0.2f, 1.0f, 0.3f, 0.16, "frame_goals"));
        arr.markers.push_back(makeLineMarker(20, goals.pre, goals.post, frame_id, stamp));
        marker_pub_.publish(arr);
    }

    void clearMarkers(const std::string& frame_id, const ros::Time& stamp) {
        visualization_msgs::MarkerArray arr;
        addDeleteMarker(arr, frame_id, stamp);
        marker_pub_.publish(arr);
    }

    void addDeleteMarker(visualization_msgs::MarkerArray& arr,
                         const std::string& frame_id,
                         const ros::Time& stamp) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(marker);
    }

    void publishDebugImage(const cv::Mat& depth,
                           const FrameDetection& det,
                           const std_msgs::Header& header) {
        if (debug_image_pub_.getNumSubscribers() == 0) return;

        cv::Mat debug(depth.rows, depth.cols, CV_8UC3, cv::Scalar(20, 20, 20));
        for (int v = 0; v < depth.rows; ++v) {
            for (int u = 0; u < depth.cols; ++u) {
                const double d = readDepthMeters(depth, u, v);
                if (!std::isfinite(d)) {
                    debug.at<cv::Vec3b>(v, u) = cv::Vec3b(40, 40, 40);
                } else if (d < depth_min_) {
                    debug.at<cv::Vec3b>(v, u) = cv::Vec3b(20, 20, 90);
                } else if (d < depth_near_threshold_) {
                    const int value = clampInt(static_cast<int>(180.0 * (d - depth_min_) /
                                                                std::max(0.01, depth_near_threshold_ - depth_min_)),
                                               30, 180);
                    debug.at<cv::Vec3b>(v, u) = cv::Vec3b(value, value, value);
                } else if (d < depth_max_) {
                    debug.at<cv::Vec3b>(v, u) = cv::Vec3b(45, 120, 45);
                } else {
                    debug.at<cv::Vec3b>(v, u) = cv::Vec3b(80, 30, 30);
                }
            }
        }

        const int x_margin = clampInt(static_cast<int>(depth.cols * roi_x_margin_ratio_), 0, depth.cols / 2 - 1);
        const int y_top = clampInt(static_cast<int>(depth.rows * roi_y_top_ratio_), 0, depth.rows - 1);
        const int y_bottom_margin = clampInt(static_cast<int>(depth.rows * roi_y_bottom_ratio_), 0, depth.rows - 1);
        cv::rectangle(debug,
                      cv::Rect(cv::Point(x_margin, y_top),
                               cv::Point(depth.cols - x_margin - 1, depth.rows - y_bottom_margin - 1)),
                      cv::Scalar(255, 180, 0), 1);

        if (det.found) {
            cv::rectangle(debug,
                          cv::Rect(cv::Point(det.u0, det.v0), cv::Point(det.u1, det.v1)),
                          cv::Scalar(0, 255, 0), 2);
            cv::circle(debug, cv::Point((det.u0 + det.u1) / 2, (det.v0 + det.v1) / 2),
                       4, cv::Scalar(0, 255, 255), -1);
            std::ostringstream label;
            label << "dy=" << det.err_y << " dz=" << det.err_z << " d=" << det.dist;
            cv::putText(debug, label.str(), cv::Point(det.u0, std::max(20, det.v0 - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1);
        }

        cv_bridge::CvImage out;
        out.header = header;
        out.encoding = "bgr8";
        out.image = debug;
        debug_image_pub_.publish(out.toImageMsg());
    }

    void publishStatus(const std::string& status) {
        if (status == last_status_ && (ros::Time::now() - last_status_time_).toSec() < 0.2) {
            return;
        }
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
        last_status_ = status;
        last_status_time_ = ros::Time::now();
    }

    void healthTimerCb(const ros::TimerEvent&) {
        if (!have_info_) {
            publishStatus("waiting_camera_info");
            return;
        }
        if (!have_depth_ ||
            (ros::Time::now() - last_depth_stamp_).toSec() > depth_stale_timeout_) {
            publishStatus("no_depth");
            return;
        }
        if (!have_odom_) {
            publishStatus("waiting_odom");
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber info_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher status_pub_;
    ros::Publisher center_pub_;
    ros::Publisher pre_pub_;
    ros::Publisher post_pub_;
    ros::Publisher marker_pub_;
    image_transport::Publisher debug_image_pub_;
    ros::Timer health_timer_;

    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string odom_topic_;
    std::string configured_frame_id_;
    double depth_min_ = 0.35;
    double depth_near_threshold_ = 1.8;
    double depth_max_ = 3.0;
    int min_open_width_px_ = 70;
    int min_open_height_px_ = 70;
    double roi_x_margin_ratio_ = 0.20;
    double roi_y_top_ratio_ = 0.20;
    double roi_y_bottom_ratio_ = 0.10;
    double row_open_ratio_ = 0.65;
    double pre_offset_ = 0.60;
    double post_offset_ = 0.90;
    double debug_log_period_ = 1.0;
    double depth_stale_timeout_ = 1.0;
    bool publish_debug_image_ = true;

    bool have_info_ = false;
    bool have_odom_ = false;
    bool have_depth_ = false;
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    nav_msgs::Odometry latest_odom_;
    ros::Time last_depth_stamp_;
    std::string last_status_;
    ros::Time last_status_time_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "frame_debug_node");
    FrameDebugNode node;
    ros::spin();
    return 0;
}
