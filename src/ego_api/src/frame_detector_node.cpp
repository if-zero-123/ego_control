#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

double clampDouble(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

double quantileFromSorted(const std::vector<double>& sorted_values, double q) {
    if (sorted_values.empty()) return 0.0;
    if (sorted_values.size() == 1) return sorted_values.front();
    const double clamped_q = clampDouble(q, 0.0, 1.0);
    const double pos = clamped_q * static_cast<double>(sorted_values.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = std::min(sorted_values.size() - 1, lo + 1);
    const double t = pos - static_cast<double>(lo);
    return sorted_values[lo] * (1.0 - t) + sorted_values[hi] * t;
}

double normalizeYaw(double yaw) {
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    return yaw;
}

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

struct GatePostCandidate {
    Eigen::Vector3d min_pt = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_pt = Eigen::Vector3d::Zero();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector2d local_center = Eigen::Vector2d::Zero();
    double local_forward_width = 0.0;
    double local_lateral_width = 0.0;
    double height = 0.0;
    int points = 0;
    int vertical_bins = 0;
    double low_z = 0.0;
    double high_z = 0.0;
    double z_span = 0.0;
    double confidence = 0.0;
    bool accepted = false;
    bool strong = false;
    bool weak = false;
    std::string reason = "unclassified";
};

struct ClusterDebugInfo {
    int points = 0;
    double local_x = 0.0;
    double local_y = 0.0;
    double width_forward = 0.0;
    double width_lateral = 0.0;
    double height = 0.0;
    int vertical_bins = 0;
    bool accepted = false;
    bool strong = false;
    bool weak = false;
    std::string reason;
};

struct GateDetection {
    bool valid = false;
    Eigen::Vector3d left = Eigen::Vector3d::Zero();
    Eigen::Vector3d right = Eigen::Vector3d::Zero();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d pre_goal = Eigen::Vector3d::Zero();
    Eigen::Vector3d post_goal = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double gap = 0.0;
    bool partial = false;
    bool weak_pair = false;
    bool split_frame = false;
    bool frame_cluster = false;
    std::string height_source = "none";
    double height_low_z = 0.0;
    double height_high_z = 0.0;
    double height_span = 0.0;
    int height_points = 0;
};

struct HeightEstimate {
    bool valid = false;
    double center_z = 0.0;
    double low_z = 0.0;
    double high_z = 0.0;
    double span = 0.0;
    int points = 0;
    std::string source = "none";
};

struct FrameClusterCandidate {
    GatePostCandidate whole;
    GatePostCandidate left;
    GatePostCandidate right;
    double score = -std::numeric_limits<double>::infinity();
};

class FrameDetectorNode {
public:
    FrameDetectorNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
        pnh_.param<std::string>("frame_id", configured_frame_id_, "");
        pnh_.param<std::string>("goal_frame_id", goal_frame_id_, "world");
        pnh_.param<double>("flight_z", flight_z_, 0.9);
        pnh_.param<std::string>("height_mode", height_mode_, "auto");
        pnh_.param<double>("gate_center_z", gate_center_z_, flight_z_);
        pnh_.param<double>("crossing_z", crossing_z_, flight_z_);
        pnh_.param<bool>("use_detected_center_z", use_detected_center_z_, true);
        pnh_.param<bool>("goal_z_follows_center", goal_z_follows_center_, true);
        pnh_.param<double>("detected_center_z_bias", detected_center_z_bias_, 0.0);
        pnh_.param<double>("goal_z_bias", goal_z_bias_, 0.0);
        pnh_.param<bool>("clamp_detected_center_z", clamp_detected_center_z_, false);
        pnh_.param<double>("center_z_min", center_z_min_, 0.45);
        pnh_.param<double>("center_z_max", center_z_max_, 1.20);
        pnh_.param<std::string>("auto_height_bottom_mode", auto_height_bottom_mode_, "detected");
        pnh_.param<double>("gate_bottom_z", gate_bottom_z_, 0.15);
        pnh_.param<double>("auto_height_top_z_max", auto_height_top_z_max_, 2.20);
        pnh_.param<double>("auto_height_forward_tolerance", auto_height_forward_tolerance_, 0.35);
        pnh_.param<double>("auto_height_lateral_margin", auto_height_lateral_margin_, 0.18);
        pnh_.param<double>("auto_height_post_forward_margin", auto_height_post_forward_margin_, 0.12);
        pnh_.param<double>("auto_height_post_lateral_margin", auto_height_post_lateral_margin_, 0.12);
        pnh_.param<double>("auto_height_low_quantile", auto_height_low_quantile_, 0.05);
        pnh_.param<double>("auto_height_high_quantile", auto_height_high_quantile_, 0.95);
        pnh_.param<double>("auto_height_min_span", auto_height_min_span_, 0.45);
        pnh_.param<int>("auto_height_min_points", auto_height_min_points_, 18);

        pnh_.param<bool>("use_local_rule_roi", use_local_rule_roi_, true);
        pnh_.param<double>("field_length", field_length_, 6.0);
        pnh_.param<double>("field_width", field_width_, 5.0);
        pnh_.param<double>("gate_search_x_min", gate_search_x_min_, -0.50);
        pnh_.param<double>("gate_search_x_max", gate_search_x_max_, 4.20);
        pnh_.param<double>("gate_search_y_min", gate_search_y_min_, -2.50);
        pnh_.param<double>("gate_search_y_max", gate_search_y_max_, 2.50);

        pnh_.param<double>("roi_forward_min", roi_forward_min_, 0.20);
        pnh_.param<double>("roi_forward_max", roi_forward_max_, 5.00);
        pnh_.param<double>("roi_lateral_abs", roi_lateral_abs_, 2.50);
        pnh_.param<double>("roi_z_min", roi_z_min_, 0.15);
        pnh_.param<double>("roi_z_max", roi_z_max_, 2.20);
        pnh_.param<double>("post_extract_z_max", post_extract_z_max_, 2.00);
        pnh_.param<bool>("use_fixed_center_roi", use_fixed_center_roi_, true);
        pnh_.param<double>("fixed_roi_center_x", fixed_roi_center_x_, 2.70);
        pnh_.param<double>("fixed_roi_center_y", fixed_roi_center_y_, -1.35);
        pnh_.param<double>("fixed_roi_center_z", fixed_roi_center_z_, 1.30);
        pnh_.param<double>("fixed_roi_radius", fixed_roi_radius_, 2.65);
        pnh_.param<bool>("use_body_exclusion", use_body_exclusion_, false);
        pnh_.param<double>("body_exclusion_forward_min", body_exclusion_forward_min_, -0.55);
        pnh_.param<double>("body_exclusion_forward_max", body_exclusion_forward_max_, 0.45);
        pnh_.param<double>("body_exclusion_lateral_abs", body_exclusion_lateral_abs_, 0.45);
        pnh_.param<double>("body_exclusion_z_min", body_exclusion_z_min_, -0.45);
        pnh_.param<double>("body_exclusion_z_max", body_exclusion_z_max_, 0.35);

        pnh_.param<double>("voxel_leaf", voxel_leaf_, 0.05);
        pnh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.22);
        pnh_.param<int>("min_cluster_size", min_cluster_size_, 8);
        pnh_.param<int>("max_cluster_size", max_cluster_size_, 5000);

        pnh_.param<double>("gate_post_min_width", gate_post_min_width_, 0.05);
        pnh_.param<double>("gate_post_max_width", gate_post_max_width_, 0.42);
        pnh_.param<double>("gate_post_weak_max_width", gate_post_weak_max_width_, 0.55);
        pnh_.param<double>("gate_frame_cluster_min_width", gate_frame_cluster_min_width_, 0.75);
        pnh_.param<double>("gate_frame_cluster_max_width", gate_frame_cluster_max_width_, 1.40);
        pnh_.param<bool>("prefer_frame_cluster", prefer_frame_cluster_, true);
        pnh_.param<double>("frame_cluster_forward_width_min", frame_cluster_forward_width_min_, 0.02);
        pnh_.param<double>("frame_cluster_forward_width_max", frame_cluster_forward_width_max_, 0.30);
        pnh_.param<double>("frame_cluster_min_height", frame_cluster_min_height_, 0.80);
        pnh_.param<double>("frame_cluster_max_height", frame_cluster_max_height_, 2.20);
        pnh_.param<int>("frame_cluster_min_points", frame_cluster_min_points_, 80);
        pnh_.param<double>("gate_split_side_band", gate_split_side_band_, 0.22);
        pnh_.param<double>("gate_post_min_height", gate_post_min_height_, 0.40);
        pnh_.param<double>("gate_post_weak_min_height", gate_post_weak_min_height_, 0.18);
        pnh_.param<double>("vertical_bin_size", vertical_bin_size_, 0.18);
        pnh_.param<int>("gate_post_min_vertical_bins", gate_post_min_vertical_bins_, 2);
        pnh_.param<int>("gate_post_weak_min_vertical_bins", gate_post_weak_min_vertical_bins_, 2);

        pnh_.param<double>("gate_width_min", gate_width_min_, 0.85);
        pnh_.param<double>("gate_width_max", gate_width_max_, 1.35);
        pnh_.param<double>("gate_width_expected", gate_width_expected_, 1.00);
        pnh_.param<double>("pair_forward_tolerance", pair_forward_tolerance_, 0.35);
        pnh_.param<bool>("enforce_gate_center_constraint", enforce_gate_center_constraint_, true);
        pnh_.param<double>("gate_center_x", gate_center_x_, fixed_roi_center_x_);
        pnh_.param<double>("gate_center_y", gate_center_y_, fixed_roi_center_y_);
        pnh_.param<std::string>("gate_center_y_frame", gate_center_y_frame_, "local");
        pnh_.param<double>("gate_center_y_tolerance", gate_center_y_tolerance_, 0.45);
        pnh_.param<double>("partial_center_y_tolerance", partial_center_y_tolerance_, 0.65);
        pnh_.param<double>("pair_max_height_delta", pair_max_height_delta_, 0.60);
        pnh_.param<double>("pair_min_height_overlap", pair_min_height_overlap_, 0.15);
        pnh_.param<bool>("allow_weak_single_partial", allow_weak_single_partial_, true);

        pnh_.param<double>("pre_offset", pre_offset_, 0.70);
        pnh_.param<double>("post_offset", post_offset_, 1.00);
        pnh_.param<double>("stability_threshold", stability_threshold_, 0.18);
        pnh_.param<int>("stable_frames", stable_frames_, 3);
        pnh_.param<int>("history_size", history_size_, 5);
        pnh_.param<double>("debug_log_period", debug_log_period_, 1.0);
        pnh_.param<int>("debug_top_clusters", debug_top_clusters_, 6);
        pnh_.param<bool>("readable_logs", readable_logs_, false);
        pnh_.param<double>("readable_log_period", readable_log_period_, 2.0);

        cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &FrameDetectorNode::cloudCb, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20, &FrameDetectorNode::odomCb, this);

        status_pub_ = nh_.advertise<std_msgs::String>("/craic/frame_status", 1, true);
        center_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/craic/frame_center", 1, true);
        pre_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic/frame_pre_goal", 1, true);
        post_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic/frame_post_goal", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/craic/frame_markers", 1, true);

        if (!readable_logs_) {
            ROS_INFO("[frame_detector] cloud=%s odom=%s marker_frame=%s goal_frame=%s flight_z=%.2f height_mode=%s gate_center_z=%.2f crossing_z=%.2f detected_z=%d goal_z_follows_center=%d center_z_bias=%.2f goal_z_bias=%.2f clamp_z=%d center_z_safety=[%.2f %.2f] bottom_mode=%s gate_bottom_z=%.2f auto_top_z_max=%.2f auto_gate_tol=%.2f auto_post_margin=(%.2f %.2f) q=[%.2f %.2f] min_span=%.2f min_pts=%d local_rule_roi=%d field=%.2fx%.2f gate_roi=x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f] post_z_max=%.2f fallback=front[%.2f %.2f] lat=+/-%.2f weak_single=%d",
                     cloud_topic_.c_str(), odom_topic_.c_str(),
                     configured_frame_id_.empty() ? "<cloud>" : configured_frame_id_.c_str(),
                     goal_frame_id_.c_str(), flight_z_,
                     height_mode_.c_str(), gate_center_z_, crossing_z_,
                     use_detected_center_z_, goal_z_follows_center_,
                     detected_center_z_bias_, goal_z_bias_, clamp_detected_center_z_,
                     center_z_min_, center_z_max_, auto_height_bottom_mode_.c_str(), gate_bottom_z_,
                     auto_height_top_z_max_,
                     auto_height_forward_tolerance_, auto_height_post_forward_margin_,
                     auto_height_post_lateral_margin_, auto_height_low_quantile_,
                     auto_height_high_quantile_, auto_height_min_span_, auto_height_min_points_,
                     use_local_rule_roi_,
                     field_length_, field_width_, gate_search_x_min_, gate_search_x_max_,
                     gate_search_y_min_, gate_search_y_max_, roi_z_min_, roi_z_max_, post_extract_z_max_,
                     roi_forward_min_, roi_forward_max_, roi_lateral_abs_, allow_weak_single_partial_);
            ROS_INFO("[frame_detector] fixed_roi=%d center=(%.2f %.2f %.2f) radius=%.2f body_exclusion=%d body_box=x[%.2f %.2f] y=+/-%.2f dz[%.2f %.2f]",
                     use_fixed_center_roi_, fixed_roi_center_x_, fixed_roi_center_y_,
                     fixed_roi_center_z_, fixed_roi_radius_,
                     use_body_exclusion_, body_exclusion_forward_min_, body_exclusion_forward_max_,
                     body_exclusion_lateral_abs_, body_exclusion_z_min_, body_exclusion_z_max_);
            ROS_INFO("[frame_detector] cluster leaf=%.2f tol=%.2f min=%d post_width=[%.2f %.2f] weak_width<=%.2f frame_width=[%.2f %.2f] frame_forward=[%.2f %.2f] frame_h=[%.2f %.2f] frame_min_pts=%d prefer_frame=%d post_h>=%.2f weak_h>=%.2f gate_width=[%.2f %.2f] forward_tol=%.2f center=(%.2f %.2f) center_y_frame=%s effective_center_y=%.2f center_constraint=%d center_y_tol=%.2f partial_y_tol=%.2f pair_height_delta<=%.2f pair_height_overlap>=%.2f",
                     voxel_leaf_, cluster_tolerance_, min_cluster_size_,
                     gate_post_min_width_, gate_post_max_width_, gate_post_weak_max_width_,
                     gate_frame_cluster_min_width_, gate_frame_cluster_max_width_,
                     frame_cluster_forward_width_min_, frame_cluster_forward_width_max_,
                     frame_cluster_min_height_, frame_cluster_max_height_, frame_cluster_min_points_,
                     prefer_frame_cluster_,
                     gate_post_min_height_, gate_post_weak_min_height_,
                     gate_width_min_, gate_width_max_, pair_forward_tolerance_,
                     gate_center_x_, gate_center_y_, gate_center_y_frame_.c_str(), gateCenterY(),
                     enforce_gate_center_constraint_, gate_center_y_tolerance_,
                     partial_center_y_tolerance_, pair_max_height_delta_, pair_min_height_overlap_);
        }
    }

private:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
        odom_yaw_ = yawFromQuat(msg->pose.pose.orientation);
        odom_frame_id_ = msg->header.frame_id;
        have_odom_ = true;

        if (!have_home_) {
            home_pos_ = odom_pos_;
            home_yaw_ = odom_yaw_;
            have_home_ = true;
            if (!readable_logs_) {
                ROS_INFO("[frame_detector] home locked at (%.2f %.2f %.2f), yaw=%.2f",
                         home_pos_.x(), home_pos_.y(), home_pos_.z(), home_yaw_);
            }
        }
    }

    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        const std::string frame_id = outputFrame(*msg);
        if (!have_odom_) {
            publishStatus("waiting_odom");
            clearMarkers(frame_id, msg->header.stamp);
            if (readable_logs_) {
                logFrame("waiting_odom", *msg, 0, 0, 0, 0, 0, 0,
                         std::vector<ClusterDebugInfo>(), nullptr);
            } else {
                ROS_WARN_THROTTLE(debug_log_period_, "[frame_detector] waiting_odom");
            }
            return;
        }
        if (use_local_rule_roi_ && !have_home_) {
            publishStatus("waiting_odom");
            clearMarkers(frame_id, msg->header.stamp);
            if (readable_logs_) {
                logFrame("waiting_home", *msg, 0, 0, 0, 0, 0, 0,
                         std::vector<ClusterDebugInfo>(), nullptr);
            } else {
                ROS_WARN_THROTTLE(debug_log_period_, "[frame_detector] waiting_odom for local rule ROI");
            }
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *input);
        if (input->empty()) {
            publishStatus("no_cloud");
            clearMarkers(frame_id, msg->header.stamp);
            if (readable_logs_) {
                logFrame("no_cloud", *msg, 0, 0, 0, 0, 0, 0,
                         std::vector<ClusterDebugInfo>(), nullptr);
            } else {
                ROS_WARN_THROTTLE(debug_log_period_, "[frame_detector] no_cloud on %s", cloud_topic_.c_str());
            }
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(input);
        voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        voxel.filter(*down);

        pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr post_roi(new pcl::PointCloud<pcl::PointXYZ>());
        roi->header = down->header;
        post_roi->header = down->header;
        for (const auto& pt : down->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            const Eigen::Vector3d world(pt.x, pt.y, pt.z);
            if (!inFixedCenterRoi(world)) continue;
            if (inBodyExclusion(world)) continue;
            if (!inSearchRoi(world)) continue;
            if (world.z() < roi_z_min_ || world.z() > roi_z_max_) continue;
            roi->points.push_back(pt);
            if (world.z() <= post_extract_z_max_) {
                post_roi->points.push_back(pt);
            }
        }
        finishCloud(*roi);
        finishCloud(*post_roi);

        if (roi->empty()) {
            publishStatus("empty_roi");
            debug_candidates_.clear();
            publishCandidateMarkers(frame_id, msg->header.stamp);
            if (readable_logs_) {
                logFrame("empty_roi", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                         0, 0, std::vector<ClusterDebugInfo>(), nullptr);
            } else {
                ROS_WARN_THROTTLE(debug_log_period_,
                                  "[frame_detector] empty_roi input=%zu down=%zu roi=0 local_rule=%d gate_roi=x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]",
                                  input->size(), down->size(), use_local_rule_roi_,
                                  gate_search_x_min_, gate_search_x_max_, gate_search_y_min_, gate_search_y_max_,
                                  roi_z_min_, roi_z_max_);
            }
            return;
        }
        if (post_roi->empty()) {
            publishStatus("empty_post_roi");
            debug_candidates_.clear();
            publishCandidateMarkers(frame_id, msg->header.stamp);
            if (readable_logs_) {
                logFrame("empty_post_roi", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                         0, 0, std::vector<ClusterDebugInfo>(), nullptr);
            } else {
                ROS_WARN_THROTTLE(debug_log_period_,
                                  "[frame_detector] empty_post_roi input=%zu down=%zu roi=%zu",
                                  input->size(), down->size(), roi->size());
            }
            return;
        }

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(post_roi);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(post_roi);
        ec.extract(cluster_indices);

        std::vector<GatePostCandidate> candidates;
        std::vector<GatePostCandidate> split_candidates;
        std::vector<FrameClusterCandidate> frame_clusters;
        std::vector<ClusterDebugInfo> cluster_debug;
        for (const auto& indices : cluster_indices) {
            GatePostCandidate candidate;
            if (!buildCandidate(*post_roi, indices, candidate)) continue;
            classifyCandidate(candidate);
            bool add_candidate_to_pairs = candidate.accepted;
            if (!candidate.accepted) {
                FrameClusterCandidate frame_cluster;
                if (buildFrameClusterCandidate(*post_roi, indices, frame_cluster)) {
                    frame_clusters.push_back(frame_cluster);
                    split_candidates.push_back(frame_cluster.left);
                    split_candidates.push_back(frame_cluster.right);
                    candidate.accepted = true;
                    candidate.strong = true;
                    candidate.weak = false;
                    candidate.reason = "accepted_frame_cluster";
                    candidate.confidence = frame_cluster.score;
                    add_candidate_to_pairs = false;
                } else {
                    GatePostCandidate split_left;
                    GatePostCandidate split_right;
                    if (splitWideFrameCluster(*post_roi, indices, split_left, split_right)) {
                        split_candidates.push_back(split_left);
                        split_candidates.push_back(split_right);
                    }
                }
            }
            cluster_debug.push_back(makeDebugInfo(candidate));
            if (add_candidate_to_pairs) {
                candidates.push_back(candidate);
            }
        }
        sortDebug(cluster_debug);
        std::vector<GatePostCandidate> all_debug_candidates = candidates;
        all_debug_candidates.insert(all_debug_candidates.end(), split_candidates.begin(), split_candidates.end());
        for (const auto& frame_cluster : frame_clusters) {
            all_debug_candidates.push_back(frame_cluster.whole);
        }
        cacheCandidateCenters(all_debug_candidates);

        GatePostCandidate left_post;
        GatePostCandidate right_post;
        double pair_score = 0.0;

        if (prefer_frame_cluster_ &&
            selectBestFrameCluster(frame_clusters, left_post, right_post, pair_score)) {
            GateDetection detection = makeFullDetection(left_post, right_post, *post_roi);
            detection.frame_cluster = true;
            if (!detection.valid) {
                publishStatus("invalid_frame_cluster_geometry");
                publishCandidateMarkers(frame_id, msg->header.stamp);
                logFrame("invalid_frame_cluster_geometry", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                         cluster_indices.size(), candidates.size() + split_candidates.size(), cluster_debug, nullptr);
                return;
            }

            history_.push_back(detection);
            while (static_cast<int>(history_.size()) > history_size_) {
                history_.pop_front();
            }

            if (!isStable()) {
                publishStatus("stabilizing_frame_cluster");
                publishMarkers(detection, frame_id, msg->header.stamp, false);
                logFrame("stabilizing_frame_cluster", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                         cluster_indices.size(), candidates.size() + split_candidates.size(), cluster_debug,
                         &detection, pair_score);
                return;
            }

            latest_ = averageHistory();
            latest_.frame_cluster = true;
            publishOutputs(latest_, msg->header.stamp);
            publishMarkers(latest_, frame_id, msg->header.stamp, true);
            publishStatus("valid_frame_cluster");
            logFrame("valid_frame_cluster", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                     cluster_indices.size(), candidates.size() + split_candidates.size(), cluster_debug,
                     &latest_, pair_score);
            return;
        }

        if (candidates.size() >= 2 &&
            selectBestPair(candidates, left_post, right_post, pair_score, false)) {
            GateDetection detection = makeFullDetection(left_post, right_post, *post_roi);
            if (!detection.valid) {
                publishStatus("invalid_geometry");
                publishCandidateMarkers(frame_id, msg->header.stamp);
                logFrame("invalid_geometry", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                         cluster_indices.size(), candidates.size(), cluster_debug, nullptr);
                return;
            }

            history_.push_back(detection);
            while (static_cast<int>(history_.size()) > history_size_) {
                history_.pop_front();
            }

            if (!isStable()) {
                publishStatus("stabilizing_full");
                publishMarkers(detection, frame_id, msg->header.stamp, false);
                logFrame("stabilizing_full", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                         cluster_indices.size(), candidates.size(), cluster_debug, &detection, pair_score);
                return;
            }

            latest_ = averageHistory();
            publishOutputs(latest_, msg->header.stamp);
            publishMarkers(latest_, frame_id, msg->header.stamp, true);
            publishStatus("valid_full");
            logFrame("valid_full", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                     cluster_indices.size(), candidates.size(), cluster_debug, &latest_, pair_score);
            return;
        }

        if (split_candidates.size() >= 2 &&
            selectBestPair(split_candidates, left_post, right_post, pair_score, false)) {
            GateDetection detection = makeFullDetection(left_post, right_post, *post_roi);
            detection.split_frame = true;
            if (detection.valid) {
                history_.clear();
                publishOutputs(detection, msg->header.stamp);
                publishMarkers(detection, frame_id, msg->header.stamp, true);
                publishStatus("valid_split_frame");
                logFrame("valid_split_frame", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                         cluster_indices.size(), candidates.size() + split_candidates.size(), cluster_debug,
                         &detection, pair_score);
                return;
            }
        }

        if (candidates.size() >= 2 &&
            selectBestPair(candidates, left_post, right_post, pair_score, true)) {
            GateDetection detection = makeFullDetection(left_post, right_post, *post_roi);
            detection.weak_pair = true;
            if (!detection.valid) {
                publishStatus("invalid_weak_pair_geometry");
                publishCandidateMarkers(frame_id, msg->header.stamp);
                logFrame("invalid_weak_pair_geometry", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                         cluster_indices.size(), candidates.size(), cluster_debug, nullptr);
                return;
            }

            history_.clear();
            publishOutputs(detection, msg->header.stamp);
            publishMarkers(detection, frame_id, msg->header.stamp, true);
            publishStatus("valid_weak_pair");
            logFrame("valid_weak_pair", *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                     cluster_indices.size(), candidates.size(), cluster_debug, &detection, pair_score);
            return;
        }

        GatePostCandidate single_post;
        if (selectBestSingle(candidates, single_post)) {
            GateDetection detection = makePartialDetection(single_post, *post_roi);
            if (detection.valid) {
                history_.clear();
                const std::string status = single_post.local_center.y() >= gateCenterY()
                                               ? "valid_partial_left"
                                               : "valid_partial_right";
                publishOutputs(detection, msg->header.stamp);
                publishMarkers(detection, frame_id, msg->header.stamp, true);
                publishStatus(status);
                logFrame(status, *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                         cluster_indices.size(), candidates.size(), cluster_debug, &detection, single_post.confidence);
                return;
            }
        }

        publishStatus(candidates.empty() ? "not_enough_candidates" : "invalid_pair");
        publishCandidateMarkers(frame_id, msg->header.stamp);
        logFrame(candidates.empty() ? "not_enough_candidates" : "invalid_pair",
                 *msg, input->size(), down->size(), roi->size(), post_roi->size(),
                 cluster_indices.size(), candidates.size(), cluster_debug, nullptr);
    }

    void finishCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) const {
        cloud.width = static_cast<uint32_t>(cloud.points.size());
        cloud.height = 1;
        cloud.is_dense = false;
    }

    bool inSearchRoi(const Eigen::Vector3d& world) const {
        const Eigen::Vector2d local = worldToLocalXY(world);
        if (use_local_rule_roi_) {
            return local.x() >= gate_search_x_min_ &&
                   local.x() <= gate_search_x_max_ &&
                   local.y() >= gate_search_y_min_ &&
                   local.y() <= gate_search_y_max_;
        }
        return local.x() >= roi_forward_min_ &&
               local.x() <= roi_forward_max_ &&
               std::abs(local.y()) <= roi_lateral_abs_;
    }

    bool inFixedCenterRoi(const Eigen::Vector3d& world) const {
        if (!use_fixed_center_roi_) return true;
        const Eigen::Vector3d center(fixed_roi_center_x_,
                                     fixed_roi_center_y_,
                                     fixed_roi_center_z_);
        return (world - center).squaredNorm() <= fixed_roi_radius_ * fixed_roi_radius_;
    }

    bool inBodyExclusion(const Eigen::Vector3d& world) const {
        if (!use_body_exclusion_) return false;
        const Eigen::Vector2d body = worldToBodyLocalXY(world);
        const double dz = world.z() - odom_pos_.z();
        return body.x() >= body_exclusion_forward_min_ &&
               body.x() <= body_exclusion_forward_max_ &&
               std::abs(body.y()) <= body_exclusion_lateral_abs_ &&
               dz >= body_exclusion_z_min_ &&
               dz <= body_exclusion_z_max_;
    }

    Eigen::Vector2d worldToLocalXY(const Eigen::Vector3d& world) const {
        return use_local_rule_roi_ ? worldToFieldLocalXY(world) : worldToBodyLocalXY(world);
    }

    Eigen::Vector2d worldToFieldLocalXY(const Eigen::Vector3d& world) const {
        const double dx = world.x() - home_pos_.x();
        const double dy = world.y() - home_pos_.y();
        const double c = std::cos(home_yaw_);
        const double s = std::sin(home_yaw_);
        return Eigen::Vector2d(c * dx + s * dy, -s * dx + c * dy);
    }

    Eigen::Vector2d worldToBodyLocalXY(const Eigen::Vector3d& world) const {
        const double dx = world.x() - odom_pos_.x();
        const double dy = world.y() - odom_pos_.y();
        const double c = std::cos(odom_yaw_);
        const double s = std::sin(odom_yaw_);
        return Eigen::Vector2d(c * dx + s * dy, -s * dx + c * dy);
    }

    Eigen::Vector3d localToWorld(double local_x, double local_y, double z) const {
        return use_local_rule_roi_
                   ? fieldLocalToWorldXY(local_x, local_y, z)
                   : bodyLocalToWorldXY(local_x, local_y, z);
    }

    Eigen::Vector3d fieldLocalToWorldXY(double local_x, double local_y, double z) const {
        const double c = std::cos(home_yaw_);
        const double s = std::sin(home_yaw_);
        return Eigen::Vector3d(home_pos_.x() + c * local_x - s * local_y,
                               home_pos_.y() + s * local_x + c * local_y,
                               z);
    }

    Eigen::Vector3d bodyLocalToWorldXY(double local_x, double local_y, double z) const {
        const double c = std::cos(odom_yaw_);
        const double s = std::sin(odom_yaw_);
        return Eigen::Vector3d(odom_pos_.x() + c * local_x - s * local_y,
                               odom_pos_.y() + s * local_x + c * local_y,
                               z);
    }

    double gateSearchCenterY() const {
        return 0.5 * (gate_search_y_min_ + gate_search_y_max_);
    }

    double gateCenterY() const {
        if (std::isfinite(gate_center_y_)) {
            if (gate_center_y_frame_ == "world" && use_local_rule_roi_ && have_home_) {
                const double world_x = std::isfinite(gate_center_x_) ? gate_center_x_ : home_pos_.x();
                return worldToFieldLocalXY(Eigen::Vector3d(world_x, gate_center_y_, home_pos_.z())).y();
            }
            return gate_center_y_;
        }
        if (use_fixed_center_roi_) return fixed_roi_center_y_;
        return gateSearchCenterY();
    }

    bool passesGateCenterConstraint(double center_y, double tolerance) const {
        if (!enforce_gate_center_constraint_) return true;
        return std::abs(center_y - gateCenterY()) <= tolerance;
    }

    double pairHeightDelta(const GatePostCandidate& a,
                           const GatePostCandidate& b) const {
        const double a_center_z = 0.5 * (a.low_z + a.high_z);
        const double b_center_z = 0.5 * (b.low_z + b.high_z);
        return std::abs(a_center_z - b_center_z);
    }

    bool pairHeightCompatible(const GatePostCandidate& a,
                              const GatePostCandidate& b) const {
        if (pair_max_height_delta_ >= 0.0 &&
            pairHeightDelta(a, b) > pair_max_height_delta_) {
            return false;
        }
        if (pair_min_height_overlap_ <= 0.0) return true;
        const double overlap = std::min(a.high_z, b.high_z) - std::max(a.low_z, b.low_z);
        return overlap >= pair_min_height_overlap_;
    }

    double clampCenterZ(double z) const {
        if (!clamp_detected_center_z_) return z;
        return clampDouble(z, center_z_min_, center_z_max_);
    }

    double autoHeightBottomZ(double detected_low_z) const {
        if (auto_height_bottom_mode_ == "detected") return detected_low_z;
        if (auto_height_bottom_mode_ == "known") return gate_bottom_z_;
        return roi_z_min_;
    }

    HeightEstimate configuredCenterEstimate(const std::string& source) const {
        HeightEstimate estimate;
        estimate.valid = true;
        estimate.center_z = gate_center_z_;
        estimate.low_z = gate_center_z_;
        estimate.high_z = gate_center_z_;
        estimate.source = source;
        return estimate;
    }

    bool usableForAutoHeight(double z) const {
        return z >= roi_z_min_ && z <= auto_height_top_z_max_;
    }

    HeightEstimate makeHeightEstimate(std::vector<double>& z_values,
                                      const std::string& source,
                                      bool require_span) const {
        HeightEstimate estimate;
        estimate.source = source;
        estimate.points = static_cast<int>(z_values.size());
        if (estimate.points < auto_height_min_points_) return estimate;

        std::sort(z_values.begin(), z_values.end());
        estimate.low_z = quantileFromSorted(z_values, auto_height_low_quantile_);
        estimate.high_z = quantileFromSorted(z_values, auto_height_high_quantile_);
        estimate.span = estimate.high_z - estimate.low_z;
        if (require_span && estimate.span < auto_height_min_span_) return estimate;

        const double bottom_z = autoHeightBottomZ(estimate.low_z);
        estimate.center_z = clampCenterZ(0.5 * (bottom_z + estimate.high_z) +
                                         detected_center_z_bias_);
        estimate.valid = true;
        return estimate;
    }

    HeightEstimate makeBoundsHeightEstimate(double low_z,
                                            double high_z,
                                            int points,
                                            const std::string& source,
                                            bool require_span) const {
        HeightEstimate estimate;
        estimate.source = source;
        estimate.points = points;
        if (!std::isfinite(low_z) || !std::isfinite(high_z) || high_z <= low_z) {
            return estimate;
        }

        estimate.low_z = low_z;
        estimate.high_z = high_z;
        estimate.span = high_z - low_z;
        if (require_span && estimate.span < auto_height_min_span_) {
            return estimate;
        }

        estimate.center_z = clampCenterZ(0.5 * (low_z + high_z) + detected_center_z_bias_);
        estimate.valid = true;
        return estimate;
    }

    HeightEstimate candidateHeightEstimate(const GatePostCandidate& post,
                                           const std::string& source) const {
        return makeBoundsHeightEstimate(post.low_z, post.high_z, post.points, source, true);
    }

    HeightEstimate candidateHeightEstimate(const GatePostCandidate& left_post,
                                           const GatePostCandidate& right_post,
                                           const std::string& source) const {
        const int points = left_post.points + right_post.points;
        const double overlap_low_z = std::max(left_post.low_z, right_post.low_z);
        const double overlap_high_z = std::min(left_post.high_z, right_post.high_z);
        HeightEstimate overlap = makeBoundsHeightEstimate(overlap_low_z, overlap_high_z,
                                                          points, source + "_overlap", true);
        if (overlap.valid) {
            return overlap;
        }

        const double avg_low_z = 0.5 * (left_post.low_z + right_post.low_z);
        const double avg_high_z = 0.5 * (left_post.high_z + right_post.high_z);
        return makeBoundsHeightEstimate(avg_low_z, avg_high_z, points, source, true);
    }

    HeightEstimate estimatePostBandCenterZ(const pcl::PointCloud<pcl::PointXYZ>& roi,
                                           const GatePostCandidate& post,
                                           const std::string& source) const {
        const double min_x = post.local_center.x() - 0.5 * post.local_forward_width -
                             auto_height_post_forward_margin_;
        const double max_x = post.local_center.x() + 0.5 * post.local_forward_width +
                             auto_height_post_forward_margin_;
        const double min_y = post.local_center.y() - 0.5 * post.local_lateral_width -
                             auto_height_post_lateral_margin_;
        const double max_y = post.local_center.y() + 0.5 * post.local_lateral_width +
                             auto_height_post_lateral_margin_;

        std::vector<double> z_values;
        z_values.reserve(roi.points.size());
        for (const auto& pt : roi.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            if (!usableForAutoHeight(pt.z)) continue;
            const Eigen::Vector2d local = worldToLocalXY(Eigen::Vector3d(pt.x, pt.y, pt.z));
            if (local.x() < min_x || local.x() > max_x) continue;
            if (local.y() < min_y || local.y() > max_y) continue;
            z_values.push_back(pt.z);
        }

        return makeHeightEstimate(z_values, source, true);
    }

    HeightEstimate estimatePostBandCenterZ(const pcl::PointCloud<pcl::PointXYZ>& roi,
                                           const GatePostCandidate& left_post,
                                           const GatePostCandidate& right_post,
                                           const std::string& source) const {
        const double left_min_x = left_post.local_center.x() - 0.5 * left_post.local_forward_width -
                                  auto_height_post_forward_margin_;
        const double left_max_x = left_post.local_center.x() + 0.5 * left_post.local_forward_width +
                                  auto_height_post_forward_margin_;
        const double left_min_y = left_post.local_center.y() - 0.5 * left_post.local_lateral_width -
                                  auto_height_post_lateral_margin_;
        const double left_max_y = left_post.local_center.y() + 0.5 * left_post.local_lateral_width +
                                  auto_height_post_lateral_margin_;
        const double right_min_x = right_post.local_center.x() - 0.5 * right_post.local_forward_width -
                                   auto_height_post_forward_margin_;
        const double right_max_x = right_post.local_center.x() + 0.5 * right_post.local_forward_width +
                                   auto_height_post_forward_margin_;
        const double right_min_y = right_post.local_center.y() - 0.5 * right_post.local_lateral_width -
                                   auto_height_post_lateral_margin_;
        const double right_max_y = right_post.local_center.y() + 0.5 * right_post.local_lateral_width +
                                   auto_height_post_lateral_margin_;

        std::vector<double> z_values;
        z_values.reserve(roi.points.size());
        for (const auto& pt : roi.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            if (!usableForAutoHeight(pt.z)) continue;
            const Eigen::Vector2d local = worldToLocalXY(Eigen::Vector3d(pt.x, pt.y, pt.z));
            const bool in_left = local.x() >= left_min_x && local.x() <= left_max_x &&
                                 local.y() >= left_min_y && local.y() <= left_max_y;
            const bool in_right = local.x() >= right_min_x && local.x() <= right_max_x &&
                                  local.y() >= right_min_y && local.y() <= right_max_y;
            if (in_left || in_right) {
                z_values.push_back(pt.z);
            }
        }

        return makeHeightEstimate(z_values, source, true);
    }

    HeightEstimate estimateGateBandCenterZ(const pcl::PointCloud<pcl::PointXYZ>& roi,
                                           const GatePostCandidate& left_post,
                                           const GatePostCandidate& right_post,
                                           const std::string& source) const {
        const double min_x = std::min(left_post.local_center.x(), right_post.local_center.x()) -
                             auto_height_forward_tolerance_;
        const double max_x = std::max(left_post.local_center.x(), right_post.local_center.x()) +
                             auto_height_forward_tolerance_;
        const double min_y = std::min(left_post.local_center.y(), right_post.local_center.y()) -
                             auto_height_lateral_margin_;
        const double max_y = std::max(left_post.local_center.y(), right_post.local_center.y()) +
                             auto_height_lateral_margin_;

        std::vector<double> z_values;
        z_values.reserve(roi.points.size());
        for (const auto& pt : roi.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            if (!usableForAutoHeight(pt.z)) continue;
            const Eigen::Vector2d local = worldToLocalXY(Eigen::Vector3d(pt.x, pt.y, pt.z));
            if (local.x() < min_x || local.x() > max_x) continue;
            if (local.y() < min_y || local.y() > max_y) continue;
            z_values.push_back(pt.z);
        }

        return makeHeightEstimate(z_values, source, true);
    }

    HeightEstimate detectionCenterZ(const GatePostCandidate& left_post,
                                    const GatePostCandidate& right_post,
                                    const pcl::PointCloud<pcl::PointXYZ>& roi) const {
        if (height_mode_ == "known_center") {
            HeightEstimate estimate;
            estimate.valid = true;
            estimate.center_z = gate_center_z_;
            estimate.low_z = gate_center_z_;
            estimate.high_z = gate_center_z_;
            estimate.source = "known_center";
            return estimate;
        }
        if (height_mode_ == "fixed") {
            HeightEstimate estimate;
            estimate.valid = true;
            estimate.center_z = flight_z_;
            estimate.low_z = flight_z_;
            estimate.high_z = flight_z_;
            estimate.source = "fixed";
            return estimate;
        }
        if (!use_detected_center_z_) {
            HeightEstimate estimate;
            estimate.valid = true;
            estimate.center_z = gate_center_z_;
            estimate.low_z = gate_center_z_;
            estimate.high_z = gate_center_z_;
            estimate.source = "configured_center";
            return estimate;
        }
        if (height_mode_ == "auto") {
            HeightEstimate post_band = estimatePostBandCenterZ(roi, left_post, right_post,
                                                               "auto_post_bands");
            if (post_band.valid) return post_band;

            HeightEstimate gate_band = estimateGateBandCenterZ(roi, left_post, right_post,
                                                               "auto_gate_band");
            if (gate_band.valid) return gate_band;

            HeightEstimate candidate = candidateHeightEstimate(left_post, right_post,
                                                               "auto_candidate_posts");
            if (candidate.valid) return candidate;
        }
        HeightEstimate candidate = candidateHeightEstimate(left_post, right_post, "candidate_posts");
        if (candidate.valid) return candidate;
        return configuredCenterEstimate("configured_center_fallback");
    }

    HeightEstimate detectionCenterZ(const GatePostCandidate& visible_post,
                                    const pcl::PointCloud<pcl::PointXYZ>& roi) const {
        if (height_mode_ == "known_center") {
            HeightEstimate estimate;
            estimate.valid = true;
            estimate.center_z = gate_center_z_;
            estimate.low_z = gate_center_z_;
            estimate.high_z = gate_center_z_;
            estimate.source = "known_center";
            return estimate;
        }
        if (height_mode_ == "fixed") {
            HeightEstimate estimate;
            estimate.valid = true;
            estimate.center_z = flight_z_;
            estimate.low_z = flight_z_;
            estimate.high_z = flight_z_;
            estimate.source = "fixed";
            return estimate;
        }
        if (!use_detected_center_z_) {
            HeightEstimate estimate;
            estimate.valid = true;
            estimate.center_z = gate_center_z_;
            estimate.low_z = gate_center_z_;
            estimate.high_z = gate_center_z_;
            estimate.source = "configured_center";
            return estimate;
        }

        if (height_mode_ == "auto") {
            HeightEstimate post_band = estimatePostBandCenterZ(roi, visible_post,
                                                               "auto_visible_post");
            if (post_band.valid) return post_band;

            GatePostCandidate synthetic_missing = visible_post;
            const double mid_y = gateCenterY();
            synthetic_missing.local_center.y() = visible_post.local_center.y() >= mid_y
                                                     ? visible_post.local_center.y() - gate_width_expected_
                                                     : visible_post.local_center.y() + gate_width_expected_;
            HeightEstimate gate_band = estimateGateBandCenterZ(roi, visible_post, synthetic_missing,
                                                               "auto_partial_gate_band");
            if (gate_band.valid) return gate_band;

            HeightEstimate candidate = candidateHeightEstimate(visible_post,
                                                               "auto_candidate_visible_post");
            if (candidate.valid) return candidate;
        }
        HeightEstimate candidate = candidateHeightEstimate(visible_post, "candidate_visible_post");
        if (candidate.valid) return candidate;
        return configuredCenterEstimate("configured_center_fallback");
    }

    double goalZ(double center_z) const {
        if (height_mode_ == "known_center") return crossing_z_;
        if (height_mode_ == "fixed") return flight_z_;
        return (goal_z_follows_center_ ? center_z : flight_z_) + goal_z_bias_;
    }

    double gateForwardMin() const {
        return use_local_rule_roi_ ? gate_search_x_min_ : roi_forward_min_;
    }

    double gateForwardMax() const {
        return use_local_rule_roi_ ? gate_search_x_max_ : roi_forward_max_;
    }

    bool buildCandidate(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                        const pcl::PointIndices& indices,
                        GatePostCandidate& out) const {
        if (indices.indices.empty()) return false;

        Eigen::Vector4f min_pt;
        Eigen::Vector4f max_pt;
        pcl::getMinMax3D(cloud, indices.indices, min_pt, max_pt);

        double min_forward = std::numeric_limits<double>::infinity();
        double max_forward = -std::numeric_limits<double>::infinity();
        double min_lateral = std::numeric_limits<double>::infinity();
        double max_lateral = -std::numeric_limits<double>::infinity();
        for (int idx : indices.indices) {
            const auto& pt = cloud.points[idx];
            const Eigen::Vector2d local = worldToLocalXY(Eigen::Vector3d(pt.x, pt.y, pt.z));
            min_forward = std::min(min_forward, local.x());
            max_forward = std::max(max_forward, local.x());
            min_lateral = std::min(min_lateral, local.y());
            max_lateral = std::max(max_lateral, local.y());
        }
        if (!std::isfinite(min_forward) || !std::isfinite(max_forward) ||
            !std::isfinite(min_lateral) || !std::isfinite(max_lateral)) {
            return false;
        }

        out.min_pt = Eigen::Vector3d(min_pt.x(), min_pt.y(), min_pt.z());
        out.max_pt = Eigen::Vector3d(max_pt.x(), max_pt.y(), max_pt.z());
        out.height = out.max_pt.z() - out.min_pt.z();
        out.low_z = out.min_pt.z();
        out.high_z = out.max_pt.z();
        out.z_span = out.height;
        out.points = static_cast<int>(indices.indices.size());
        out.vertical_bins = countVerticalBins(cloud, indices, out.min_pt.z(), out.max_pt.z());
        out.local_forward_width = max_forward - min_forward;
        out.local_lateral_width = max_lateral - min_lateral;
        out.local_center = Eigen::Vector2d(0.5 * (min_forward + max_forward),
                                           0.5 * (min_lateral + max_lateral));
        out.center = localToWorld(out.local_center.x(), out.local_center.y(),
                                  0.5 * (out.min_pt.z() + out.max_pt.z()));
        return true;
    }

    int countVerticalBins(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                          const pcl::PointIndices& indices,
                          double min_z,
                          double max_z) const {
        if (indices.indices.empty() || max_z <= min_z) return 0;
        const int bins = std::max(1, static_cast<int>(std::ceil((max_z - min_z) / vertical_bin_size_)));
        std::vector<bool> occupied(bins, false);
        for (int idx : indices.indices) {
            const double z = cloud.points[idx].z;
            int bin = static_cast<int>((z - min_z) / vertical_bin_size_);
            bin = std::max(0, std::min(bin, bins - 1));
            occupied[bin] = true;
        }
        return static_cast<int>(std::count(occupied.begin(), occupied.end(), true));
    }

    void classifyCandidate(GatePostCandidate& c) const {
        c.accepted = false;
        c.strong = false;
        c.weak = false;
        c.reason = "unknown";
        c.confidence = 0.0;

        const double width_min = std::min(c.local_forward_width, c.local_lateral_width);
        const double width_max = std::max(c.local_forward_width, c.local_lateral_width);

        if (c.height < gate_post_weak_min_height_) {
            c.reason = "height_too_low";
            return;
        }
        if (c.vertical_bins < gate_post_weak_min_vertical_bins_) {
            c.reason = "vertical_bins_too_few";
            return;
        }
        if (width_max > gate_post_weak_max_width_) {
            c.reason = "post_too_wide";
            return;
        }
        if (width_max < gate_post_min_width_) {
            c.reason = "post_too_thin";
            return;
        }

        c.accepted = true;
        const bool strong_height = c.height >= gate_post_min_height_ &&
                                   c.vertical_bins >= gate_post_min_vertical_bins_;
        const bool strong_width = width_max <= gate_post_max_width_;
        if (strong_height && strong_width) {
            c.strong = true;
            c.reason = "accepted_strong_post";
            c.confidence = std::min(1.0, c.height) +
                           0.04 * static_cast<double>(c.vertical_bins) +
                           0.0008 * static_cast<double>(c.points) -
                           0.4 * std::max(0.0, width_max - 0.18) -
                           0.1 * width_min;
        } else {
            c.weak = true;
            c.reason = strong_height ? "accepted_weak_wide_post" : "accepted_weak_low_post";
            c.confidence = 0.35 * std::min(1.0, c.height) +
                           0.02 * static_cast<double>(c.vertical_bins) +
                           0.0005 * static_cast<double>(c.points) -
                           0.25 * std::max(0.0, width_max - gate_post_max_width_) -
                           0.1 * width_min;
        }
    }

    bool splitWideFrameCluster(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                               const pcl::PointIndices& indices,
                               GatePostCandidate& out_left,
                               GatePostCandidate& out_right) const {
        if (indices.indices.empty()) return false;

        GatePostCandidate whole;
        if (!buildCandidate(cloud, indices, whole)) return false;

        const double lateral_width = whole.local_lateral_width;
        const double forward_width = whole.local_forward_width;
        if (lateral_width < gate_frame_cluster_min_width_ ||
            lateral_width > gate_frame_cluster_max_width_) {
            return false;
        }
        if (forward_width > pair_forward_tolerance_ + 0.25) {
            return false;
        }
        if (whole.height < gate_post_min_height_) {
            return false;
        }

        double min_y = std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        for (int idx : indices.indices) {
            const auto& pt = cloud.points[idx];
            const Eigen::Vector2d local = worldToLocalXY(Eigen::Vector3d(pt.x, pt.y, pt.z));
            min_y = std::min(min_y, local.y());
            max_y = std::max(max_y, local.y());
        }

        pcl::PointIndices left_indices;
        pcl::PointIndices right_indices;
        const double band = std::max(gate_split_side_band_, gate_post_min_width_);
        for (int idx : indices.indices) {
            const auto& pt = cloud.points[idx];
            const Eigen::Vector2d local = worldToLocalXY(Eigen::Vector3d(pt.x, pt.y, pt.z));
            if (max_y - local.y() <= band) {
                left_indices.indices.push_back(idx);
            }
            if (local.y() - min_y <= band) {
                right_indices.indices.push_back(idx);
            }
        }

        if (!buildCandidate(cloud, left_indices, out_left) ||
            !buildCandidate(cloud, right_indices, out_right)) {
            return false;
        }

        classifyCandidate(out_left);
        classifyCandidate(out_right);
        if (!out_left.accepted || !out_right.accepted) return false;
        if (out_left.height < gate_post_min_height_ ||
            out_right.height < gate_post_min_height_ ||
            out_left.vertical_bins < gate_post_min_vertical_bins_ ||
            out_right.vertical_bins < gate_post_min_vertical_bins_) {
            return false;
        }

        out_left.strong = true;
        out_left.weak = false;
        out_left.reason = "accepted_split_left_post";
        out_right.strong = true;
        out_right.weak = false;
        out_right.reason = "accepted_split_right_post";
        out_left.confidence += 0.15;
        out_right.confidence += 0.15;
        return true;
    }

    bool buildFrameClusterCandidate(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                    const pcl::PointIndices& indices,
                                    FrameClusterCandidate& out) const {
        if (indices.indices.empty()) return false;

        GatePostCandidate whole;
        if (!buildCandidate(cloud, indices, whole)) return false;

        if (whole.points < frame_cluster_min_points_) return false;
        if (whole.local_lateral_width < gate_frame_cluster_min_width_ ||
            whole.local_lateral_width > gate_frame_cluster_max_width_) {
            return false;
        }
        if (whole.local_forward_width < frame_cluster_forward_width_min_ ||
            whole.local_forward_width > frame_cluster_forward_width_max_) {
            return false;
        }
        if (whole.height < frame_cluster_min_height_ ||
            whole.height > frame_cluster_max_height_) {
            return false;
        }
        if (whole.vertical_bins < gate_post_min_vertical_bins_) return false;
        if (!passesGateCenterConstraint(whole.local_center.y(), gate_center_y_tolerance_)) {
            return false;
        }

        GatePostCandidate left;
        GatePostCandidate right;
        if (!splitWideFrameCluster(cloud, indices, left, right)) return false;

        const double lateral_gap = std::abs(left.local_center.y() - right.local_center.y());
        if (lateral_gap < gate_width_min_ || lateral_gap > gate_width_max_) return false;
        const double forward_delta = std::abs(left.local_center.x() - right.local_center.x());
        if (forward_delta > pair_forward_tolerance_) return false;
        const double pair_center_y = 0.5 * (left.local_center.y() + right.local_center.y());
        if (!passesGateCenterConstraint(pair_center_y, gate_center_y_tolerance_)) return false;
        if (!pairHeightCompatible(left, right)) return false;

        const double width_penalty = std::abs(lateral_gap - gate_width_expected_);
        const double center_penalty = std::abs(pair_center_y - gateCenterY());
        const double thickness_penalty = std::max(0.0, whole.local_forward_width - 0.08);
        const double height_bonus = std::min(1.5, whole.height);
        const double point_bonus = 0.0005 * static_cast<double>(whole.points);

        left.strong = true;
        left.weak = false;
        left.reason = "accepted_frame_cluster_left";
        left.confidence += 0.35;
        right.strong = true;
        right.weak = false;
        right.reason = "accepted_frame_cluster_right";
        right.confidence += 0.35;
        whole.accepted = true;
        whole.strong = true;
        whole.weak = false;
        whole.reason = "accepted_frame_cluster";

        out.whole = whole;
        out.left = left;
        out.right = right;
        out.score = left.confidence + right.confidence + height_bonus + point_bonus -
                    0.35 * width_penalty -
                    0.25 * center_penalty -
                    0.20 * forward_delta -
                    1.00 * thickness_penalty;
        return true;
    }

    bool selectBestFrameCluster(const std::vector<FrameClusterCandidate>& frame_clusters,
                                GatePostCandidate& out_left,
                                GatePostCandidate& out_right,
                                double& out_score) const {
        double best_score = -std::numeric_limits<double>::infinity();
        bool found = false;
        for (const auto& c : frame_clusters) {
            if (c.score > best_score) {
                best_score = c.score;
                out_left = c.left;
                out_right = c.right;
                found = true;
            }
        }
        out_score = best_score;
        return found;
    }

    bool selectBestPair(const std::vector<GatePostCandidate>& candidates,
                        GatePostCandidate& out_left,
                        GatePostCandidate& out_right,
                        double& out_score,
                        bool allow_weak) const {
        double best_score = -std::numeric_limits<double>::infinity();
        bool found = false;

        for (size_t i = 0; i < candidates.size(); ++i) {
            for (size_t j = i + 1; j < candidates.size(); ++j) {
                const auto& a = candidates[i];
                const auto& b = candidates[j];
                if (allow_weak) {
                    if (!((a.strong && b.weak) || (a.weak && b.strong))) continue;
                } else if (!a.strong || !b.strong) {
                    continue;
                }

                const double lateral_gap = std::abs(a.local_center.y() - b.local_center.y());
                if (lateral_gap < gate_width_min_ || lateral_gap > gate_width_max_) continue;

                const double forward_delta = std::abs(a.local_center.x() - b.local_center.x());
                if (forward_delta > pair_forward_tolerance_) continue;

                const double pair_center_y = 0.5 * (a.local_center.y() + b.local_center.y());
                if (!passesGateCenterConstraint(pair_center_y, gate_center_y_tolerance_)) continue;
                if (!pairHeightCompatible(a, b)) continue;

                const double avg_forward = 0.5 * (a.local_center.x() + b.local_center.x());
                if (avg_forward < gateForwardMin() || avg_forward > gateForwardMax()) continue;

                const double gap_penalty = std::abs(lateral_gap - gate_width_expected_);
                const double center_penalty = std::abs(pair_center_y - gateCenterY());
                const double height_penalty = pairHeightDelta(a, b);
                const double score = a.confidence + b.confidence -
                                     0.65 * forward_delta -
                                     0.35 * gap_penalty -
                                     0.25 * center_penalty -
                                     0.20 * height_penalty -
                                     0.08 * std::max(0.0, avg_forward) -
                                     (allow_weak ? 0.15 : 0.0);
                if (score > best_score) {
                    best_score = score;
                    if (a.local_center.y() >= b.local_center.y()) {
                        out_left = a;
                        out_right = b;
                    } else {
                        out_left = b;
                        out_right = a;
                    }
                    found = true;
                }
            }
        }

        out_score = best_score;
        return found;
    }

    bool selectBestSingle(const std::vector<GatePostCandidate>& candidates,
                          GatePostCandidate& out_post) const {
        if (candidates.empty()) return false;

        const double mid_y = gateCenterY();
        double best_score = -std::numeric_limits<double>::infinity();
        bool found = false;
        for (const auto& c : candidates) {
            if (!c.strong && !(allow_weak_single_partial_ && c.weak)) continue;
            if (use_local_rule_roi_) {
                if (c.local_center.x() < gate_search_x_min_ || c.local_center.x() > gate_search_x_max_) continue;
                if (c.local_center.y() < gate_search_y_min_ || c.local_center.y() > gate_search_y_max_) continue;
            }

            const double completed_other_y = c.local_center.y() >= mid_y
                                                 ? c.local_center.y() - gate_width_expected_
                                                 : c.local_center.y() + gate_width_expected_;
            const double completed_center_y = 0.5 * (c.local_center.y() + completed_other_y);
            if (!passesGateCenterConstraint(completed_center_y, partial_center_y_tolerance_)) {
                continue;
            }
            if (use_local_rule_roi_ &&
                (completed_center_y < gate_search_y_min_ || completed_center_y > gate_search_y_max_)) {
                continue;
            }

            const double side_bonus = 0.15 * std::abs(c.local_center.y() - mid_y);
            const double forward_penalty = 0.04 * std::abs(c.local_center.x() - gateForwardMin());
            const double score = c.confidence + side_bonus - forward_penalty - (c.weak ? 0.20 : 0.0);
            if (score > best_score) {
                best_score = score;
                out_post = c;
                found = true;
            }
        }
        return found;
    }

    GateDetection makeFullDetection(const GatePostCandidate& left_post,
                                    const GatePostCandidate& right_post,
                                    const pcl::PointCloud<pcl::PointXYZ>& roi) const {
        GateDetection d;
        d.left = left_post.center;
        d.right = right_post.center;
        d.center = 0.5 * (left_post.center + right_post.center);
        const HeightEstimate height = detectionCenterZ(left_post, right_post, roi);
        const double display_z = goalZ(height.center_z);
        d.center.z() = display_z;
        d.left.z() = display_z;
        d.right.z() = display_z;
        d.height_source = height.source;
        d.height_low_z = height.low_z;
        d.height_high_z = height.high_z;
        d.height_span = height.span;
        d.height_points = height.points;
        d.gap = std::abs(left_post.local_center.y() - right_post.local_center.y());

        const Eigen::Vector2d span(d.right.x() - d.left.x(), d.right.y() - d.left.y());
        Eigen::Vector2d forward(span.y(), -span.x());
        if (forward.norm() < 1e-3) {
            d.valid = false;
            return d;
        }
        forward.normalize();

        const Eigen::Vector2d center_xy(d.center.x(), d.center.y());
        const Eigen::Vector2d odom_xy(odom_pos_.x(), odom_pos_.y());
        const Eigen::Vector2d pre_xy = center_xy - pre_offset_ * forward;
        const Eigen::Vector2d post_xy = center_xy + post_offset_ * forward;
        if ((pre_xy - odom_xy).squaredNorm() > (post_xy - odom_xy).squaredNorm()) {
            forward = -forward;
        }

        d.yaw = normalizeYaw(std::atan2(forward.y(), forward.x()));
        d.pre_goal = d.center - Eigen::Vector3d(pre_offset_ * forward.x(), pre_offset_ * forward.y(), 0.0);
        d.post_goal = d.center + Eigen::Vector3d(post_offset_ * forward.x(), post_offset_ * forward.y(), 0.0);
        d.pre_goal.z() = display_z;
        d.post_goal.z() = display_z;
        d.valid = true;
        return d;
    }

    GateDetection makePartialDetection(const GatePostCandidate& visible_post,
                                       const pcl::PointCloud<pcl::PointXYZ>& roi) const {
        GateDetection d;
        d.partial = true;

        const double mid_y = gateCenterY();
        const bool visible_left = visible_post.local_center.y() >= mid_y;
        const double visible_y = visible_post.local_center.y();
        const double missing_y = visible_left
                                     ? visible_y - gate_width_expected_
                                     : visible_y + gate_width_expected_;
        const double center_x = clampDouble(visible_post.local_center.x(), gateForwardMin(), gateForwardMax());
        const double center_y = 0.5 * (visible_y + missing_y);

        if (use_local_rule_roi_ &&
            (center_y < gate_search_y_min_ || center_y > gate_search_y_max_)) {
            return d;
        }
        if (!passesGateCenterConstraint(center_y, partial_center_y_tolerance_)) {
            return d;
        }

        const Eigen::Vector3d missing_post = localToWorld(center_x, missing_y, visible_post.center.z());
        if (visible_left) {
            d.left = visible_post.center;
            d.right = missing_post;
        } else {
            d.left = missing_post;
            d.right = visible_post.center;
        }
        const HeightEstimate height = detectionCenterZ(visible_post, roi);
        const double display_z = goalZ(height.center_z);
        d.center = localToWorld(center_x, center_y, display_z);
        d.left.z() = display_z;
        d.right.z() = display_z;
        d.height_source = height.source;
        d.height_low_z = height.low_z;
        d.height_high_z = height.high_z;
        d.height_span = height.span;
        d.height_points = height.points;
        d.gap = gate_width_expected_;

        Eigen::Vector2d forward(std::cos(use_local_rule_roi_ ? home_yaw_ : odom_yaw_),
                                std::sin(use_local_rule_roi_ ? home_yaw_ : odom_yaw_));
        const Eigen::Vector2d center_xy(d.center.x(), d.center.y());
        const Eigen::Vector2d odom_xy(odom_pos_.x(), odom_pos_.y());
        const Eigen::Vector2d pre_xy = center_xy - pre_offset_ * forward;
        const Eigen::Vector2d post_xy = center_xy + post_offset_ * forward;
        if ((pre_xy - odom_xy).squaredNorm() > (post_xy - odom_xy).squaredNorm()) {
            forward = -forward;
        }

        d.yaw = normalizeYaw(std::atan2(forward.y(), forward.x()));
        d.pre_goal = d.center - Eigen::Vector3d(pre_offset_ * forward.x(), pre_offset_ * forward.y(), 0.0);
        d.post_goal = d.center + Eigen::Vector3d(post_offset_ * forward.x(), post_offset_ * forward.y(), 0.0);
        d.pre_goal.z() = display_z;
        d.post_goal.z() = display_z;
        d.valid = true;
        return d;
    }

    bool isStable() const {
        if (static_cast<int>(history_.size()) < stable_frames_) return false;
        const GateDetection& ref = history_.back();
        int stable = 0;
        for (const auto& d : history_) {
            if ((d.center - ref.center).norm() < stability_threshold_) {
                ++stable;
            }
        }
        return stable >= stable_frames_;
    }

    GateDetection averageHistory() const {
        GateDetection avg;
        if (history_.empty()) return avg;

        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (const auto& d : history_) {
            avg.left += d.left;
            avg.right += d.right;
            avg.center += d.center;
            avg.pre_goal += d.pre_goal;
            avg.post_goal += d.post_goal;
            avg.gap += d.gap;
            avg.height_low_z += d.height_low_z;
            avg.height_high_z += d.height_high_z;
            avg.height_span += d.height_span;
            avg.height_points += d.height_points;
            sin_sum += std::sin(d.yaw);
            cos_sum += std::cos(d.yaw);
        }
        const double n = static_cast<double>(history_.size());
        avg.left /= n;
        avg.right /= n;
        avg.center /= n;
        avg.pre_goal /= n;
        avg.post_goal /= n;
        avg.gap /= n;
        avg.height_low_z /= n;
        avg.height_high_z /= n;
        avg.height_span /= n;
        avg.height_points = static_cast<int>(std::round(static_cast<double>(avg.height_points) / n));
        avg.height_source = history_.back().height_source + "_avg";
        avg.yaw = std::atan2(sin_sum, cos_sum);
        avg.valid = true;
        return avg;
    }

    int stableHistoryCount() const {
        if (history_.empty()) return 0;
        const GateDetection& ref = history_.back();
        int stable = 0;
        for (const auto& d : history_) {
            if ((d.center - ref.center).norm() < stability_threshold_) {
                ++stable;
            }
        }
        return stable;
    }

    ClusterDebugInfo makeDebugInfo(const GatePostCandidate& c) const {
        ClusterDebugInfo info;
        info.points = c.points;
        info.local_x = c.local_center.x();
        info.local_y = c.local_center.y();
        info.width_forward = c.local_forward_width;
        info.width_lateral = c.local_lateral_width;
        info.height = c.height;
        info.vertical_bins = c.vertical_bins;
        info.accepted = c.accepted;
        info.strong = c.strong;
        info.weak = c.weak;
        info.reason = c.reason;
        return info;
    }

    void sortDebug(std::vector<ClusterDebugInfo>& infos) const {
        std::sort(infos.begin(), infos.end(),
                  [](const ClusterDebugInfo& a, const ClusterDebugInfo& b) {
                      if (a.accepted != b.accepted) return a.accepted > b.accepted;
                      return a.points > b.points;
                  });
    }

    void cacheCandidateCenters(const std::vector<GatePostCandidate>& candidates) {
        debug_candidates_.clear();
        for (const auto& c : candidates) {
            debug_candidates_.push_back(c.center);
        }
        if (debug_candidates_.size() > 40) {
            debug_candidates_.resize(40);
        }
    }

    void logFrame(const std::string& status,
                  const sensor_msgs::PointCloud2& msg,
                  size_t input_size,
                  size_t down_size,
                  size_t roi_size,
                  size_t post_roi_size,
                  size_t cluster_count,
                  size_t accepted_count,
                  const std::vector<ClusterDebugInfo>& cluster_debug,
                  const GateDetection* detection,
                  double pair_score = 0.0) const {
        if (readable_logs_ && (!detection || !detection->valid)) return;
        if (!shouldLog()) return;

        if (readable_logs_) {
            int stable_count = stableHistoryCount();
            if (detection && detection->valid && status.find("valid_") == 0 && stable_count == 0) {
                stable_count = stable_frames_;
            }

            if (detection && detection->valid) {
                ROS_INFO("[frame_detector] DETECT frame status=%s stable=%d/%d center=(%.2f %.2f %.2f) pre=(%.2f %.2f %.2f) post=(%.2f %.2f %.2f) offset_before=%.2f offset_after=%.2f detected_z=%.2f height_source=%s yaw=%.2f",
                         status.c_str(), stable_count, stable_frames_,
                         detection->center.x(), detection->center.y(), detection->center.z(),
                         detection->pre_goal.x(), detection->pre_goal.y(), detection->pre_goal.z(),
                         detection->post_goal.x(), detection->post_goal.y(), detection->post_goal.z(),
                         pre_offset_, post_offset_,
                         detection->post_goal.z(), detection->height_source.c_str(), detection->yaw);
                return;
            }

            (void)status;
            (void)input_size;
            (void)down_size;
            (void)roi_size;
            (void)post_roi_size;
            (void)cluster_count;
            (void)accepted_count;
            (void)pair_score;
            return;
        }

        std::ostringstream oss;
        oss << "[frame_detector] status=" << status
            << " gate_center_y=" << gateCenterY()
            << " frame=" << msg.header.frame_id
            << " input=" << input_size
            << " down=" << down_size
            << " roi=" << roi_size
            << " post_roi=" << post_roi_size
            << " clusters=" << cluster_count
            << " accepted=" << accepted_count
            << " local_rule=" << use_local_rule_roi_
            << " gate_roi=x[" << gateForwardMin() << "," << gateForwardMax() << "]";

        if (use_local_rule_roi_) {
            oss << " y[" << gate_search_y_min_ << "," << gate_search_y_max_ << "]";
        } else {
            oss << " y[+/-" << roi_lateral_abs_ << "]";
        }

        const int count = std::min<int>(debug_top_clusters_, cluster_debug.size());
        for (int i = 0; i < count; ++i) {
            const auto& c = cluster_debug[i];
            oss << " | [" << i << "] pts=" << c.points
                << " local=(" << c.local_x << "," << c.local_y << ")"
                << " size=(" << c.width_forward << "," << c.width_lateral << "," << c.height << ")"
                << " zbins=" << c.vertical_bins
                << " acc=" << c.accepted
                << " strong=" << c.strong
                << " weak=" << c.weak
                << " reason=" << c.reason;
        }

        if (detection && detection->valid) {
            const Eigen::Vector2d center_body = worldToBodyLocalXY(detection->center);
            oss << " | pair gap=" << detection->gap
                << " score=" << pair_score
                << " height_source=" << detection->height_source
                << " height_pts=" << detection->height_points
                << " height_z=[" << detection->height_low_z << "," << detection->height_high_z << "]"
                << " height_span=" << detection->height_span
                << " bottom_mode=" << auto_height_bottom_mode_
                << " bottom_z=" << autoHeightBottomZ(detection->height_low_z)
                << " goal_z_bias=" << goal_z_bias_
                << " odom=(" << odom_pos_.x() << "," << odom_pos_.y() << "," << odom_pos_.z() << ")"
                << " center_world=(" << detection->center.x() << "," << detection->center.y() << "," << detection->center.z() << ")"
                << " center_body_rel=(" << center_body.x() << "," << center_body.y() << "," << detection->center.z() - odom_pos_.z() << ")"
                << " pre_world=(" << detection->pre_goal.x() << "," << detection->pre_goal.y() << "," << detection->pre_goal.z() << ")"
                << " post_world=(" << detection->post_goal.x() << "," << detection->post_goal.y() << "," << detection->post_goal.z() << ")"
                << " yaw=" << detection->yaw;
        }

        ROS_INFO("%s", oss.str().c_str());
    }

    bool shouldLog() const {
        if (readable_logs_ && readable_log_period_ <= 0.0) return false;
        const ros::Time now = ros::Time::now();
        const double period = readable_logs_ ? readable_log_period_ : debug_log_period_;
        if (!last_log_time_.isValid() ||
            (now - last_log_time_).toSec() >= period) {
            last_log_time_ = now;
            return true;
        }
        return false;
    }

    std::string outputFrame(const sensor_msgs::PointCloud2& msg) const {
        if (!configured_frame_id_.empty()) return configured_frame_id_;
        if (!msg.header.frame_id.empty()) return msg.header.frame_id;
        if (!odom_frame_id_.empty()) return odom_frame_id_;
        return "map";
    }

    geometry_msgs::Point toPoint(const Eigen::Vector3d& p) const {
        geometry_msgs::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        return point;
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

    void publishOutputs(const GateDetection& d, const ros::Time& stamp) {
        geometry_msgs::PointStamped center;
        center.header.stamp = stamp;
        center.header.frame_id = goal_frame_id_;
        center.point.x = d.center.x();
        center.point.y = d.center.y();
        center.point.z = d.center.z();
        center_pub_.publish(center);
        pre_pub_.publish(makePose(d.pre_goal, d.yaw, goal_frame_id_, stamp));
        post_pub_.publish(makePose(d.post_goal, d.yaw, goal_frame_id_, stamp));
    }

    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
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
                                              const ros::Time& stamp,
                                              float r,
                                              float g,
                                              float bl,
                                              const std::string& ns) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.035;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = bl;
        marker.color.a = 0.9f;
        marker.points.push_back(toPoint(a));
        marker.points.push_back(toPoint(b));
        return marker;
    }

    visualization_msgs::Marker makeRoiMarker(const std::string& frame_id,
                                             const ros::Time& stamp) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = "gate_search_roi";
        marker.id = 300;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.025;
        marker.color.r = 0.2f;
        marker.color.g = 0.7f;
        marker.color.b = 1.0f;
        marker.color.a = 0.85f;

        const double x_min = gateForwardMin();
        const double x_max = gateForwardMax();
        const double y_min = use_local_rule_roi_ ? gate_search_y_min_ : -roi_lateral_abs_;
        const double y_max = use_local_rule_roi_ ? gate_search_y_max_ : roi_lateral_abs_;
        const double z = flight_z_;

        const Eigen::Vector3d p00 = localToWorld(x_min, y_min, z);
        const Eigen::Vector3d p01 = localToWorld(x_min, y_max, z);
        const Eigen::Vector3d p11 = localToWorld(x_max, y_max, z);
        const Eigen::Vector3d p10 = localToWorld(x_max, y_min, z);
        marker.points.push_back(toPoint(p00));
        marker.points.push_back(toPoint(p01));
        marker.points.push_back(toPoint(p01));
        marker.points.push_back(toPoint(p11));
        marker.points.push_back(toPoint(p11));
        marker.points.push_back(toPoint(p10));
        marker.points.push_back(toPoint(p10));
        marker.points.push_back(toPoint(p00));
        return marker;
    }

    visualization_msgs::Marker makeFixedCenterRoiMarker(const std::string& frame_id,
                                                        const ros::Time& stamp) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = "fixed_center_roi";
        marker.id = 301;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = fixed_roi_center_x_;
        marker.pose.position.y = fixed_roi_center_y_;
        marker.pose.position.z = fixed_roi_center_z_;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0 * fixed_roi_radius_;
        marker.scale.y = 2.0 * fixed_roi_radius_;
        marker.scale.z = 2.0 * fixed_roi_radius_;
        marker.color.r = 0.1f;
        marker.color.g = 0.8f;
        marker.color.b = 0.9f;
        marker.color.a = 0.12f;
        return marker;
    }

    void addRoiMarker(visualization_msgs::MarkerArray& arr,
                      const std::string& frame_id,
                      const ros::Time& stamp) const {
        if (use_local_rule_roi_ && !have_home_) return;
        if (!use_local_rule_roi_ && !have_odom_) return;
        arr.markers.push_back(makeRoiMarker(frame_id, stamp));
        if (use_fixed_center_roi_) {
            arr.markers.push_back(makeFixedCenterRoiMarker(frame_id, stamp));
        }
    }

    void publishCandidateMarkers(const std::string& frame_id, const ros::Time& stamp) {
        visualization_msgs::MarkerArray arr;
        addDeleteMarker(arr, frame_id, stamp);
        addRoiMarker(arr, frame_id, stamp);
        int id = 100;
        for (const auto& c : debug_candidates_) {
            arr.markers.push_back(makeSphereMarker(id++, c, frame_id, stamp, 1.0f, 0.85f, 0.1f, 0.08, "gate_post_candidates"));
        }
        marker_pub_.publish(arr);
    }

    void publishMarkers(const GateDetection& d,
                        const std::string& frame_id,
                        const ros::Time& stamp,
                        bool stable) {
        visualization_msgs::MarkerArray arr;
        addDeleteMarker(arr, frame_id, stamp);
        addRoiMarker(arr, frame_id, stamp);

        const bool uncertain = d.partial || d.weak_pair || d.split_frame;
        const float status_r = uncertain ? 1.0f : 0.1f;
        const float status_g = uncertain ? 0.85f : 0.9f;
        const float status_b = uncertain ? 0.05f : 0.25f;
        arr.markers.push_back(makeSphereMarker(1, d.left, frame_id, stamp, status_r, status_g, status_b, 0.18, "gate_posts"));
        arr.markers.push_back(makeSphereMarker(2, d.right, frame_id, stamp, status_r, status_g, status_b, 0.18, "gate_posts"));
        arr.markers.push_back(makeSphereMarker(3, d.center, frame_id, stamp, status_r, status_g, status_b, stable ? 0.20 : 0.12, "gate_center"));
        arr.markers.push_back(makeSphereMarker(4, d.pre_goal, frame_id, stamp, 0.2f, 0.6f, 1.0f, stable ? 0.18 : 0.11, "gate_goals"));
        arr.markers.push_back(makeSphereMarker(5, d.post_goal, frame_id, stamp, 0.2f, 1.0f, 0.3f, stable ? 0.18 : 0.11, "gate_goals"));
        arr.markers.push_back(makeLineMarker(20, d.left, d.right, frame_id, stamp, status_r, status_g, status_b, "gate_lines"));
        arr.markers.push_back(makeLineMarker(21, d.pre_goal, d.post_goal, frame_id, stamp, 0.1f, 1.0f, 0.3f, "gate_lines"));

        int id = 100;
        for (const auto& c : debug_candidates_) {
            arr.markers.push_back(makeSphereMarker(id++, c, frame_id, stamp, 1.0f, 0.85f, 0.1f, 0.08, "gate_post_candidates"));
        }
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

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher status_pub_;
    ros::Publisher center_pub_;
    ros::Publisher pre_pub_;
    ros::Publisher post_pub_;
    ros::Publisher marker_pub_;

    std::string cloud_topic_;
    std::string odom_topic_;
    std::string configured_frame_id_;
    std::string goal_frame_id_ = "world";
    std::string odom_frame_id_;
    double flight_z_ = 0.9;
    std::string height_mode_ = "auto";
    double gate_center_z_ = 0.9;
    double crossing_z_ = 0.9;
    bool use_detected_center_z_ = true;
    bool goal_z_follows_center_ = true;
    double detected_center_z_bias_ = 0.0;
    double goal_z_bias_ = 0.0;
    bool clamp_detected_center_z_ = false;
    double center_z_min_ = 0.45;
    double center_z_max_ = 1.20;
    std::string auto_height_bottom_mode_ = "detected";
    double gate_bottom_z_ = 0.15;
    double auto_height_top_z_max_ = 2.20;
    double auto_height_forward_tolerance_ = 0.35;
    double auto_height_lateral_margin_ = 0.18;
    double auto_height_post_forward_margin_ = 0.12;
    double auto_height_post_lateral_margin_ = 0.12;
    double auto_height_low_quantile_ = 0.05;
    double auto_height_high_quantile_ = 0.95;
    double auto_height_min_span_ = 0.45;
    int auto_height_min_points_ = 18;
    bool use_local_rule_roi_ = true;
    double field_length_ = 6.0;
    double field_width_ = 5.0;
    double gate_search_x_min_ = -0.50;
    double gate_search_x_max_ = 4.20;
    double gate_search_y_min_ = -2.50;
    double gate_search_y_max_ = 2.50;
    double roi_forward_min_ = 0.20;
    double roi_forward_max_ = 5.00;
    double roi_lateral_abs_ = 2.50;
    double roi_z_min_ = 0.15;
    double roi_z_max_ = 2.20;
    double post_extract_z_max_ = 2.00;
    bool use_fixed_center_roi_ = true;
    double fixed_roi_center_x_ = 2.70;
    double fixed_roi_center_y_ = -1.35;
    double fixed_roi_center_z_ = 1.30;
    double fixed_roi_radius_ = 2.65;
    bool use_body_exclusion_ = false;
    double body_exclusion_forward_min_ = -0.55;
    double body_exclusion_forward_max_ = 0.45;
    double body_exclusion_lateral_abs_ = 0.45;
    double body_exclusion_z_min_ = -0.45;
    double body_exclusion_z_max_ = 0.35;
    double voxel_leaf_ = 0.05;
    double cluster_tolerance_ = 0.22;
    int min_cluster_size_ = 8;
    int max_cluster_size_ = 5000;
    double gate_post_min_width_ = 0.05;
    double gate_post_max_width_ = 0.42;
    double gate_post_weak_max_width_ = 0.55;
    double gate_frame_cluster_min_width_ = 0.75;
    double gate_frame_cluster_max_width_ = 1.40;
    bool prefer_frame_cluster_ = true;
    double frame_cluster_forward_width_min_ = 0.02;
    double frame_cluster_forward_width_max_ = 0.30;
    double frame_cluster_min_height_ = 0.80;
    double frame_cluster_max_height_ = 2.20;
    int frame_cluster_min_points_ = 80;
    double gate_split_side_band_ = 0.22;
    double gate_post_min_height_ = 0.40;
    double gate_post_weak_min_height_ = 0.18;
    double vertical_bin_size_ = 0.18;
    int gate_post_min_vertical_bins_ = 2;
    int gate_post_weak_min_vertical_bins_ = 2;
    double gate_width_min_ = 0.85;
    double gate_width_max_ = 1.35;
    double gate_width_expected_ = 1.00;
    double pair_forward_tolerance_ = 0.35;
    bool enforce_gate_center_constraint_ = true;
    double gate_center_x_ = std::numeric_limits<double>::quiet_NaN();
    double gate_center_y_ = std::numeric_limits<double>::quiet_NaN();
    std::string gate_center_y_frame_ = "local";
    double gate_center_y_tolerance_ = 0.45;
    double partial_center_y_tolerance_ = 0.65;
    double pair_max_height_delta_ = 0.60;
    double pair_min_height_overlap_ = 0.15;
    bool allow_weak_single_partial_ = true;
    double pre_offset_ = 0.70;
    double post_offset_ = 1.00;
    double stability_threshold_ = 0.18;
    int stable_frames_ = 3;
    int history_size_ = 5;
    double debug_log_period_ = 1.0;
    int debug_top_clusters_ = 6;
    bool readable_logs_ = false;
    double readable_log_period_ = 2.0;

    bool have_odom_ = false;
    bool have_home_ = false;
    Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
    double odom_yaw_ = 0.0;
    Eigen::Vector3d home_pos_ = Eigen::Vector3d::Zero();
    double home_yaw_ = 0.0;
    std::deque<GateDetection> history_;
    GateDetection latest_;
    std::vector<Eigen::Vector3d> debug_candidates_;
    mutable ros::Time last_log_time_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "craic_frame_detector");
    FrameDetectorNode node;
    ros::spin();
    return 0;
}
