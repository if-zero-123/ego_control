#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

namespace {

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

std::string candidateKindName(int kind) {
    switch (kind) {
        case 1: return "full_box";
        case 2: return "face_patch";
        case 3: return "weak_face";
        case 4: return "corner_edge";
        default: return "rejected";
    }
}

struct PillarCandidate {
    Eigen::Vector3d min_pt = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_pt = Eigen::Vector3d::Zero();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double width_x = 0.0;
    double width_y = 0.0;
    double height = 0.0;
    double score = 0.0;
    int points = 0;
    int kind = 0;
    int vertical_bins = 0;
    double confidence = 0.0;
    std::string reject_reason = "unclassified";
    std::vector<Eigen::Vector3d> center_hypotheses;
};

struct ClusterDebugInfo {
    int points = 0;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double min_z = 0.0;
    double max_z = 0.0;
    double width_x = 0.0;
    double width_y = 0.0;
    double height = 0.0;
    int vertical_bins = 0;
    int kind = 0;
    size_t hypotheses = 0;
    bool accepted = false;
    std::string reason;
};

struct Detection {
    bool valid = false;
    Eigen::Vector3d left = Eigen::Vector3d::Zero();
    Eigen::Vector3d right = Eigen::Vector3d::Zero();
    Eigen::Vector3d corridor = Eigen::Vector3d::Zero();
    Eigen::Vector3d pre_goal = Eigen::Vector3d::Zero();
    Eigen::Vector3d post_goal = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double gap = 0.0;
};

class PillarDebugNode {
public:
    PillarDebugNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
        pnh_.param<std::string>("frame_id", configured_frame_id_, "");
        pnh_.param<double>("flight_z", flight_z_, 0.9);

        pnh_.param<bool>("use_local_rule_roi", use_local_rule_roi_, false);
        pnh_.param<double>("field_length", field_length_, 6.0);
        pnh_.param<double>("field_width", field_width_, 5.0);
        pnh_.param<double>("search_forward_min", search_forward_min_, -0.25);
        pnh_.param<double>("search_forward_max", search_forward_max_, 6.5);
        pnh_.param<double>("search_lateral_abs", search_lateral_abs_, 3.0);

        pnh_.param<double>("roi_x_min", roi_x_min_, -0.3);
        pnh_.param<double>("roi_x_max", roi_x_max_, 4.0);
        pnh_.param<double>("roi_y_min", roi_y_min_, -2.5);
        pnh_.param<double>("roi_y_max", roi_y_max_, 2.5);
        pnh_.param<double>("roi_z_min", roi_z_min_, 0.05);
        pnh_.param<double>("roi_z_max", roi_z_max_, 1.9);

        pnh_.param<double>("voxel_leaf", voxel_leaf_, 0.06);
        pnh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.28);
        pnh_.param<int>("min_cluster_size", min_cluster_size_, 10);
        pnh_.param<int>("max_cluster_size", max_cluster_size_, 8000);

        pnh_.param<double>("pillar_width", pillar_width_, 0.50);
        pnh_.param<double>("pillar_min_visible_height", pillar_min_visible_height_, 0.32);
        pnh_.param<double>("pillar_min_strong_height", pillar_min_strong_height_, 0.50);
        pnh_.param<double>("pillar_full_min_width", pillar_full_min_width_, 0.25);
        pnh_.param<double>("pillar_full_max_width", pillar_full_max_width_, 0.75);
        pnh_.param<double>("pillar_face_span_min", pillar_face_span_min_, 0.25);
        pnh_.param<double>("pillar_face_span_max", pillar_face_span_max_, 0.75);
        pnh_.param<double>("pillar_thin_face_max_width", pillar_thin_face_max_width_, 0.22);
        pnh_.param<double>("pillar_corner_max_width", pillar_corner_max_width_, 0.28);
        pnh_.param<double>("pillar_long_reject_width", pillar_long_reject_width_, 0.85);
        pnh_.param<double>("vertical_bin_size", vertical_bin_size_, 0.18);
        pnh_.param<int>("pillar_min_vertical_bins", pillar_min_vertical_bins_, 2);

        pnh_.param<double>("pillar_pair_gap_min", pillar_pair_gap_min_, 0.8);
        pnh_.param<double>("pillar_pair_gap_max", pillar_pair_gap_max_, 2.8);
        pnh_.param<double>("pillar_pair_expected_gap", pillar_pair_expected_gap_, 1.6);
        pnh_.param<double>("pair_forward_tolerance", pair_forward_tolerance_, 1.0);
        pnh_.param<double>("goal_offset", goal_offset_, 0.90);
        pnh_.param<double>("stability_threshold", stability_threshold_, 0.15);
        pnh_.param<int>("stable_frames", stable_frames_, 3);
        pnh_.param<int>("history_size", history_size_, 5);

        pnh_.param<double>("debug_log_period", debug_log_period_, 1.0);
        pnh_.param<int>("debug_top_clusters", debug_top_clusters_, 5);

        cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &PillarDebugNode::cloudCb, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &PillarDebugNode::odomCb, this);

        status_pub_ = nh_.advertise<std_msgs::String>("/craic_debug/pillar_status", 1, true);
        center_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/craic_debug/pillar_center", 1, true);
        pre_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic_debug/pillar_pre_goal", 1, true);
        post_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic_debug/pillar_post_goal", 1, true);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/craic_debug/pillar_markers", 1, true);

        ROS_INFO("[pillar_debug] cloud=%s odom=%s flight_z=%.2f local_roi=%d abs_roi=x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]",
                 cloud_topic_.c_str(), odom_topic_.c_str(), flight_z_, use_local_rule_roi_,
                 roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_);
        ROS_INFO("[pillar_debug] cluster leaf=%.2f tol=%.2f min=%d height_min=%.2f strong=%.2f pair_gap=[%.2f %.2f] forward_tol=%.2f",
                 voxel_leaf_, cluster_tolerance_, min_cluster_size_,
                 pillar_min_visible_height_, pillar_min_strong_height_,
                 pillar_pair_gap_min_, pillar_pair_gap_max_, pair_forward_tolerance_);
    }

private:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        if (have_home_) return;
        home_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
        home_yaw_ = yawFromQuat(msg->pose.pose.orientation);
        have_home_ = true;
        ROS_INFO("[pillar_debug] home locked at (%.2f %.2f %.2f), yaw=%.2f",
                 home_pos_.x(), home_pos_.y(), home_pos_.z(), home_yaw_);
    }

    Eigen::Vector2d worldToLocalXY(const Eigen::Vector3d& world) const {
        const double dx = world.x() - home_pos_.x();
        const double dy = world.y() - home_pos_.y();
        const double c = std::cos(home_yaw_);
        const double s = std::sin(home_yaw_);
        return Eigen::Vector2d(c * dx + s * dy, -s * dx + c * dy);
    }

    bool inLocalRuleRoi(const Eigen::Vector3d& world) const {
        const Eigen::Vector2d local = worldToLocalXY(world);
        return local.x() >= search_forward_min_ &&
               local.x() <= search_forward_max_ &&
               std::abs(local.y()) <= search_lateral_abs_ &&
               world.z() >= roi_z_min_ &&
               world.z() <= roi_z_max_;
    }

    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        const std::string frame_id = markerFrame(*msg);
        pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *input);
        if (input->empty()) {
            publishStatus("no_cloud");
            clearMarkers(frame_id, msg->header.stamp);
            ROS_WARN_THROTTLE(debug_log_period_, "[pillar_debug] no_cloud on %s frame=%s",
                              cloud_topic_.c_str(), msg->header.frame_id.c_str());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(input);
        voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        voxel.filter(*down);

        pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>());
        if (use_local_rule_roi_) {
            if (!have_home_) {
                publishStatus("waiting_odom");
                clearMarkers(frame_id, msg->header.stamp);
                ROS_WARN_THROTTLE(debug_log_period_, "[pillar_debug] waiting_odom for local ROI");
                return;
            }
            roi->header = down->header;
            for (const auto& pt : down->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                if (inLocalRuleRoi(Eigen::Vector3d(pt.x, pt.y, pt.z))) {
                    roi->points.push_back(pt);
                }
            }
            roi->width = static_cast<uint32_t>(roi->points.size());
            roi->height = 1;
            roi->is_dense = false;
        } else {
            pcl::CropBox<pcl::PointXYZ> crop;
            crop.setInputCloud(down);
            crop.setMin(Eigen::Vector4f(roi_x_min_, roi_y_min_, roi_z_min_, 1.0f));
            crop.setMax(Eigen::Vector4f(roi_x_max_, roi_y_max_, roi_z_max_, 1.0f));
            crop.filter(*roi);
        }

        if (roi->empty()) {
            publishStatus("empty_roi");
            clearMarkers(frame_id, msg->header.stamp);
            ROS_WARN_THROTTLE(debug_log_period_,
                              "[pillar_debug] status=empty_roi frame=%s input=%zu down=%zu roi=0 abs_roi=x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]",
                              msg->header.frame_id.c_str(), input->size(), down->size(),
                              roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_);
            return;
        }

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(roi);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(roi);
        ec.extract(cluster_indices);

        std::vector<PillarCandidate> candidates;
        std::vector<ClusterDebugInfo> cluster_debug;
        for (const auto& indices : cluster_indices) {
            PillarCandidate c;
            if (!buildCandidate(*roi, indices, c)) continue;
            classifyCandidate(c);
            ClusterDebugInfo info = makeClusterDebugInfo(c);
            info.accepted = acceptCandidate(c);
            cluster_debug.push_back(info);
            if (info.accepted) {
                candidates.push_back(c);
            }
        }
        sortClusterDebug(cluster_debug);
        cacheDebugHypotheses(candidates);

        if (candidates.size() < 2) {
            publishStatus("not_enough_candidates");
            publishCandidateMarkers(frame_id, msg->header.stamp);
            logFrame("not_enough_candidates", *msg, input->size(), down->size(), roi->size(),
                     cluster_indices.size(), candidates.size(), cluster_debug, nullptr);
            return;
        }

        PillarCandidate left_candidate;
        PillarCandidate right_candidate;
        Eigen::Vector3d left_center;
        Eigen::Vector3d right_center;
        double pair_score = 0.0;
        if (!selectBestPair(candidates, left_candidate, right_candidate,
                            left_center, right_center, pair_score)) {
            publishStatus("invalid_pair");
            publishCandidateMarkers(frame_id, msg->header.stamp);
            logFrame("invalid_pair", *msg, input->size(), down->size(), roi->size(),
                     cluster_indices.size(), candidates.size(), cluster_debug, nullptr);
            return;
        }

        Detection detection = makeDetection(left_center, right_center);
        if (!detection.valid) {
            publishStatus("invalid_geometry");
            publishCandidateMarkers(frame_id, msg->header.stamp);
            logFrame("invalid_geometry", *msg, input->size(), down->size(), roi->size(),
                     cluster_indices.size(), candidates.size(), cluster_debug, nullptr);
            return;
        }

        history_.push_back(detection);
        while (static_cast<int>(history_.size()) > history_size_) {
            history_.pop_front();
        }

        if (!isStable()) {
            publishStatus("stabilizing");
            publishMarkers(detection, frame_id, msg->header.stamp, false);
            logFrame("stabilizing", *msg, input->size(), down->size(), roi->size(),
                     cluster_indices.size(), candidates.size(), cluster_debug, &detection, pair_score);
            return;
        }

        latest_ = averageHistory();
        publishOutputs(latest_, frame_id, msg->header.stamp);
        publishMarkers(latest_, frame_id, msg->header.stamp, true);
        publishStatus("valid");
        logFrame("valid", *msg, input->size(), down->size(), roi->size(),
                 cluster_indices.size(), candidates.size(), cluster_debug, &latest_, pair_score);
    }

    std::string markerFrame(const sensor_msgs::PointCloud2& msg) const {
        if (!configured_frame_id_.empty()) return configured_frame_id_;
        if (!msg.header.frame_id.empty()) return msg.header.frame_id;
        return "map";
    }

    bool buildCandidate(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                        const pcl::PointIndices& indices,
                        PillarCandidate& out) const {
        if (indices.indices.empty()) return false;

        Eigen::Vector4f min_pt;
        Eigen::Vector4f max_pt;
        pcl::getMinMax3D(cloud, indices.indices, min_pt, max_pt);

        out.min_pt = Eigen::Vector3d(min_pt.x(), min_pt.y(), min_pt.z());
        out.max_pt = Eigen::Vector3d(max_pt.x(), max_pt.y(), max_pt.z());
        out.center = 0.5 * (out.min_pt + out.max_pt);
        out.width_x = out.max_pt.x() - out.min_pt.x();
        out.width_y = out.max_pt.y() - out.min_pt.y();
        out.height = out.max_pt.z() - out.min_pt.z();
        out.points = static_cast<int>(indices.indices.size());
        out.vertical_bins = countVerticalBins(cloud, indices, out.min_pt.z(), out.max_pt.z());

        const double width_error = std::abs(out.width_x - pillar_width_) +
                                   std::abs(out.width_y - pillar_width_);
        const double height_score = std::min(out.height, 1.5);
        out.score = height_score - width_error + 0.0005 * out.points;
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

    void classifyCandidate(PillarCandidate& c) const {
        c.kind = 0;
        c.confidence = 0.0;
        c.center_hypotheses.clear();
        c.reject_reason = "unknown";

        const double w_min = std::min(c.width_x, c.width_y);
        const double w_max = std::max(c.width_x, c.width_y);

        if (c.height < pillar_min_visible_height_) {
            c.reject_reason = "height_too_low";
            return;
        }
        if (c.vertical_bins < pillar_min_vertical_bins_) {
            c.reject_reason = "vertical_bins_too_few";
            return;
        }
        if (w_max > pillar_long_reject_width_) {
            c.reject_reason = "horizontal_extent_too_long";
            return;
        }

        const bool full_like =
            c.width_x >= pillar_full_min_width_ && c.width_x <= pillar_full_max_width_ &&
            c.width_y >= pillar_full_min_width_ && c.width_y <= pillar_full_max_width_;
        const bool face_like =
            w_max >= pillar_face_span_min_ && w_max <= pillar_face_span_max_ &&
            w_min <= pillar_thin_face_max_width_;
        const bool corner_like =
            w_max <= pillar_corner_max_width_ && w_max >= 0.06 &&
            c.height >= pillar_min_strong_height_;

        if (full_like) {
            c.kind = 1;
            c.confidence = 1.0 + std::min(0.5, 0.001 * c.points);
            c.center_hypotheses.push_back(c.center);
            c.reject_reason = "accepted_full_box";
        } else if (face_like && c.height >= pillar_min_strong_height_) {
            c.kind = 2;
            c.confidence = 0.78 + std::min(0.4, 0.001 * c.points);
            addFaceCenterHypotheses(c);
            c.reject_reason = "accepted_face_patch";
        } else if (face_like) {
            c.kind = 3;
            c.confidence = 0.58 + std::min(0.3, 0.001 * c.points);
            addFaceCenterHypotheses(c);
            c.reject_reason = "accepted_weak_face";
        } else if (corner_like) {
            c.kind = 4;
            c.confidence = 0.45 + std::min(0.25, 0.001 * c.points);
            addCornerCenterHypotheses(c);
            c.reject_reason = "accepted_corner_edge";
        } else {
            c.reject_reason = "shape_not_pillar";
        }
    }

    bool acceptCandidate(const PillarCandidate& c) const {
        return c.kind != 0 && !c.center_hypotheses.empty();
    }

    void addFaceCenterHypotheses(PillarCandidate& c) const {
        c.center_hypotheses.push_back(c.center);
        if (c.width_x <= c.width_y) {
            const double offset = std::max(0.0, 0.5 * (pillar_width_ - c.width_x));
            Eigen::Vector3d h1 = c.center;
            Eigen::Vector3d h2 = c.center;
            h1.x() -= offset;
            h2.x() += offset;
            c.center_hypotheses.push_back(h1);
            c.center_hypotheses.push_back(h2);
        } else {
            const double offset = std::max(0.0, 0.5 * (pillar_width_ - c.width_y));
            Eigen::Vector3d h1 = c.center;
            Eigen::Vector3d h2 = c.center;
            h1.y() -= offset;
            h2.y() += offset;
            c.center_hypotheses.push_back(h1);
            c.center_hypotheses.push_back(h2);
        }
    }

    void addCornerCenterHypotheses(PillarCandidate& c) const {
        const double ox = std::max(0.0, 0.5 * (pillar_width_ - c.width_x));
        const double oy = std::max(0.0, 0.5 * (pillar_width_ - c.width_y));
        c.center_hypotheses.push_back(c.center);
        for (double sx : {-1.0, 1.0}) {
            for (double sy : {-1.0, 1.0}) {
                Eigen::Vector3d h = c.center;
                h.x() += sx * ox;
                h.y() += sy * oy;
                c.center_hypotheses.push_back(h);
            }
        }
    }

    bool selectBestPair(const std::vector<PillarCandidate>& candidates,
                        PillarCandidate& out_a,
                        PillarCandidate& out_b,
                        Eigen::Vector3d& out_center_a,
                        Eigen::Vector3d& out_center_b,
                        double& out_score) const {
        double best_score = -std::numeric_limits<double>::infinity();
        bool found = false;

        for (size_t i = 0; i < candidates.size(); ++i) {
            for (size_t j = i + 1; j < candidates.size(); ++j) {
                const auto& a = candidates[i];
                const auto& b = candidates[j];
                for (const auto& center_a : a.center_hypotheses) {
                    for (const auto& center_b : b.center_hypotheses) {
                        const Eigen::Vector2d span(center_b.x() - center_a.x(),
                                                   center_b.y() - center_a.y());
                        const double gap = span.norm();
                        if (gap < pillar_pair_gap_min_ || gap > pillar_pair_gap_max_) continue;

                        double forward_delta = std::abs(center_a.x() - center_b.x());
                        double lateral_delta = std::abs(center_a.y() - center_b.y());
                        double forward_position_bonus = 0.0;
                        if (use_local_rule_roi_ && have_home_) {
                            const Eigen::Vector2d la = worldToLocalXY(center_a);
                            const Eigen::Vector2d lb = worldToLocalXY(center_b);
                            forward_delta = std::abs(la.x() - lb.x());
                            lateral_delta = std::abs(la.y() - lb.y());
                            const double avg_forward = 0.5 * (la.x() + lb.x());
                            if (avg_forward > 0.0 && avg_forward < field_length_ + 0.5) {
                                forward_position_bonus = 0.25;
                            }
                        }
                        if (forward_delta > pair_forward_tolerance_) continue;

                        const double gap_penalty = std::abs(gap - pillar_pair_expected_gap_);
                        const double kind_bonus = 0.05 * (a.kind + b.kind);
                        const double score = a.confidence + b.confidence + kind_bonus +
                                             forward_position_bonus -
                                             0.55 * gap_penalty -
                                             0.75 * forward_delta +
                                             0.05 * lateral_delta;
                        if (score > best_score) {
                            best_score = score;
                            out_a = a;
                            out_b = b;
                            out_center_a = center_a;
                            out_center_b = center_b;
                            found = true;
                        }
                    }
                }
            }
        }

        out_score = best_score;
        return found;
    }

    Detection makeDetection(Eigen::Vector3d center_a, Eigen::Vector3d center_b) const {
        Detection d;
        if (have_home_) {
            if (worldToLocalXY(center_a).y() > worldToLocalXY(center_b).y()) {
                std::swap(center_a, center_b);
            }
        } else if (center_a.y() > center_b.y()) {
            std::swap(center_a, center_b);
        }

        d.left = center_a;
        d.right = center_b;
        d.corridor = 0.5 * (center_a + center_b);
        d.corridor.z() = flight_z_;
        d.gap = (center_a - center_b).head<2>().norm();

        const Eigen::Vector2d span(center_b.x() - center_a.x(), center_b.y() - center_a.y());
        Eigen::Vector2d forward(span.y(), -span.x());
        if (forward.norm() < 1e-3) {
            d.valid = false;
            return d;
        }
        forward.normalize();

        Eigen::Vector2d home_xy(0.0, 0.0);
        if (have_home_) {
            home_xy = Eigen::Vector2d(home_pos_.x(), home_pos_.y());
        }
        const Eigen::Vector2d corridor_xy(d.corridor.x(), d.corridor.y());
        const Eigen::Vector2d pre_xy = corridor_xy - goal_offset_ * forward;
        const Eigen::Vector2d post_xy = corridor_xy + goal_offset_ * forward;
        if ((pre_xy - home_xy).squaredNorm() > (post_xy - home_xy).squaredNorm()) {
            forward = -forward;
        }

        d.yaw = normalizeYaw(std::atan2(forward.y(), forward.x()));
        d.pre_goal = d.corridor - Eigen::Vector3d(goal_offset_ * forward.x(), goal_offset_ * forward.y(), 0.0);
        d.post_goal = d.corridor + Eigen::Vector3d(goal_offset_ * forward.x(), goal_offset_ * forward.y(), 0.0);
        d.pre_goal.z() = flight_z_;
        d.post_goal.z() = flight_z_;
        d.valid = true;
        return d;
    }

    bool isStable() const {
        if (static_cast<int>(history_.size()) < stable_frames_) return false;
        const Detection& ref = history_.back();
        int stable = 0;
        for (const auto& d : history_) {
            const double err = (d.corridor - ref.corridor).norm();
            if (err < stability_threshold_) ++stable;
        }
        return stable >= stable_frames_;
    }

    Detection averageHistory() const {
        Detection avg;
        if (history_.empty()) return avg;

        for (const auto& d : history_) {
            avg.left += d.left;
            avg.right += d.right;
            avg.corridor += d.corridor;
            avg.pre_goal += d.pre_goal;
            avg.post_goal += d.post_goal;
            avg.yaw += d.yaw;
            avg.gap += d.gap;
        }
        const double n = static_cast<double>(history_.size());
        avg.left /= n;
        avg.right /= n;
        avg.corridor /= n;
        avg.pre_goal /= n;
        avg.post_goal /= n;
        avg.yaw = normalizeYaw(avg.yaw / n);
        avg.gap /= n;
        avg.valid = true;
        return avg;
    }

    ClusterDebugInfo makeClusterDebugInfo(const PillarCandidate& c) const {
        ClusterDebugInfo info;
        info.points = c.points;
        info.min_x = c.min_pt.x();
        info.max_x = c.max_pt.x();
        info.min_y = c.min_pt.y();
        info.max_y = c.max_pt.y();
        info.min_z = c.min_pt.z();
        info.max_z = c.max_pt.z();
        info.width_x = c.width_x;
        info.width_y = c.width_y;
        info.height = c.height;
        info.vertical_bins = c.vertical_bins;
        info.kind = c.kind;
        info.hypotheses = c.center_hypotheses.size();
        info.reason = c.reject_reason;
        return info;
    }

    void sortClusterDebug(std::vector<ClusterDebugInfo>& infos) const {
        std::sort(infos.begin(), infos.end(),
                  [](const ClusterDebugInfo& a, const ClusterDebugInfo& b) {
                      if (a.accepted != b.accepted) return a.accepted > b.accepted;
                      return a.points > b.points;
                  });
    }

    void cacheDebugHypotheses(const std::vector<PillarCandidate>& candidates) {
        debug_hypotheses_.clear();
        for (const auto& c : candidates) {
            for (const auto& h : c.center_hypotheses) {
                debug_hypotheses_.push_back(h);
            }
        }
        if (debug_hypotheses_.size() > 40) {
            debug_hypotheses_.resize(40);
        }
    }

    void logFrame(const std::string& status,
                  const sensor_msgs::PointCloud2& msg,
                  size_t input_size,
                  size_t down_size,
                  size_t roi_size,
                  size_t cluster_count,
                  size_t accepted_count,
                  const std::vector<ClusterDebugInfo>& cluster_debug,
                  const Detection* detection,
                  double pair_score = 0.0) const {
        if (!shouldLog()) return;

        std::ostringstream oss;
        oss << "[pillar_debug] status=" << status
            << " frame=" << msg.header.frame_id
            << " input=" << input_size
            << " down=" << down_size
            << " roi=" << roi_size
            << " clusters=" << cluster_count
            << " accepted=" << accepted_count;

        const int count = std::min<int>(debug_top_clusters_, cluster_debug.size());
        for (int i = 0; i < count; ++i) {
            const auto& c = cluster_debug[i];
            oss << " | [" << i << "] pts=" << c.points
                << " bbox=x " << c.min_x << ".." << c.max_x
                << " y " << c.min_y << ".." << c.max_y
                << " z " << c.min_z << ".." << c.max_z
                << " size=" << c.width_x << "," << c.width_y << "," << c.height
                << " zbins=" << c.vertical_bins
                << " kind=" << candidateKindName(c.kind)
                << " hyps=" << c.hypotheses
                << " acc=" << c.accepted
                << " reason=" << c.reason;
        }

        if (detection && detection->valid) {
            oss << " | pair gap=" << detection->gap
                << " score=" << pair_score
                << " center=(" << detection->corridor.x() << ","
                << detection->corridor.y() << "," << detection->corridor.z() << ")"
                << " pre=(" << detection->pre_goal.x() << ","
                << detection->pre_goal.y() << "," << detection->pre_goal.z() << ")"
                << " post=(" << detection->post_goal.x() << ","
                << detection->post_goal.y() << "," << detection->post_goal.z() << ")"
                << " yaw=" << detection->yaw;
        }

        ROS_INFO("%s", oss.str().c_str());
    }

    bool shouldLog() const {
        const ros::Time now = ros::Time::now();
        if (!last_log_time_.isValid() ||
            (now - last_log_time_).toSec() >= debug_log_period_) {
            last_log_time_ = now;
            return true;
        }
        return false;
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

    void publishOutputs(const Detection& d, const std::string& frame_id, const ros::Time& stamp) {
        geometry_msgs::PointStamped center;
        center.header.stamp = stamp;
        center.header.frame_id = frame_id;
        center.point.x = d.corridor.x();
        center.point.y = d.corridor.y();
        center.point.z = d.corridor.z();
        center_pub_.publish(center);
        pre_pub_.publish(makePose(d.pre_goal, d.yaw, frame_id, stamp));
        post_pub_.publish(makePose(d.post_goal, d.yaw, frame_id, stamp));
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
                                                const std::string& ns = "pillars") const {
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
                                              float bl) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = "debug_lines";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.035;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = bl;
        marker.color.a = 0.9f;
        geometry_msgs::Point pa;
        geometry_msgs::Point pb;
        pa.x = a.x();
        pa.y = a.y();
        pa.z = a.z();
        pb.x = b.x();
        pb.y = b.y();
        pb.z = b.z();
        marker.points.push_back(pa);
        marker.points.push_back(pb);
        return marker;
    }

    void publishCandidateMarkers(const std::string& frame_id, const ros::Time& stamp) {
        visualization_msgs::MarkerArray arr;
        addDeleteMarker(arr, frame_id, stamp);
        int id = 100;
        for (const auto& h : debug_hypotheses_) {
            arr.markers.push_back(makeSphereMarker(id++, h, frame_id, stamp, 1.0f, 0.85f, 0.1f, 0.08, "hypotheses"));
        }
        marker_pub_.publish(arr);
    }

    void publishMarkers(const Detection& d,
                        const std::string& frame_id,
                        const ros::Time& stamp,
                        bool stable) {
        visualization_msgs::MarkerArray arr;
        addDeleteMarker(arr, frame_id, stamp);
        arr.markers.push_back(makeSphereMarker(1, d.left, frame_id, stamp, 1.0f, 0.2f, 0.2f, 0.22));
        arr.markers.push_back(makeSphereMarker(2, d.right, frame_id, stamp, 1.0f, 0.2f, 0.2f, 0.22));
        arr.markers.push_back(makeSphereMarker(3, d.corridor, frame_id, stamp, 0.1f, 0.9f, 1.0f, stable ? 0.20 : 0.12, "center"));
        arr.markers.push_back(makeSphereMarker(4, d.pre_goal, frame_id, stamp, 0.2f, 0.6f, 1.0f, stable ? 0.18 : 0.11));
        arr.markers.push_back(makeSphereMarker(5, d.post_goal, frame_id, stamp, 0.2f, 1.0f, 0.3f, stable ? 0.18 : 0.11));
        arr.markers.push_back(makeLineMarker(20, d.left, d.right, frame_id, stamp, 1.0f, 0.9f, 0.1f));
        arr.markers.push_back(makeLineMarker(21, d.pre_goal, d.post_goal, frame_id, stamp, 0.1f, 1.0f, 0.3f));

        int id = 100;
        for (const auto& h : debug_hypotheses_) {
            arr.markers.push_back(makeSphereMarker(id++, h, frame_id, stamp, 1.0f, 0.85f, 0.1f, 0.08, "hypotheses"));
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
    double flight_z_ = 0.9;
    bool use_local_rule_roi_ = false;
    bool have_home_ = false;
    Eigen::Vector3d home_pos_ = Eigen::Vector3d::Zero();
    double home_yaw_ = 0.0;
    double field_length_ = 6.0;
    double field_width_ = 5.0;
    double search_forward_min_ = -0.25;
    double search_forward_max_ = 6.5;
    double search_lateral_abs_ = 3.0;
    double roi_x_min_ = -0.3;
    double roi_x_max_ = 4.0;
    double roi_y_min_ = -2.5;
    double roi_y_max_ = 2.5;
    double roi_z_min_ = 0.05;
    double roi_z_max_ = 1.9;
    double voxel_leaf_ = 0.06;
    double cluster_tolerance_ = 0.28;
    int min_cluster_size_ = 10;
    int max_cluster_size_ = 8000;
    double pillar_width_ = 0.50;
    double pillar_min_visible_height_ = 0.32;
    double pillar_min_strong_height_ = 0.50;
    double pillar_full_min_width_ = 0.25;
    double pillar_full_max_width_ = 0.75;
    double pillar_face_span_min_ = 0.25;
    double pillar_face_span_max_ = 0.75;
    double pillar_thin_face_max_width_ = 0.22;
    double pillar_corner_max_width_ = 0.28;
    double pillar_long_reject_width_ = 0.85;
    double vertical_bin_size_ = 0.18;
    int pillar_min_vertical_bins_ = 2;
    double pillar_pair_gap_min_ = 0.8;
    double pillar_pair_gap_max_ = 2.8;
    double pillar_pair_expected_gap_ = 1.6;
    double pair_forward_tolerance_ = 1.0;
    double goal_offset_ = 0.90;
    double stability_threshold_ = 0.15;
    int stable_frames_ = 3;
    int history_size_ = 5;
    double debug_log_period_ = 1.0;
    int debug_top_clusters_ = 5;

    std::deque<Detection> history_;
    Detection latest_;
    std::vector<Eigen::Vector3d> debug_hypotheses_;
    mutable ros::Time last_log_time_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "pillar_debug_node");
    PillarDebugNode node;
    ros::spin();
    return 0;
}
