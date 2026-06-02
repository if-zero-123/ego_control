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
    // 单个聚类的包围盒信息，用尺寸和点数给“像柱子”的程度打分。
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
    // 对只看到一个面/一条边的柱子，不直接用 bbox 中心，而是生成多个可能柱心。
    std::vector<Eigen::Vector3d> center_hypotheses;
};

struct Detection {
    // 一次完整柱子识别结果：左右柱、通道中心、柱前/柱后安全目标点。
    bool valid = false;
    Eigen::Vector3d left = Eigen::Vector3d::Zero();
    Eigen::Vector3d right = Eigen::Vector3d::Zero();
    Eigen::Vector3d corridor = Eigen::Vector3d::Zero();
    Eigen::Vector3d pre_goal = Eigen::Vector3d::Zero();
    Eigen::Vector3d post_goal = Eigen::Vector3d::Zero();
    double yaw = 0.0;
};

class PillarDetectorNode {
public:
    PillarDetectorNode() : nh_(), pnh_("~") {
        // 默认使用已经在调试包验证过的 FAST-LIO MID360 点云。
        pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
        pnh_.param<std::string>("frame_id", frame_id_, "");
        pnh_.param<double>("flight_z", flight_z_, 0.9);
        // 当前实测成功路径使用 /cloud_registered 的绝对 ROI，避免 home_yaw 假设影响识别。
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
        // 规则约束：方柱不低于 1.5m，截面不小于 0.5m；实际点云可能只看到一个面/角点。
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
        pnh_.param<double>("pillar_pair_x_tolerance", pair_forward_tolerance_, pair_forward_tolerance_);
        pnh_.param<double>("safety_inflation", safety_inflation_, 0.40);
        pnh_.param<double>("goal_offset", goal_offset_, 0.90);
        pnh_.param<double>("stability_threshold", stability_threshold_, 0.15);
        pnh_.param<int>("stable_frames", stable_frames_, 3);
        pnh_.param<int>("history_size", history_size_, 5);
        pnh_.param<bool>("readable_logs", readable_logs_, false);
        pnh_.param<double>("readable_log_period", readable_log_period_, 2.0);

        cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &PillarDetectorNode::cloudCb, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &PillarDetectorNode::odomCb, this);
        pre_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic/pillar_pre_goal", 1, true);
        post_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic/pillar_post_goal", 1, true);
        status_pub_ = nh_.advertise<std_msgs::String>("/craic/pillar_status", 1, true);
        center_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/craic/pillar_center", 1, true);
        // Marker 只用于 RViz 调试，不参与飞控闭环。
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/craic/pillar_markers", 1, true);

        if (!readable_logs_) {
            ROS_INFO("[pillar_detector] cloud=%s odom=%s frame=%s flight_z=%.2f local_rule_roi=%d abs_roi=x[%.2f %.2f] y[%.2f %.2f] z[%.2f %.2f]",
                     cloud_topic_.c_str(), odom_topic_.c_str(), frame_id_.c_str(),
                     flight_z_, use_local_rule_roi_, roi_x_min_, roi_x_max_,
                     roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_);
            ROS_INFO("[pillar_detector] cluster leaf=%.2f tol=%.2f min=%d strong_height=%.2f pair_gap=[%.2f %.2f] forward_tol=%.2f stable=%d/%d thr=%.2f",
                     voxel_leaf_, cluster_tolerance_, min_cluster_size_,
                     pillar_min_strong_height_, pillar_pair_gap_min_, pillar_pair_gap_max_,
                     pair_forward_tolerance_, stable_frames_, history_size_, stability_threshold_);
        }
    }

private:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        if (have_home_) return;
        home_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
        home_yaw_ = yawFromQuat(msg->pose.pose.orientation);
        have_home_ = true;
        if (!readable_logs_) {
            ROS_INFO("[pillar_detector] home locked at (%.2f %.2f %.2f), yaw=%.2f",
                     home_pos_.x(), home_pos_.y(), home_pos_.z(), home_yaw_);
        }
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
        const std::string frame_id = outputFrame(*msg);
        pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *input);
        if (input->empty()) {
            publishStatus("no_cloud");
            if (readable_logs_) {
                logReadable("no_cloud", input->size(), 0, 0, 0, 0, nullptr);
            } else {
                ROS_WARN_THROTTLE(1.0, "[pillar_detector] no_cloud on %s", cloud_topic_.c_str());
            }
            return;
        }

        // 先体素降采样，减少欧式聚类的计算量，同时压掉局部噪声。
        pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(input);
        voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        voxel.filter(*down);

        // 用规则场地尺寸做 ROI 约束：默认在 home 局部坐标中裁剪，不绑定 Gazebo 绝对坐标。
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>());
        if (use_local_rule_roi_) {
            if (!have_home_) {
                publishStatus("waiting_odom");
                if (readable_logs_) {
                    logReadable("waiting_odom", input->size(), down->size(), 0, 0, 0, nullptr);
                } else {
                    ROS_WARN_THROTTLE(1.0, "[pillar_detector] waiting_odom before local rule ROI can run");
                }
                return;
            }
            roi->header = down->header;
            for (const auto& pt : down->points) {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                const Eigen::Vector3d world(pt.x, pt.y, pt.z);
                if (inLocalRuleRoi(world)) {
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
            if (readable_logs_) {
                logReadable("empty_roi", input->size(), down->size(), roi->size(), 0, 0, nullptr);
            } else {
                ROS_WARN_THROTTLE(1.0,
                                  "[pillar_detector] empty_roi input=%zu down=%zu frame=%s local_rule=%d roi=[forward %.2f..%.2f lateral +/-%.2f z %.2f..%.2f]",
                                  input->size(), down->size(), msg->header.frame_id.c_str(),
                                  use_local_rule_roi_, search_forward_min_, search_forward_max_,
                                  search_lateral_abs_, roi_z_min_, roi_z_max_);
            }
            return;
        }

        // 对 ROI 点云做欧式聚类，后面再用高度/宽度筛出方形柱。
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
        for (const auto& indices : cluster_indices) {
            PillarCandidate c;
            // 每个聚类转成候选柱，再按规则尺寸和可见形态生成柱心假设。
            if (!buildCandidate(*roi, indices, c)) continue;
            classifyCandidate(c);
            if (!acceptCandidate(c)) continue;
            candidates.push_back(c);
        }
        cacheDebugHypotheses(candidates);

        if (candidates.size() < 2) {
            publishStatus("not_enough_candidates");
            if (readable_logs_) {
                logReadable("not_enough_candidates", input->size(), down->size(), roi->size(),
                            cluster_indices.size(), candidates.size(), nullptr);
            } else {
                ROS_WARN_THROTTLE(1.0,
                                  "[pillar_detector] not_enough_candidates input=%zu down=%zu roi=%zu clusters=%zu accepted=%zu frame=%s",
                                  input->size(), down->size(), roi->size(), cluster_indices.size(), candidates.size(),
                                  msg->header.frame_id.c_str());
                logClusterSummary(*roi, cluster_indices);
            }
            return;
        }

        PillarCandidate left_candidate;
        PillarCandidate right_candidate;
        Eigen::Vector3d left_center;
        Eigen::Vector3d right_center;
        if (!selectBestPair(candidates, left_candidate, right_candidate, left_center, right_center)) {
            publishStatus("invalid_pair");
            if (readable_logs_) {
                logReadable("invalid_pair", input->size(), down->size(), roi->size(),
                            cluster_indices.size(), candidates.size(), nullptr);
            } else {
                ROS_WARN_THROTTLE(1.0,
                                  "[pillar_detector] invalid_pair accepted=%zu gap=[%.2f..%.2f] forward_tol=%.2f",
                                  candidates.size(), pillar_pair_gap_min_, pillar_pair_gap_max_, pair_forward_tolerance_);
            }
            return;
        }

        Detection detection = makeDetection(left_center, right_center, msg->header.stamp);
        if (!detection.valid) {
            publishStatus("invalid_geometry");
            if (readable_logs_) {
                logReadable("invalid_geometry", input->size(), down->size(), roi->size(),
                            cluster_indices.size(), candidates.size(), nullptr);
            }
            return;
        }

        history_.push_back(detection);
        while (static_cast<int>(history_.size()) > history_size_) {
            history_.pop_front();
        }

        // 连续多帧稳定才发布 valid，避免单帧误检直接驱动主流程起飞穿越。
        if (!isStable()) {
            publishStatus("stabilizing");
            publishMarkers(detection, frame_id, msg->header.stamp, false);
            if (readable_logs_) {
                logReadable("stabilizing", input->size(), down->size(), roi->size(),
                            cluster_indices.size(), candidates.size(), &detection);
            }
            return;
        }

        latest_ = averageHistory();
        publishGoals(latest_, frame_id, msg->header.stamp);
        publishMarkers(latest_, frame_id, msg->header.stamp, true);
        publishStatus("valid");
        if (readable_logs_) {
            logReadable("valid", input->size(), down->size(), roi->size(),
                        cluster_indices.size(), candidates.size(), &latest_);
        }
    }

    std::string outputFrame(const sensor_msgs::PointCloud2& msg) const {
        if (!frame_id_.empty()) return frame_id_;
        if (!msg.header.frame_id.empty()) return msg.header.frame_id;
        return "world";
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

        const double width_error = std::abs(out.width_x - 0.5) + std::abs(out.width_y - 0.5);
        const double height_score = std::min(out.height, 1.5);
        // 方柱尺寸越接近 0.5m、高度越高、点数越多，候选优先级越高。
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
            if (bin < 0) bin = 0;
            if (bin >= bins) bin = bins - 1;
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
                        Eigen::Vector3d& out_center_b) const {
        if (candidates.size() < 2) return false;

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

        if (found && !readable_logs_) {
            ROS_INFO_THROTTLE(1.0,
                              "[pillar_detector] selected pair %s/%s center_a=(%.2f %.2f %.2f) center_b=(%.2f %.2f %.2f) gap=%.2f score=%.2f",
                              candidateKindName(out_a.kind).c_str(),
                              candidateKindName(out_b.kind).c_str(),
                              out_center_a.x(), out_center_a.y(), out_center_a.z(),
                              out_center_b.x(), out_center_b.y(), out_center_b.z(),
                              (out_center_a - out_center_b).norm(), best_score);
        }
        return found;
    }

    void logClusterSummary(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                           const std::vector<pcl::PointIndices>& cluster_indices) const {
        if (cluster_indices.empty()) return;

        struct ClusterInfo {
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

        std::vector<ClusterInfo> infos;
        for (const auto& indices : cluster_indices) {
            PillarCandidate c;
            if (!buildCandidate(cloud, indices, c)) continue;
            ClusterInfo info;
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
            classifyCandidate(c);
            info.kind = c.kind;
            info.hypotheses = c.center_hypotheses.size();
            info.accepted = acceptCandidate(c);
            info.reason = c.reject_reason;
            infos.push_back(info);
        }

        std::sort(infos.begin(), infos.end(),
                  [](const ClusterInfo& a, const ClusterInfo& b) {
                      return a.points > b.points;
                  });

        const int count = std::min<int>(5, infos.size());
        std::ostringstream oss;
        oss << "[pillar_detector] cluster_summary";
        for (int i = 0; i < count; ++i) {
            const auto& c = infos[i];
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
        ROS_WARN_THROTTLE(1.0, "%s", oss.str().c_str());
    }

    void cacheDebugHypotheses(const std::vector<PillarCandidate>& candidates) {
        debug_hypotheses_.clear();
        for (const auto& c : candidates) {
            for (const auto& h : c.center_hypotheses) {
                debug_hypotheses_.push_back(h);
            }
        }
        if (debug_hypotheses_.size() > 30) {
            debug_hypotheses_.resize(30);
        }
    }

    Detection makeDetection(Eigen::Vector3d center_a,
                            Eigen::Vector3d center_b,
                            const ros::Time&) const {
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

        const Eigen::Vector2d span(center_b.x() - center_a.x(), center_b.y() - center_a.y());
        Eigen::Vector2d forward(span.y(), -span.x());
        if (forward.norm() < 1e-3) {
            forward = Eigen::Vector2d(1.0, 0.0);
        } else {
            forward.normalize();
        }

        // 两根柱子的连线是“门宽方向”，真正穿越方向应取它的垂线。
        // 选择让 pre_goal 更靠近 home 的一侧，避免把“柱后点”当成第一个目标。
        const Eigen::Vector2d corridor_xy(d.corridor.x(), d.corridor.y());
        const Eigen::Vector2d pre_xy = corridor_xy - goal_offset_ * forward;
        const Eigen::Vector2d post_xy = corridor_xy + goal_offset_ * forward;
        Eigen::Vector2d home_xy(0.0, 0.0);
        if (have_home_) {
            home_xy = Eigen::Vector2d(home_pos_.x(), home_pos_.y());
        }
        if ((pre_xy - home_xy).squaredNorm() > (post_xy - home_xy).squaredNorm()) {
            forward = -forward;
        }

        const double inflated_gap = span.norm() - safety_inflation_ * 2.0;
        // 安全膨胀只做风险提示；第一版仍发布目标点，让主流程按实际情况决定是否继续。
        if (inflated_gap < 0.45) {
            ROS_WARN_THROTTLE(1.0, "[pillar_detector] narrow pillar gap after inflation: %.2f", inflated_gap);
        }

        d.yaw = normalizeYaw(std::atan2(forward.y(), forward.x()));
        d.pre_goal = d.corridor - Eigen::Vector3d(goal_offset_ * forward.x(), goal_offset_ * forward.y(), 0.0);
        d.post_goal = d.corridor + Eigen::Vector3d(goal_offset_ * forward.x(), goal_offset_ * forward.y(), 0.0);
        d.pre_goal.z() = flight_z_;
        d.post_goal.z() = flight_z_;
        d.valid = true;

        if (!readable_logs_) {
            ROS_INFO_THROTTLE(1.0,
                              "[pillar_detector] goals corridor=(%.2f %.2f %.2f) pre=(%.2f %.2f %.2f) post=(%.2f %.2f %.2f) yaw=%.2f",
                              d.corridor.x(), d.corridor.y(), d.corridor.z(),
                              d.pre_goal.x(), d.pre_goal.y(), d.pre_goal.z(),
                              d.post_goal.x(), d.post_goal.y(), d.post_goal.z(),
                              d.yaw);
        }
        return d;
    }

    bool isStable() const {
        if (static_cast<int>(history_.size()) < stable_frames_) return false;
        const Detection& ref = history_.back();
        int stable = 0;
        // 用通道中心漂移量判断稳定性，比单独比较某一根柱子更贴近飞行目标。
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
            // 对稳定窗口做平均，给主流程一个更平滑的 pre/post goal。
            avg.left += d.left;
            avg.right += d.right;
            avg.corridor += d.corridor;
            avg.pre_goal += d.pre_goal;
            avg.post_goal += d.post_goal;
            avg.yaw += d.yaw;
        }
        const double n = static_cast<double>(history_.size());
        avg.left /= n;
        avg.right /= n;
        avg.corridor /= n;
        avg.pre_goal /= n;
        avg.post_goal /= n;
        avg.yaw = normalizeYaw(avg.yaw / n);
        avg.valid = true;
        return avg;
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

    void publishGoals(const Detection& d, const std::string& frame_id, const ros::Time& stamp) {
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

    int stableHistoryCount() const {
        if (history_.empty()) return 0;
        const Detection& ref = history_.back();
        int stable = 0;
        for (const auto& d : history_) {
            if ((d.corridor - ref.corridor).norm() < stability_threshold_) ++stable;
        }
        return stable;
    }

    bool shouldReadableLog() const {
        if (readable_log_period_ <= 0.0) return false;
        const ros::Time now = ros::Time::now();
        if (!last_readable_log_time_.isValid() ||
            (now - last_readable_log_time_).toSec() >= readable_log_period_) {
            last_readable_log_time_ = now;
            return true;
        }
        return false;
    }

    void logReadable(const std::string& status,
                     size_t input_size,
                     size_t down_size,
                     size_t roi_size,
                     size_t cluster_count,
                     size_t accepted_count,
                     const Detection* detection) const {
        if (!detection || !detection->valid) {
            (void)status;
            (void)input_size;
            (void)down_size;
            (void)roi_size;
            (void)cluster_count;
            (void)accepted_count;
            return;
        }
        if (!shouldReadableLog()) return;

        const int stable_count = stableHistoryCount();
        const double gap = (detection->right - detection->left).norm();
        ROS_INFO("[pillar_detector] DETECT pillar status=%s stable=%d/%d center=(%.2f %.2f %.2f) pre=(%.2f %.2f %.2f) post=(%.2f %.2f %.2f) offset_before=%.2f offset_after=%.2f gap=%.2f yaw=%.2f",
                 status.c_str(), stable_count, stable_frames_,
                 detection->corridor.x(), detection->corridor.y(), detection->corridor.z(),
                 detection->pre_goal.x(), detection->pre_goal.y(), detection->pre_goal.z(),
                 detection->post_goal.x(), detection->post_goal.y(), detection->post_goal.z(),
                 goal_offset_, goal_offset_,
                 gap, detection->yaw);
    }

    visualization_msgs::Marker makeSphereMarker(int id,
                                                const Eigen::Vector3d& p,
                                                const std::string& frame_id,
                                                const ros::Time& stamp,
                                                float r,
                                                float g,
                                                float b,
                                                double scale) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = "craic_pillars";
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

    void publishMarkers(const Detection& d,
                        const std::string& frame_id,
                        const ros::Time& stamp,
                        bool stable) {
        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(makeSphereMarker(1, d.left, frame_id, stamp, 1.0f, 0.2f, 0.2f, 0.22));
        arr.markers.push_back(makeSphereMarker(2, d.right, frame_id, stamp, 1.0f, 0.2f, 0.2f, 0.22));
        arr.markers.push_back(makeSphereMarker(3, d.corridor, frame_id, stamp, 0.1f, 0.9f, 1.0f, stable ? 0.20 : 0.12));
        arr.markers.push_back(makeSphereMarker(4, d.pre_goal, frame_id, stamp, 0.2f, 0.6f, 1.0f, stable ? 0.20 : 0.12));
        arr.markers.push_back(makeSphereMarker(5, d.post_goal, frame_id, stamp, 0.2f, 1.0f, 0.3f, stable ? 0.20 : 0.12));
        int id = 100;
        for (const auto& h : debug_hypotheses_) {
            arr.markers.push_back(makeSphereMarker(id++, h, frame_id, stamp, 1.0f, 0.85f, 0.1f, 0.08));
        }
        marker_pub_.publish(arr);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher pre_pub_;
    ros::Publisher post_pub_;
    ros::Publisher status_pub_;
    ros::Publisher center_pub_;
    ros::Publisher marker_pub_;

    std::string cloud_topic_;
    std::string odom_topic_;
    std::string frame_id_;
    double flight_z_;
    bool use_local_rule_roi_;
    bool have_home_ = false;
    Eigen::Vector3d home_pos_ = Eigen::Vector3d::Zero();
    double home_yaw_ = 0.0;
    double field_length_;
    double field_width_;
    double search_forward_min_;
    double search_forward_max_;
    double search_lateral_abs_;
    double roi_x_min_;
    double roi_x_max_;
    double roi_y_min_;
    double roi_y_max_;
    double roi_z_min_;
    double roi_z_max_;
    double voxel_leaf_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double pillar_width_;
    double pillar_min_visible_height_;
    double pillar_min_strong_height_;
    double pillar_full_min_width_;
    double pillar_full_max_width_;
    double pillar_face_span_min_;
    double pillar_face_span_max_;
    double pillar_thin_face_max_width_;
    double pillar_corner_max_width_;
    double pillar_long_reject_width_;
    double vertical_bin_size_;
    int pillar_min_vertical_bins_;
    double pillar_pair_gap_min_;
    double pillar_pair_gap_max_;
    double pillar_pair_expected_gap_;
    double pair_forward_tolerance_;
    double safety_inflation_;
    double goal_offset_;
    double stability_threshold_;
    int stable_frames_;
    int history_size_;
    bool readable_logs_ = false;
    double readable_log_period_ = 2.0;

    std::deque<Detection> history_;
    Detection latest_;
    std::vector<Eigen::Vector3d> debug_hypotheses_;
    mutable ros::Time last_readable_log_time_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "craic_pillar_detector");
    PillarDetectorNode node;
    ros::spin();
    return 0;
}
