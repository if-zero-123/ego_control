#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
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
        // 默认订阅 FAST-LIO 输出到 world/aligned 坐标系下的 MID360 点云。
        pnh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_registered_aligned");
        pnh_.param<std::string>("frame_id", frame_id_, "world");
        pnh_.param<double>("flight_z", flight_z_, 1.1);
        // FAST-LIO 的 world 坐标会随初始位姿建立，不直接等于 Gazebo 模型坐标。
        // 默认 ROI 放宽到起点前方的大场地区域，再靠聚类尺寸筛柱子。
        pnh_.param<double>("roi_x_min", roi_x_min_, -0.5);
        pnh_.param<double>("roi_x_max", roi_x_max_, 3.6);
        pnh_.param<double>("roi_y_min", roi_y_min_, -3.2);
        pnh_.param<double>("roi_y_max", roi_y_max_, 4.0);
        pnh_.param<double>("roi_z_min", roi_z_min_, 0.05);
        pnh_.param<double>("roi_z_max", roi_z_max_, 1.9);
        pnh_.param<double>("voxel_leaf", voxel_leaf_, 0.06);
        pnh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.24);
        pnh_.param<int>("min_cluster_size", min_cluster_size_, 15);
        pnh_.param<int>("max_cluster_size", max_cluster_size_, 8000);
        // 低高度起飞时 MID360 可能只扫到柱子的一部分，因此第一版允许部分柱体。
        pnh_.param<double>("pillar_min_height", pillar_min_height_, 0.55);
        pnh_.param<double>("pillar_min_width", pillar_min_width_, 0.30);
        pnh_.param<double>("pillar_min_face_width", pillar_min_face_width_, 0.04);
        pnh_.param<double>("pillar_max_width", pillar_max_width_, 0.95);
        pnh_.param<double>("pillar_pair_gap_min", pillar_pair_gap_min_, 1.0);
        pnh_.param<double>("pillar_pair_gap_max", pillar_pair_gap_max_, 2.3);
        pnh_.param<double>("pillar_pair_expected_gap", pillar_pair_expected_gap_, 1.6);
        pnh_.param<double>("pillar_pair_x_tolerance", pillar_pair_x_tolerance_, 0.8);
        pnh_.param<double>("safety_inflation", safety_inflation_, 0.40);
        pnh_.param<double>("goal_offset", goal_offset_, 0.90);
        pnh_.param<double>("stability_threshold", stability_threshold_, 0.18);
        pnh_.param<int>("stable_frames", stable_frames_, 3);
        pnh_.param<int>("history_size", history_size_, 5);

        cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &PillarDetectorNode::cloudCb, this);
        pre_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic/pillar_pre_goal", 1, true);
        post_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/craic/pillar_post_goal", 1, true);
        status_pub_ = nh_.advertise<std_msgs::String>("/craic/pillar_status", 1, true);
        // Marker 只用于 RViz 调试，不参与飞控闭环。
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/craic/pillar_markers", 1, true);

        ROS_INFO("[pillar_detector] cloud=%s frame=%s roi=[x %.2f..%.2f, y %.2f..%.2f, z %.2f..%.2f]",
                 cloud_topic_.c_str(), frame_id_.c_str(),
                 roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_);
    }

private:
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *input);
        if (input->empty()) {
            publishStatus("no_cloud");
            ROS_WARN_THROTTLE(1.0, "[pillar_detector] no_cloud on %s", cloud_topic_.c_str());
            return;
        }

        // 先体素降采样，减少欧式聚类的计算量，同时压掉局部噪声。
        pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(input);
        voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        voxel.filter(*down);

        // 用比赛场地尺寸做 ROI 约束：只看柱子可能出现的区域，并过滤地面附近点。
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(down);
        crop.setMin(Eigen::Vector4f(roi_x_min_, roi_y_min_, roi_z_min_, 1.0f));
        crop.setMax(Eigen::Vector4f(roi_x_max_, roi_y_max_, roi_z_max_, 1.0f));
        crop.filter(*roi);
        if (roi->empty()) {
            publishStatus("empty_roi");
            ROS_WARN_THROTTLE(1.0,
                              "[pillar_detector] empty_roi input=%zu down=%zu frame=%s roi=[x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f]",
                              input->size(), down->size(), msg->header.frame_id.c_str(),
                              roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_);
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
            // 每个聚类转成候选柱，再按尺寸条件过滤。
            if (!buildCandidate(*roi, indices, c)) continue;
            if (!acceptCandidate(c)) continue;
            candidates.push_back(c);
        }

        if (candidates.size() < 2) {
            publishStatus("not_enough_candidates");
            ROS_WARN_THROTTLE(1.0,
                              "[pillar_detector] not_enough_candidates input=%zu down=%zu roi=%zu clusters=%zu accepted=%zu frame=%s",
                              input->size(), down->size(), roi->size(), cluster_indices.size(), candidates.size(),
                              msg->header.frame_id.c_str());
            logClusterSummary(*roi, cluster_indices);
            return;
        }

        PillarCandidate left_candidate;
        PillarCandidate right_candidate;
        if (!selectBestPair(candidates, left_candidate, right_candidate)) {
            publishStatus("invalid_pair");
            ROS_WARN_THROTTLE(1.0,
                              "[pillar_detector] invalid_pair accepted=%zu gap=[%.2f..%.2f] x_tol=%.2f",
                              candidates.size(), pillar_pair_gap_min_, pillar_pair_gap_max_, pillar_pair_x_tolerance_);
            return;
        }

        Detection detection = makeDetection(left_candidate, right_candidate, msg->header.stamp);
        if (!detection.valid) {
            publishStatus("invalid_geometry");
            return;
        }

        history_.push_back(detection);
        while (static_cast<int>(history_.size()) > history_size_) {
            history_.pop_front();
        }

        // 连续多帧稳定才发布 valid，避免单帧误检直接驱动主流程起飞穿越。
        if (!isStable()) {
            publishStatus("stabilizing");
            publishMarkers(detection, msg->header.stamp, false);
            return;
        }

        latest_ = averageHistory();
        publishGoals(latest_, msg->header.stamp);
        publishMarkers(latest_, msg->header.stamp, true);
        publishStatus("valid");
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

        const double width_error = std::abs(out.width_x - 0.5) + std::abs(out.width_y - 0.5);
        const double height_score = std::min(out.height, 1.5);
        // 方柱尺寸越接近 0.5m、高度越高、点数越多，候选优先级越高。
        out.score = height_score - width_error + 0.0005 * out.points;
        return true;
    }

    bool acceptCandidate(const PillarCandidate& c) const {
        if (c.height < pillar_min_height_) return false;
        const double w_min = std::min(c.width_x, c.width_y);
        const double w_max = std::max(c.width_x, c.width_y);
        if (w_max > pillar_max_width_) return false;

        // 完整方柱：两个水平尺寸都接近柱子宽度。
        if (c.width_x >= pillar_min_width_ && c.width_y >= pillar_min_width_) return true;

        // 低高度或遮挡时，MID360 可能只看到方柱的一个侧面：一个尺寸接近柱宽，
        // 另一个尺寸很薄。允许这种“柱面候选”，后续再通过成对几何关系筛掉误检。
        return w_max >= pillar_min_width_ && w_min >= pillar_min_face_width_;
    }

    bool selectBestPair(const std::vector<PillarCandidate>& candidates,
                        PillarCandidate& out_a,
                        PillarCandidate& out_b) const {
        if (candidates.size() < 2) return false;

        double best_score = -std::numeric_limits<double>::infinity();
        bool found = false;

        for (size_t i = 0; i < candidates.size(); ++i) {
            for (size_t j = i + 1; j < candidates.size(); ++j) {
                const auto& a = candidates[i];
                const auto& b = candidates[j];
                const double dx = std::abs(a.center.x() - b.center.x());
                const double dy = std::abs(a.center.y() - b.center.y());
                const double gap = std::hypot(a.center.x() - b.center.x(),
                                              a.center.y() - b.center.y());

                if (gap < pillar_pair_gap_min_ || gap > pillar_pair_gap_max_) continue;
                if (dx > pillar_pair_x_tolerance_) continue;

                const double gap_penalty = std::abs(gap - pillar_pair_expected_gap_);
                const double score = a.score + b.score - gap_penalty - 0.5 * dx + 0.1 * dy;
                if (score > best_score) {
                    best_score = score;
                    out_a = a;
                    out_b = b;
                    found = true;
                }
            }
        }

        if (found) {
            ROS_INFO_THROTTLE(1.0,
                              "[pillar_detector] selected pair center_a=(%.2f %.2f %.2f) center_b=(%.2f %.2f %.2f) gap=%.2f score=%.2f",
                              out_a.center.x(), out_a.center.y(), out_a.center.z(),
                              out_b.center.x(), out_b.center.y(), out_b.center.z(),
                              (out_a.center - out_b.center).norm(), best_score);
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
            bool accepted = false;
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
            info.accepted = acceptCandidate(c);
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
                << " acc=" << c.accepted;
        }
        ROS_WARN_THROTTLE(1.0, "%s", oss.str().c_str());
    }

    Detection makeDetection(PillarCandidate a,
                            PillarCandidate b,
                            const ros::Time&) const {
        Detection d;
        if (a.center.y() > b.center.y()) std::swap(a, b);

        d.left = a.center;
        d.right = b.center;
        d.corridor = 0.5 * (a.center + b.center);
        d.corridor.z() = flight_z_;

        const Eigen::Vector2d span(b.center.x() - a.center.x(), b.center.y() - a.center.y());
        Eigen::Vector2d forward(span.y(), -span.x());
        if (forward.norm() < 1e-3) {
            forward = Eigen::Vector2d(1.0, 0.0);
        } else {
            forward.normalize();
        }

        // 两根柱子的连线是“门宽方向”，真正穿越方向应取它的垂线。
        // 选择让 pre_goal 更靠近起点原点的一侧，避免把“柱后点”当成第一个目标。
        const Eigen::Vector2d corridor_xy(d.corridor.x(), d.corridor.y());
        const Eigen::Vector2d pre_xy = corridor_xy - goal_offset_ * forward;
        const Eigen::Vector2d post_xy = corridor_xy + goal_offset_ * forward;
        if (pre_xy.norm() > post_xy.norm()) {
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

        ROS_INFO_THROTTLE(1.0,
                          "[pillar_detector] goals corridor=(%.2f %.2f %.2f) pre=(%.2f %.2f %.2f) post=(%.2f %.2f %.2f) yaw=%.2f",
                          d.corridor.x(), d.corridor.y(), d.corridor.z(),
                          d.pre_goal.x(), d.pre_goal.y(), d.pre_goal.z(),
                          d.post_goal.x(), d.post_goal.y(), d.post_goal.z(),
                          d.yaw);
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
                                        const ros::Time& stamp) const {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = p.z();
        pose.pose.orientation = yawToQuat(yaw);
        return pose;
    }

    void publishGoals(const Detection& d, const ros::Time& stamp) {
        pre_pub_.publish(makePose(d.pre_goal, d.yaw, stamp));
        post_pub_.publish(makePose(d.post_goal, d.yaw, stamp));
    }

    void publishStatus(const std::string& status) {
        std_msgs::String msg;
        msg.data = status;
        status_pub_.publish(msg);
    }

    visualization_msgs::Marker makeSphereMarker(int id,
                                                const Eigen::Vector3d& p,
                                                const ros::Time& stamp,
                                                float r,
                                                float g,
                                                float b,
                                                double scale) const {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_;
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

    void publishMarkers(const Detection& d, const ros::Time& stamp, bool stable) {
        visualization_msgs::MarkerArray arr;
        arr.markers.push_back(makeSphereMarker(1, d.left, stamp, 1.0f, 0.2f, 0.2f, 0.22));
        arr.markers.push_back(makeSphereMarker(2, d.right, stamp, 1.0f, 0.2f, 0.2f, 0.22));
        arr.markers.push_back(makeSphereMarker(3, d.pre_goal, stamp, 0.2f, 0.6f, 1.0f, stable ? 0.20 : 0.12));
        arr.markers.push_back(makeSphereMarker(4, d.post_goal, stamp, 0.2f, 1.0f, 0.3f, stable ? 0.20 : 0.12));
        marker_pub_.publish(arr);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher pre_pub_;
    ros::Publisher post_pub_;
    ros::Publisher status_pub_;
    ros::Publisher marker_pub_;

    std::string cloud_topic_;
    std::string frame_id_;
    double flight_z_;
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
    double pillar_min_height_;
    double pillar_min_width_;
    double pillar_min_face_width_;
    double pillar_max_width_;
    double pillar_pair_gap_min_;
    double pillar_pair_gap_max_;
    double pillar_pair_expected_gap_;
    double pillar_pair_x_tolerance_;
    double safety_inflation_;
    double goal_offset_;
    double stability_threshold_;
    int stable_frames_;
    int history_size_;

    std::deque<Detection> history_;
    Detection latest_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "craic_pillar_detector");
    PillarDetectorNode node;
    ros::spin();
    return 0;
}
