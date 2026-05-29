#include <cmath>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

namespace {

class CloudSelfFilterNode {
public:
    CloudSelfFilterNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("input_cloud_topic", input_cloud_topic_, "/cloud_registered_aligned");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
        pnh_.param<std::string>("output_cloud_topic", output_cloud_topic_, "/craic/cloud_filtered");
        pnh_.param<double>("self_radius_xy", self_radius_xy_, 0.45);
        pnh_.param<double>("self_radius_z", self_radius_z_, 0.45);
        pnh_.param<double>("min_z", min_z_, 0.03);
        pnh_.param<double>("max_z", max_z_, 2.2);

        cloud_sub_ = nh_.subscribe(input_cloud_topic_, 1, &CloudSelfFilterNode::cloudCb, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &CloudSelfFilterNode::odomCb, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);

        ROS_INFO("[cloud_self_filter] input=%s output=%s odom=%s self_radius_xy=%.2f self_radius_z=%.2f z=[%.2f %.2f]",
                 input_cloud_topic_.c_str(), output_cloud_topic_.c_str(), odom_topic_.c_str(),
                 self_radius_xy_, self_radius_z_, min_z_, max_z_);
    }

private:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_x_ = msg->pose.pose.position.x;
        odom_y_ = msg->pose.pose.position.y;
        odom_z_ = msg->pose.pose.position.z;
        have_odom_ = true;
    }

    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ> input;
        pcl::PointCloud<pcl::PointXYZ> output;
        pcl::fromROSMsg(*msg, input);
        output.reserve(input.size());

        const double self_radius_xy2 = self_radius_xy_ * self_radius_xy_;
        for (const auto& p : input.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            if (p.z < min_z_ || p.z > max_z_) continue;

            if (have_odom_) {
                const double dx = p.x - odom_x_;
                const double dy = p.y - odom_y_;
                const double dz = std::abs(p.z - odom_z_);
                if (dx * dx + dy * dy < self_radius_xy2 && dz < self_radius_z_) {
                    continue;
                }
            }

            output.push_back(p);
        }

        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(output, out_msg);
        out_msg.header = msg->header;
        cloud_pub_.publish(out_msg);

        ROS_INFO_THROTTLE(1.0, "[cloud_self_filter] cloud %zu -> %zu points", input.size(), output.size());
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cloud_pub_;

    std::string input_cloud_topic_;
    std::string odom_topic_;
    std::string output_cloud_topic_;
    double self_radius_xy_ = 0.45;
    double self_radius_z_ = 0.45;
    double min_z_ = 0.03;
    double max_z_ = 2.2;

    bool have_odom_ = false;
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    double odom_z_ = 0.0;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "craic_cloud_self_filter");
    CloudSelfFilterNode node;
    ros::spin();
    return 0;
}
