#include "plan_env/grid_map.h"
#include <std_msgs/String.h>
#include <sstream>
#include <chrono>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);
  node_.param("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_, -0.1);
  node_.param("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_, -0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 1.0);

  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);
  node_.param("grid_map/cloud_fusion", cloud_fusion_enabled_, false);

  if( mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
  extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
  }

  // use odometry and point cloud
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", cloud_fusion_enabled_ ? 1 : 10, &GridMap::cloudCallback, this);
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this);

  occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.11), &GridMap::visCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_.fromSec(0);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  if (cloud_fusion_enabled_) {
    plan_env::CloudFusion::Options o;
    o.resolution = mp_.resolution_; o.origin = mp_.map_origin_;
    o.size = mp_.map_voxel_num_; o.local_range = mp_.local_update_range_;
    o.inflate_xy = std::ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    o.inflate_z = 1; // Preserve this cloud branch's existing vertical inflation.
    o.max_ray_length = mp_.max_ray_length_;
    o.min_log = mp_.clamp_min_log_; o.max_log = mp_.clamp_max_log_;
    o.occ_log = mp_.min_occupancy_log_;
    o.hit_log = mp_.prob_hit_log_; o.miss_log = mp_.prob_miss_log_;
    node_.param("grid_map/cloud_blind", o.blind, 0.35);
    node_.param("grid_map/cloud_blind_margin", o.blind_margin, 0.10);
    node_.param("grid_map/cloud_endpoint_margin", o.endpoint_margin, 0.15);
    node_.param("grid_map/cloud_free_confirmations", o.free_confirmations, 3);
    node_.param("grid_map/cloud_max_free_rays", o.max_free_rays, 3000);
    node_.param("grid_map/cloud_angular_resolution", o.angular_resolution, 0.5);
    node_.param("grid_map/cloud_pose_tolerance", cloud_pose_tolerance_, 0.05);
    node_.param("grid_map/cloud_timeout", cloud_timeout_, 0.5);
    node_.param("grid_map/cloud_jump_distance", cloud_jump_distance_, 0.5);
    node_.param("grid_map/cloud_scan_duration", cloud_scan_duration_, 0.10);
    node_.param("grid_map/cloud_clear_max_origin_span", cloud_clear_max_span_, 0.15);
    node_.param("grid_map/cloud_clear_max_pose_gap", cloud_clear_max_pose_gap_, 0.15);
    node_.param("grid_map/cloud_clear_acceleration_bound", cloud_clear_acc_bound_, 6.0);
    node_.param("grid_map/cloud_clear_angular_acceleration_bound", cloud_clear_angular_acc_bound_, 20.0);
    node_.param("grid_map/cloud_clear_origin_margin", cloud_clear_origin_margin_, 0.0025);
    node_.param("grid_map/cloud_free_interior_margin", o.free_interior_margin, mp_.resolution_*.05);
    if(node_.hasParam("grid_map/cloud_clear_motion_limit"))
      ROS_WARN("cloud_clear_motion_limit is obsolete; clearing now uses a conservative scan-origin envelope");
    node_.param<std::string>("grid_map/cloud_frame", cloud_expected_frame_, "camera_init");
    std::string pose_topic;
    node_.param<std::string>("grid_map/cloud_pose_topic", pose_topic, "/Odometry");
    std::vector<double> translation;
    if (!node_.getParam("grid_map/cloud_sensor_translation", translation) || translation.size()!=3 ||
        !std::isfinite(translation[0]) || !std::isfinite(translation[1]) || !std::isfinite(translation[2]) ||
        !std::isfinite(cloud_pose_tolerance_) || cloud_pose_tolerance_<=0 ||
        !std::isfinite(cloud_timeout_) || cloud_timeout_<=0 ||
        !std::isfinite(cloud_jump_distance_) || cloud_jump_distance_<=0 ||
        !std::isfinite(cloud_scan_duration_) || cloud_scan_duration_<=0 || cloud_scan_duration_>.2 ||
        !std::isfinite(cloud_clear_max_span_) || cloud_clear_max_span_<=0 || cloud_clear_max_span_>.25 ||
        !std::isfinite(cloud_clear_max_pose_gap_) || cloud_clear_max_pose_gap_<=0 || cloud_clear_max_pose_gap_>.2 ||
        !std::isfinite(cloud_clear_acc_bound_) || cloud_clear_acc_bound_<6. || cloud_clear_acc_bound_>30. ||
        !std::isfinite(cloud_clear_angular_acc_bound_) || cloud_clear_angular_acc_bound_<20. ||
        cloud_clear_angular_acc_bound_>100. ||
        !std::isfinite(cloud_clear_origin_margin_) || cloud_clear_origin_margin_<.0025 ||
        cloud_clear_origin_margin_>mp_.resolution_*.25 || cloud_expected_frame_.empty())
      throw std::invalid_argument("cloud fusion requires calibrated sensor translation and valid timing/frame parameters");
    cloud_sensor_translation_ = Eigen::Vector3d(translation[0],translation[1],translation[2]);
    cloud_fusion_.reset(new plan_env::CloudFusion(o,md_.occupancy_buffer_,md_.occupancy_buffer_inflate_,
                          md_.count_hit_,md_.count_hit_and_miss_,md_.flag_traverse_,md_.flag_rayend_));
    cloud_pose_sub_ = node_.subscribe<nav_msgs::Odometry>(pose_topic,20,&GridMap::cloudPoseCallback,this);
    cloud_status_pub_ = node_.advertise<std_msgs::String>("grid_map/cloud_fusion_status",1,true);
    md_.flag_depth_odom_timeout_ = true; // Require synchronized map data before planning.
    ROS_INFO("Cloud occupancy fusion enabled; pose=%s, free-ray budget=%d",pose_topic.c_str(),o.max_free_rays);
  }

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v+=skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u+=skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{

  publishMapInflate(true);
  publishMap();
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (cloud_fusion_enabled_) {
    if (cloud_last_good_.isZero() || cloud_stable_frames_<3 ||
        (ros::Time::now()-cloud_last_good_).toSec()<-.05 ||
        (ros::Time::now()-cloud_last_good_).toSec()>cloud_timeout_) {
      md_.flag_depth_odom_timeout_=true;
      cloudStatus("waiting_or_stale_cloud");
    }
    return;
  }
  if (md_.last_occ_update_time_.toSec() < 1.0 ) md_.last_occ_update_time_ = ros::Time::now();
  
  if (!md_.occ_need_update_)
  {
    if ( md_.flag_use_depth_fusion && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_ )
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f", 
        ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }
  md_.last_occ_update_time_ = ros::Time::now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  if (cloud_fusion_enabled_) return;
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

namespace {
Eigen::Vector3d cloudPosition(const nav_msgs::Odometry& p) {
  return Eigen::Vector3d(p.pose.pose.position.x,p.pose.pose.position.y,p.pose.pose.position.z);
}
Eigen::Quaterniond cloudRotation(const nav_msgs::Odometry& p) {
  const auto& q=p.pose.pose.orientation;
  return Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized();
}
}

void GridMap::cloudStatus(const std::string& reason,double update_ms) {
  if(!cloud_fusion_)return;
  const auto& s=cloud_fusion_->stats();
  std::ostringstream out;
  out<<"{\"ready\":"<<(md_.flag_depth_odom_timeout_?"false":"true")
     <<",\"reason\":\""<<reason<<"\",\"stamp\":"<<std::fixed<<cloud_last_good_.toSec()
     <<",\"age\":"<<(cloud_last_good_.isZero()?-1.:(ros::Time::now()-cloud_last_good_).toSec())
     <<",\"update_ms\":"<<update_ms<<",\"points\":"<<s.input_points
     <<",\"valid_points\":"<<s.valid_points<<",\"hit_cells\":"<<s.hit_cells
     <<",\"rays\":"<<s.rays<<",\"cleared\":"<<s.cleared<<",\"pruned\":"<<s.pruned
     <<",\"active_cells\":"<<s.active<<",\"clearing_enabled\":"<<(s.clearing_enabled?"true":"false")
     <<",\"clear_reason\":\""<<cloud_clear_reason_<<"\",\"origin_uncertainty\":"<<s.origin_uncertainty
     <<",\"uncertain_cells\":"<<s.uncertain_cells
     <<",\"work_capacity_bytes\":"<<s.work_capacity_bytes<<"}";
  std_msgs::String msg;msg.data=out.str();cloud_status_pub_.publish(msg);
}

void GridMap::cloudPoseCallback(const nav_msgs::OdometryConstPtr& msg) {
  const auto& q=msg->pose.pose.orientation;
  double qnorm=q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
  double age=(ros::Time::now()-msg->header.stamp).toSec();
  if(msg->header.stamp.isZero() || age>cloud_timeout_ || age<-.05 || msg->header.frame_id!=cloud_expected_frame_ ||
     !cloudPosition(*msg).allFinite() || !std::isfinite(qnorm) || qnorm<.5 || qnorm>1.5)return;
  if(!cloud_pose_cache_.empty()) {
    const auto& last=cloud_pose_cache_.back();
    if(msg->header.stamp<=last.header.stamp)return;
    if((cloudPosition(*msg)-cloudPosition(last)).norm()>cloud_jump_distance_ ||
       cloudRotation(*msg).angularDistance(cloudRotation(last))>.8) {
      ROS_WARN("Cloud fusion: localization jump; reset historical map and require fresh scans");
      cloud_fusion_->reset();cloud_pose_cache_.clear();
      // Preserve only the pending cloud from exactly this new pose timestamp.
      // Older-world pending endpoints must never enter the reset map.
      if(cloud_pending_ && cloud_pending_->header.stamp!=msg->header.stamp)cloud_pending_.reset();
      cloud_stable_frames_=0;cloud_last_good_=ros::Time(0);
      md_.has_cloud_=false;
      md_.flag_depth_odom_timeout_=true;
    }
  }
  cloud_pose_cache_.push_back(*msg);
  while(cloud_pose_cache_.size()>100)cloud_pose_cache_.pop_front();
  if(cloud_pending_) {
    auto pending=cloud_pending_;cloud_pending_.reset();
    fuseCloud(pending); // Topic delivery order may put the cloud before its pose.
  }
}

bool GridMap::cloudPoseAt(const ros::Time& stamp,nav_msgs::Odometry& pose) const {
  if(cloud_pose_cache_.empty())return false;
  auto next=std::lower_bound(cloud_pose_cache_.begin(),cloud_pose_cache_.end(),stamp,
                            [](const nav_msgs::Odometry& p,const ros::Time& s){return p.header.stamp<s;});
  if(next!=cloud_pose_cache_.end() && next->header.stamp==stamp){pose=*next;return true;}
  if(next!=cloud_pose_cache_.begin() && next!=cloud_pose_cache_.end()) {
    const auto& prev=*(next-1);
    double a=(stamp-prev.header.stamp).toSec(),b=(next->header.stamp-stamp).toSec();
    if(a<=cloud_pose_tolerance_ && b<=cloud_pose_tolerance_) {
      double t=a/(a+b);pose=prev;pose.header.stamp=stamp;
      Eigen::Vector3d p=(1-t)*cloudPosition(prev)+t*cloudPosition(*next);
      Eigen::Quaterniond q=cloudRotation(prev).slerp(t,cloudRotation(*next));
      pose.pose.pose.position.x=p.x();pose.pose.pose.position.y=p.y();pose.pose.pose.position.z=p.z();
      pose.pose.pose.orientation.w=q.w();pose.pose.pose.orientation.x=q.x();
      pose.pose.pose.orientation.y=q.y();pose.pose.pose.orientation.z=q.z();return true;
    }
  }
  // Use only a nearby valid pose, never the latest unrelated MAVROS pose.
  const nav_msgs::Odometry* closest=nullptr;
  if(next!=cloud_pose_cache_.end())closest=&*next;
  if(next!=cloud_pose_cache_.begin()) {
    const auto& prev=*(next-1);
    if(!closest || std::abs((stamp-prev.header.stamp).toSec())<std::abs((stamp-closest->header.stamp).toSec()))closest=&prev;
  }
  if(closest && std::abs((stamp-closest->header.stamp).toSec())<=cloud_pose_tolerance_){pose=*closest;return true;}
  return false;
}

bool GridMap::cloudOriginEnvelope(const ros::Time& stamp,Eigen::Vector3d& center,
                                  Eigen::Vector3d& half_extent) {
  cloud_clear_reason_="insufficient_scan_pose_coverage";
  const ros::Time first=stamp-ros::Duration(cloud_scan_duration_);
  if(cloud_pose_cache_.empty() || cloud_pose_cache_.front().header.stamp>first ||
     cloud_pose_cache_.back().header.stamp<stamp)return false;
  nav_msgs::Odometry a,b;
  if(!cloudPoseAt(first,a) || !cloudPoseAt(stamp,b))return false;
  // Require bracketing poses: nearest-pose extrapolation is never clearing evidence.
  std::vector<nav_msgs::Odometry> samples;
  samples.push_back(a);
  for(const auto& p:cloud_pose_cache_)
    if(p.header.stamp>first && p.header.stamp<stamp)samples.push_back(p);
  samples.push_back(b);
  auto sensor=[this](const nav_msgs::Odometry& p) {
    return Eigen::Vector3d(cloudPosition(p)+cloudRotation(p)*cloud_sensor_translation_);
  };
  Eigen::Vector3d lo=sensor(a),hi=lo;
  double curve_margin=0.;
  // Use complete source-pose intervals, including intervals bracketing the scan
  // boundaries. Short interpolated subintervals would understate their error.
  for(size_t i=1;i<cloud_pose_cache_.size();++i) {
    const auto& prev=cloud_pose_cache_[i-1];const auto& next=cloud_pose_cache_[i];
    if(next.header.stamp<=first || prev.header.stamp>=stamp)continue;
    const double dt=(next.header.stamp-prev.header.stamp).toSec();
    if(dt<=0 || dt>cloud_clear_max_pose_gap_) {
      cloud_clear_reason_="scan_pose_gap";return false;
    }
    const double angle=cloudRotation(prev).angularDistance(cloudRotation(next));
    if(angle>.35) {cloud_clear_reason_="scan_rotation_too_large";return false;}
    curve_margin=std::max(curve_margin,cloud_clear_acc_bound_*dt*dt/8.+
      cloud_sensor_translation_.norm()*(1.-std::cos(angle*.5)+cloud_clear_angular_acc_bound_*dt*dt/8.));
  }
  for(size_t i=0;i<samples.size();++i) {
    Eigen::Vector3d s=sensor(samples[i]);lo=lo.cwiseMin(s);hi=hi.cwiseMax(s);
  }
  if((hi-lo).norm()>cloud_clear_max_span_) {
    cloud_clear_reason_="scan_origin_span_too_large";return false;
  }
  center=(lo+hi)*.5;
  half_extent=(hi-lo)*.5+Eigen::Vector3d::Constant(curve_margin+cloud_clear_origin_margin_);
  cloud_clear_reason_="bounded_scan_origins";
  return true;
}

void GridMap::fuseCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
  const double age=(ros::Time::now()-msg->header.stamp).toSec();
  size_t count=size_t(msg->width)*msg->height;
  if(msg->header.stamp.isZero() || age>cloud_timeout_ || age<-.05 ||
     msg->header.frame_id!=cloud_expected_frame_ || msg->is_bigendian || count>200000 ||
     size_t(msg->row_step)<size_t(msg->width)*msg->point_step ||
     msg->data.size()<size_t(msg->row_step)*msg->height) {
    cloudStatus("invalid_cloud_frame_time_or_size");return;
  }
  for(const std::string name:{"x","y","z"}) {
    auto field=std::find_if(msg->fields.begin(),msg->fields.end(),[&](const sensor_msgs::PointField& f){return f.name==name;});
    if(field==msg->fields.end() || field->datatype!=sensor_msgs::PointField::FLOAT32 ||
       field->count!=1 || field->offset+sizeof(float)>msg->point_step){cloudStatus("invalid_xyz_fields");return;}
  }
  nav_msgs::Odometry pose;
  if(!cloudPoseAt(msg->header.stamp,pose)) {
    cloud_pending_=msg;cloudStatus("waiting_matching_lio_pose");return;
  }
  Eigen::Vector3d sensor=cloudPosition(pose)+cloudRotation(pose)*cloud_sensor_translation_;
  Eigen::Vector3d half_extent=Eigen::Vector3d::Zero();
  bool clear_free=cloudOriginEnvelope(msg->header.stamp,sensor,half_extent);
  const auto start=std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg,cloud);
  plan_env::CloudFusion::Points points;points.reserve(cloud.size());
  for(const auto& p:cloud)points.emplace_back(p.x,p.y,p.z);
  if(!cloud_fusion_->update(points,sensor,cloudPosition(pose),msg->header.stamp.toSec(),clear_free,half_extent)) {
    cloudStatus("empty_invalid_or_out_of_order_scan");return;
  }
  md_.local_bound_min_=cloud_fusion_->lowerBound();md_.local_bound_max_=cloud_fusion_->upperBound();
  // Existing ceiling only; do not add a minimum-height constraint.
  if(mp_.virtual_ceil_height_>-.5) {
    int z=int(std::floor((mp_.virtual_ceil_height_-mp_.map_origin_.z())*mp_.resolution_inv_))-1;
    if(z>=0 && z<mp_.map_voxel_num_.z())
      for(int x=md_.local_bound_min_.x();x<=md_.local_bound_max_.x();++x)
        for(int y=md_.local_bound_min_.y();y<=md_.local_bound_max_.y();++y)
          md_.occupancy_buffer_inflate_[toAddress(x,y,z)]=1;
  }
  if(!cloud_last_good_.isZero() && (msg->header.stamp-cloud_last_good_).toSec()>cloud_timeout_)
    cloud_stable_frames_=0;
  cloud_last_good_=msg->header.stamp;cloud_pending_.reset();
  cloud_stable_frames_=std::min(3,cloud_stable_frames_+1);
  md_.has_cloud_=true;md_.flag_depth_odom_timeout_=cloud_stable_frames_<3;
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  cloudStatus(md_.flag_depth_odom_timeout_?"warming_up":(clear_free?"ok":"hits_only_motion_or_pose_uncertainty"),ms);
  ROS_INFO_THROTTLE(2.,"Cloud fusion: %zu hits, %zu rays, %zu remembered, %.1fms",cloud_fusion_->stats().hit_cells,
                    cloud_fusion_->stats().rays,cloud_fusion_->stats().active,ms);
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  if (cloud_fusion_enabled_) { fuseCloud(img); return; }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2))
    {

      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt))
              continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  if (cloud_fusion_enabled_) return;
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}
