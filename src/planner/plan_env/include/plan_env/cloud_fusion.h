#ifndef PLAN_ENV_CLOUD_FUSION_H
#define PLAN_ENV_CLOUD_FUSION_H

#include <Eigen/Eigen>
#include <cstdint>
#include <vector>

namespace plan_env {

// Uses GridMap's existing arrays. Cloud and depth fusion are mutually exclusive.
// The two char arrays are boolean flags here, never wrapping frame counters.
class CloudFusion {
public:
  struct Options {
    double resolution = 0.05;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3i size = Eigen::Vector3i::Ones();
    Eigen::Vector3d local_range = Eigen::Vector3d(5.5, 5.5, 2.5);
    int inflate_xy = 5, inflate_z = 1;
    double blind = 0.35, blind_margin = 0.10, endpoint_margin = 0.15;
    double max_ray_length = 5.0, angular_resolution = 0.5;
    int free_confirmations = 3, max_free_rays = 3000;
    double min_log = -1.9924301647, max_log = 2.1972245773;
    double occ_log = 1.3862943611, hit_log = 0.6190392084, miss_log = -0.6190392084;
  };
  struct Stats {
    size_t input_points = 0, valid_points = 0, hit_cells = 0;
    size_t rays = 0, cleared = 0, pruned = 0, active = 0;
    size_t work_capacity_bytes = 0;
    bool clearing_enabled = false;
  };
  using Points = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

  CloudFusion(const Options& options, std::vector<double>& probability,
              std::vector<char>& inflated, std::vector<short>& free_count,
              std::vector<short>& frame_hit, std::vector<char>& frame_free,
              std::vector<char>& active_flag);
  bool update(const Points& points, const Eigen::Vector3d& sensor_origin,
              const Eigen::Vector3d& map_center, double stamp, bool clear_free = true);
  void reset();
  const Stats& stats() const { return stats_; }
  Eigen::Vector3i lowerBound() const { return lower_; }
  Eigen::Vector3i upperBound() const { return upper_; }

private:
  Options o_;
  std::vector<double>& probability_;
  std::vector<char>& inflated_;
  std::vector<short>& free_count_;
  std::vector<short>& frame_hit_;
  std::vector<char>& frame_free_;
  std::vector<char>& active_flag_;
  std::vector<int> active_, hits_, misses_;
  Eigen::Vector3i lower_, upper_;
  bool have_bounds_ = false;
  double last_stamp_;
  Stats stats_;
  bool inside(const Eigen::Vector3i& i) const;
  Eigen::Vector3i index(const Eigen::Vector3d& p) const;
  Eigen::Vector3i index(int address) const;
  Eigen::Vector3d center(const Eigen::Vector3i& i) const;
  int address(const Eigen::Vector3i& i) const;
  void clearInflation(const Eigen::Vector3i& lo, const Eigen::Vector3i& hi);
  void rebuildInflation();
};
}
#endif
