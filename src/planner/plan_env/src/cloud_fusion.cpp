#include <plan_env/cloud_fusion.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace plan_env {
CloudFusion::CloudFusion(const Options& o, std::vector<double>& p,
                        std::vector<char>& inf, std::vector<short>& count,
                        std::vector<short>& hit, std::vector<char>& free,
                        std::vector<char>& active)
    : o_(o), probability_(p), inflated_(inf), free_count_(count), frame_hit_(hit),
      frame_free_(free), active_flag_(active), last_stamp_(-std::numeric_limits<double>::infinity()) {
  if (!(o_.resolution > 0) || !o_.origin.allFinite() || !o_.local_range.allFinite() ||
      (o_.size.array() <= 0).any() || (o_.local_range.array() <= 0).any() ||
      o_.blind < 0 || o_.blind_margin < 0 || o_.endpoint_margin < 0 ||
      o_.max_ray_length <= o_.blind || o_.angular_resolution <= 0 ||
      o_.free_confirmations < 1 || o_.free_confirmations > 32000 ||
      o_.max_free_rays < 1 || o_.inflate_xy < 0 || o_.inflate_z < 0 ||
      !(o_.min_log < o_.occ_log && o_.occ_log < o_.max_log) ||
      !(o_.hit_log > 0 && o_.miss_log < 0))
    throw std::invalid_argument("invalid cloud fusion options");
  for(double value:{o_.resolution,o_.blind,o_.blind_margin,o_.endpoint_margin,o_.max_ray_length,
                    o_.angular_resolution,o_.min_log,o_.max_log,o_.occ_log,o_.hit_log,o_.miss_log})
    if(!std::isfinite(value))throw std::invalid_argument("nonfinite cloud fusion option");
  if(o_.angular_resolution<.1 || o_.angular_resolution>5. || o_.max_free_rays>10000)
    throw std::invalid_argument("cloud fusion work budget out of range");
  const size_t n = size_t(o_.size.x()) * o_.size.y() * o_.size.z();
  if (p.size()!=n || inf.size()!=n || count.size()!=n || hit.size()!=n || free.size()!=n || active.size()!=n)
    throw std::invalid_argument("cloud fusion buffer size mismatch");
  // Indices only, not another dense map or an accumulation of raw scans.
  active_.reserve(std::min(n, size_t(65536)));
  hits_.reserve(std::min(n, size_t(32768)));
  misses_.reserve(std::min(n, size_t(32768)));
  reset();
}
bool CloudFusion::inside(const Eigen::Vector3i& i) const {
  return (i.array() >= 0).all() && (i.array() < o_.size.array()).all();
}
Eigen::Vector3i CloudFusion::index(const Eigen::Vector3d& p) const {
  return ((p-o_.origin)/o_.resolution).array().floor().cast<int>();
}
int CloudFusion::address(const Eigen::Vector3i& i) const {
  return (i.x()*o_.size.y()+i.y())*o_.size.z()+i.z();
}
Eigen::Vector3i CloudFusion::index(int id) const {
  int z=id%o_.size.z(); id/=o_.size.z();
  return Eigen::Vector3i(id/o_.size.y(),id%o_.size.y(),z);
}
Eigen::Vector3d CloudFusion::center(const Eigen::Vector3i& i) const {
  return o_.origin+(i.cast<double>()+Eigen::Vector3d::Constant(.5))*o_.resolution;
}
void CloudFusion::reset() {
  std::fill(probability_.begin(),probability_.end(),o_.min_log-.01);
  std::fill(inflated_.begin(),inflated_.end(),0);
  std::fill(free_count_.begin(),free_count_.end(),0);
  std::fill(frame_hit_.begin(),frame_hit_.end(),0);
  std::fill(frame_free_.begin(),frame_free_.end(),0);
  std::fill(active_flag_.begin(),active_flag_.end(),0);
  active_.clear(); hits_.clear(); misses_.clear();
  have_bounds_=false; last_stamp_=-std::numeric_limits<double>::infinity(); stats_=Stats();
}
void CloudFusion::clearInflation(const Eigen::Vector3i& lo,const Eigen::Vector3i& hi) {
  for(int x=lo.x();x<=hi.x();++x) for(int y=lo.y();y<=hi.y();++y) {
    int start=address(Eigen::Vector3i(x,y,lo.z()));
    std::fill(inflated_.begin()+start,inflated_.begin()+start+hi.z()-lo.z()+1,0);
  }
}
void CloudFusion::rebuildInflation() {
  clearInflation(lower_,upper_);
  for(int id:active_) {
    Eigen::Vector3i i=index(id);
    for(int x=std::max(0,i.x()-o_.inflate_xy);x<=std::min(o_.size.x()-1,i.x()+o_.inflate_xy);++x)
      for(int y=std::max(0,i.y()-o_.inflate_xy);y<=std::min(o_.size.y()-1,i.y()+o_.inflate_xy);++y) {
        int lo=std::max(0,i.z()-o_.inflate_z),hi=std::min(o_.size.z()-1,i.z()+o_.inflate_z);
        int start=address(Eigen::Vector3i(x,y,lo));
        std::fill(inflated_.begin()+start,inflated_.begin()+start+hi-lo+1,1);
      }
  }
}
bool CloudFusion::update(const Points& points,const Eigen::Vector3d& sensor,
                         const Eigen::Vector3d& map_center,double stamp,bool clear_free) {
  const Eigen::Vector3d map_high=o_.origin+o_.size.cast<double>()*o_.resolution;
  if (points.size()>200000 || !sensor.allFinite() || !map_center.allFinite() || !std::isfinite(stamp) ||
      stamp<=last_stamp_ || (map_center.array()<o_.origin.array()).any() ||
      (map_center.array()>=map_high.array()).any() ||
      (sensor-map_center).cwiseAbs().maxCoeff()>o_.local_range.maxCoeff()) return false;
  Stats next; next.input_points=points.size();
  // Build the finite, in-map, local return set before mutating the old map.
  struct Return { size_t point; double range; };
  std::vector<size_t> valid;
  valid.reserve(points.size());
  std::unordered_map<int,Return> sectors;
  sectors.reserve(std::min(points.size(),size_t(65536)));
  const double angle_scale=180./(std::acos(-1.)*o_.angular_resolution);
  const int elevations=int(std::ceil(180./o_.angular_resolution))+1;
  for(size_t k=0;k<points.size();++k) {
    const auto& p=points[k];
    if(!p.allFinite() || ((p-map_center).cwiseAbs().array()>o_.local_range.array()).any())continue;
    Eigen::Vector3i i=index(p); if(!inside(i))continue;
    Eigen::Vector3d d=p-sensor; double r=d.norm();
    if(r<std::max(o_.blind,1e-9) || !std::isfinite(r))continue;
    valid.push_back(k);
    int az=int(std::floor((std::atan2(d.y(),d.x())+std::acos(-1.))*angle_scale));
    int el=int(std::floor((std::atan2(d.z(),std::hypot(d.x(),d.y()))+std::acos(-1.)*.5)*angle_scale));
    int key=az*elevations+el;
    auto it=sectors.find(key);
    if(it==sectors.end() || r<it->second.range)sectors[key]=Return{k,r};
  }
  if(valid.empty())return false; // Empty/invalid scans are not free-space observations.
  next.valid_points=valid.size();
  if(have_bounds_)clearInflation(lower_,upper_);
  Eigen::Vector3d support(o_.inflate_xy*o_.resolution,o_.inflate_xy*o_.resolution,o_.inflate_z*o_.resolution);
  lower_=index((map_center-o_.local_range-support).cwiseMax(o_.origin)).cwiseMax(Eigen::Vector3i::Zero());
  upper_=index((map_center+o_.local_range+support).cwiseMin(map_high)).cwiseMin(o_.size-Eigen::Vector3i::Ones());
  have_bounds_=true;
  // Discard only cells that leave the local window; a blind-zone miss never prunes.
  size_t keep=0;
  for(int id:active_) {
    Eigen::Vector3i i=index(id);
    if((i.array()<lower_.array()).any() || (i.array()>upper_.array()).any()) {
      active_flag_[id]=0; free_count_[id]=0; probability_[id]=o_.min_log-.01; ++next.pruned;
    } else active_[keep++]=id;
  }
  active_.resize(keep); hits_.clear(); misses_.clear();
  for(size_t k:valid) {
    int id=address(index(points[k]));
    if(frame_hit_[id])continue;
    frame_hit_[id]=1; hits_.push_back(id);
    if(!active_flag_[id]){active_flag_[id]=1;active_.push_back(id);}
    // Immediate protection for every current hit, including rays omitted by budget.
    probability_[id]=std::min(o_.max_log,std::max(probability_[id],o_.occ_log+1e-6)+o_.hit_log);
    free_count_[id]=0;
  }
  next.hit_cells=hits_.size();
  std::vector<std::pair<int,Return>> rays(sectors.begin(),sectors.end());
  std::sort(rays.begin(),rays.end(),[](const auto&a,const auto&b){return a.first<b.first;});
  size_t nr=clear_free?std::min(rays.size(),size_t(o_.max_free_rays)):0;
  next.clearing_enabled=clear_free;
  const double protected_radius=o_.blind+o_.blind_margin+std::sqrt(3.)*o_.resolution*.5;
  const double end_margin=o_.endpoint_margin+std::sqrt(3.)*o_.resolution*.5;
  for(size_t n=0;n<nr;++n) {
    const Return& ret=rays[n*rays.size()/nr].second;
    const Eigen::Vector3d& measured=points[ret.point];
    Eigen::Vector3d end=sensor+(measured-sensor)*std::min(1.,o_.max_ray_length/ret.range);
    // The body can be inside the grid while the elevated LiDAR is above it.
    // Clip the ray to the grid instead of discarding that valid cloud frame.
    Eigen::Vector3d direction=end-sensor;
    double enter=0.,exit=1.;
    for(int j=0;j<3;++j) {
      double lo=o_.origin[j]+1e-8,hi=o_.origin[j]+o_.size[j]*o_.resolution-1e-8;
      if(std::abs(direction[j])<1e-12) {
        if(sensor[j]<lo || sensor[j]>hi){enter=2.;break;}
      } else {
        double a=(lo-sensor[j])/direction[j],b=(hi-sensor[j])/direction[j];
        if(a>b)std::swap(a,b);
        enter=std::max(enter,a);exit=std::min(exit,b);
      }
    }
    if(enter>=exit)continue;
    Eigen::Vector3d start=sensor+enter*direction;
    end=sensor+exit*direction;
    if((end-start).squaredNorm()<1e-18)continue;
    // Exact continuous-direction DDA, independent of legacy raycast's char flags.
    Eigen::Vector3i i=index(start),last=index(end);
    Eigen::Vector3d d=end-start,t,delta;
    Eigen::Vector3i step;
    for(int j=0;j<3;++j) {
      if(std::abs(d[j])<1e-12){step[j]=0;t[j]=delta[j]=std::numeric_limits<double>::infinity();}
      else {
        step[j]=d[j]>0?1:-1;
        double boundary=o_.origin[j]+(i[j]+(step[j]>0?1:0))*o_.resolution;
        t[j]=(boundary-start[j])/d[j];delta[j]=o_.resolution/std::abs(d[j]);
      }
    }
    const int max_steps=o_.size.sum()+3;
    for(int iter=0;iter<max_steps && inside(i);++iter) {
      int id=address(i);
      if(active_flag_[id] && !frame_hit_[id] && !frame_free_[id]) {
        Eigen::Vector3d p=center(i);
        // Grazing a voxel edge is not evidence that its occupied interior is free.
        double u=(p-start).dot(d)/d.squaredNorm();
        double lateral=(p-(start+std::max(0.,std::min(1.,u))*d)).squaredNorm();
        if((p-sensor).squaredNorm()>=protected_radius*protected_radius &&
           (p-end).squaredNorm()>=end_margin*end_margin &&
           lateral<=o_.resolution*o_.resolution/16.) {
          frame_free_[id]=1; misses_.push_back(id);
        }
      }
      if(i==last)break;
      double nearest=t.minCoeff();
      for(int j=0;j<3;++j)if(t[j]<=nearest+1e-12){i[j]+=step[j];t[j]+=delta[j];}
    }
    ++next.rays;
  }
  for(int id:misses_) {
    frame_free_[id]=0;
    free_count_[id]=std::min<int>(o_.free_confirmations,free_count_[id]+1);
    double updated=std::max(o_.min_log,probability_[id]+o_.miss_log);
    if(free_count_[id]>=o_.free_confirmations && updated<=o_.occ_log) {
      probability_[id]=updated; active_flag_[id]=0;free_count_[id]=0;++next.cleared;
    } else probability_[id]=std::max(updated,o_.occ_log+1e-6);
  }
  for(int id:hits_)frame_hit_[id]=0;
  active_.erase(std::remove_if(active_.begin(),active_.end(),[&](int id){return !active_flag_[id];}),active_.end());
  rebuildInflation();
  next.active=active_.size();
  next.work_capacity_bytes=(active_.capacity()+hits_.capacity()+misses_.capacity())*sizeof(int);
  stats_=next;last_stamp_=stamp;
  return true;
}
}
