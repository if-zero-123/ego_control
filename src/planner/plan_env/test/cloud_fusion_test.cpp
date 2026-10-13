#include <gtest/gtest.h>
#include <plan_env/cloud_fusion.h>
#include <limits>
#include <memory>
#include <stdexcept>

using plan_env::CloudFusion;
class FusionTest : public ::testing::Test {
protected:
  CloudFusion::Options o;
  std::vector<double> p;
  std::vector<char> inf,ff,active;
  std::vector<short> count,hit;
  std::unique_ptr<CloudFusion> f;
  Eigen::Vector3d sensor{.025,.025,1.025},obstacle{1.025,.025,1.025},far{2.025,.025,1.025};
  void SetUp() override {
    o.origin=Eigen::Vector3d(-4.,-4.,0.);o.size=Eigen::Vector3i(160,160,50);
    o.local_range=Eigen::Vector3d(2.5,2.5,2.5);o.inflate_xy=1;o.inflate_z=1;
    rebuild();
  }
  void rebuild() {
    size_t n=size_t(o.size.x())*o.size.y()*o.size.z();
    p.resize(n);inf.resize(n);ff.resize(n);active.resize(n);count.resize(n);hit.resize(n);
    f.reset(new CloudFusion(o,p,inf,count,hit,ff,active));
  }
  CloudFusion::Points points(std::initializer_list<Eigen::Vector3d> list) {
    CloudFusion::Points v;for(auto&x:list)v.push_back(x);return v;
  }
  int id(const Eigen::Vector3d& v) {
    Eigen::Vector3i i=((v-o.origin)/o.resolution).array().floor().cast<int>();
    return (i.x()*o.size.y()+i.y())*o.size.z()+i.z();
  }
  bool occupied(const Eigen::Vector3d& v) {return inf[id(v)]!=0;}
  bool update(std::initializer_list<Eigen::Vector3d> v,double stamp,bool clear=true) {
    return f->update(points(v),sensor,sensor,stamp,clear);
  }
};
TEST_F(FusionTest,FirstHitIsImmediatelyProtected) {
  ASSERT_TRUE(update({obstacle},1));EXPECT_TRUE(occupied(obstacle));EXPECT_GT(p[id(obstacle)],o.occ_log);
}
TEST_F(FusionTest,EmptyAndInvalidScansDoNotEraseHistory) {
  update({obstacle},1);
  EXPECT_FALSE(update({},2));
  EXPECT_FALSE(update({Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())},3));
  EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,OutOfOrderScansCannotClearObstacle) {
  update({obstacle},5);EXPECT_FALSE(update({far},4));EXPECT_FALSE(update({far},5));EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,BlindZoneSurvivesLongerValidReturns) {
  update({obstacle},1);
  Eigen::Vector3d near=obstacle-Eigen::Vector3d(.2,0,0);
  for(int t=2;t<20;++t)ASSERT_TRUE(f->update(points({far}),near,near,t));
  EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,BlindMarginProtectsVoxelVolume) {
  update({obstacle},1);
  Eigen::Vector3d near=obstacle-Eigen::Vector3d(.4,0,0);
  for(int t=2;t<10;++t)f->update(points({far}),near,near,t);
  EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,OcclusionDoesNotClearObstacleBehindReturn) {
  update({obstacle},1);
  for(int t=2;t<10;++t)update({sensor+Eigen::Vector3d(.5,0,0)},t);
  EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,OneFreeFrameDoesNotEraseObstacle) {
  update({obstacle},1);update({far},2);EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,RepeatedValidFreeEvidenceClearsMovedObstacle) {
  update({obstacle},1);
  for(int t=2;t<8;++t)update({far},t);
  EXPECT_FALSE(occupied(obstacle));EXPECT_TRUE(occupied(far));
}
TEST_F(FusionTest,CurrentHitWinsOverSameFrameFreeEvidence) {
  update({obstacle},1);
  for(int t=2;t<10;++t)update({obstacle,far},t);
  EXPECT_TRUE(occupied(obstacle));EXPECT_TRUE(occupied(far));EXPECT_EQ(count[id(obstacle)],0);
}
TEST_F(FusionTest,DuplicateReturnsAreNotIndependentConfirmations) {
  CloudFusion::Points many(1000,obstacle);
  ASSERT_TRUE(f->update(many,sensor,sensor,1));EXPECT_EQ(f->stats().hit_cells,1u);
  update({far},2);EXPECT_EQ(count[id(obstacle)],1);
}
TEST_F(FusionTest,HitResetsFreeConfirmationCount) {
  update({obstacle},1);update({far},2);update({obstacle},3);update({far},4);
  EXPECT_EQ(count[id(obstacle)],1);EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,EndpointUncertaintyBandIsNotCleared) {
  update({obstacle},1);
  for(int t=2;t<10;++t)update({obstacle+Eigen::Vector3d(.15,0,0)},t);
  EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,RayBudgetNeverDropsNewHitCells) {
  o.max_free_rays=1;rebuild();CloudFusion::Points v;
  for(int i=0;i<30;++i)v.emplace_back(1.025,-.725+i*.05,1.025);
  ASSERT_TRUE(f->update(v,sensor,sensor,1));EXPECT_EQ(f->stats().rays,1u);EXPECT_EQ(f->stats().hit_cells,30u);
  for(auto&p:v)EXPECT_TRUE(occupied(p));
}
TEST_F(FusionTest,MotionSuspendsClearingButKeepsNewHits) {
  update({obstacle},1);
  for(int t=2;t<10;++t)update({far},t,false);
  EXPECT_TRUE(occupied(obstacle));EXPECT_TRUE(occupied(far));EXPECT_EQ(f->stats().rays,0u);
  for(int t=10;t<17;++t)update({far},t,true);
  EXPECT_FALSE(occupied(obstacle));
}
TEST_F(FusionTest,OnlyReliablyTraversedInteriorCanClear) {
  update({obstacle},1);
  // This ray is only 1mm from a voxel face, inside the protected interior margin.
  Eigen::Vector3d shifted_sensor=sensor+Eigen::Vector3d(0,.024,0),end=far+Eigen::Vector3d(0,.024,0);
  for(int t=2;t<10;++t)f->update(points({end}),shifted_sensor,shifted_sensor,t);
  EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,NonVoxelAlignedOriginUsesCorrectCoordinates) {
  o.origin+=Eigen::Vector3d(.013,.007,.001);rebuild();
  sensor=o.origin+Eigen::Vector3d(80.5,80.5,20.5)*o.resolution;
  obstacle=sensor+Eigen::Vector3d(1,0,0);far=sensor+Eigen::Vector3d(2,0,0);
  update({obstacle},1);for(int t=2;t<8;++t)update({far},t);
  EXPECT_FALSE(occupied(obstacle));EXPECT_TRUE(occupied(far));
}
TEST_F(FusionTest,DiagonalThreeDimensionalRayClearsCorrectCell) {
  obstacle=sensor+Eigen::Vector3d(.5,.5,.5);far=sensor+Eigen::Vector3d(1.,1.,1.);
  update({obstacle},1);for(int t=2;t<8;++t)update({far},t);
  EXPECT_FALSE(occupied(obstacle));EXPECT_TRUE(occupied(far));
}
TEST_F(FusionTest,MovingLocalWindowPrunesStaleInflation) {
  o.local_range=Eigen::Vector3d(1.2,1.2,1.2);rebuild();update({obstacle},1);
  Eigen::Vector3d moved(-2.025,.025,1.025),end=moved+Eigen::Vector3d(.6,0,0);
  ASSERT_TRUE(f->update(points({end}),moved,moved,2));EXPECT_FALSE(occupied(obstacle));EXPECT_GT(f->stats().pruned,0u);
}
TEST_F(FusionTest,ResetDoesNotLeaveGhostInflationOrCounters) {
  update({obstacle},1);update({far},2);f->reset();
  EXPECT_FALSE(occupied(obstacle));EXPECT_FALSE(occupied(far));EXPECT_EQ(f->stats().active,0u);
  ASSERT_TRUE(update({obstacle},1));EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,NoEpochWrapOrCapacityGrowthOnLongRun) {
  update({obstacle},1);size_t capacity=f->stats().work_capacity_bytes;
  for(int t=2;t<600;++t)ASSERT_TRUE(update({obstacle},t));
  EXPECT_TRUE(occupied(obstacle));EXPECT_EQ(f->stats().active,1u);
  EXPECT_EQ(f->stats().work_capacity_bytes,capacity);
}
TEST_F(FusionTest,MalformedInputIsRejectedBeforeMutatingMap) {
  update({obstacle},1);
  EXPECT_FALSE(f->update(points({far}),Eigen::Vector3d::Constant(100),sensor,2));
  EXPECT_FALSE(f->update(points({far}),sensor,Eigen::Vector3d::Constant(1e308),2));
  EXPECT_FALSE(update({far},std::numeric_limits<double>::quiet_NaN()));
  CloudFusion::Points excessive(200001,far);EXPECT_FALSE(f->update(excessive,sensor,sensor,2));
  EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,InvalidConfigurationCannotSilentlyRun) {
  o.angular_resolution=0.;EXPECT_THROW(rebuild(),std::invalid_argument);
  o.angular_resolution=.5;o.blind=std::numeric_limits<double>::quiet_NaN();EXPECT_THROW(rebuild(),std::invalid_argument);
}

TEST_F(FusionTest,SensorOutsideGridDoesNotDropValidBodyInsideCloud) {
  Eigen::Vector3d body(.025,.025,2.4),lidar(.025,.025,2.55),old(.025,.025,1.925),behind(.025,.025,.925);
  ASSERT_TRUE(f->update(points({old}),lidar,body,1));EXPECT_TRUE(occupied(old));
  for(int t=2;t<9;++t)ASSERT_TRUE(f->update(points({behind}),lidar,body,t));
  EXPECT_FALSE(occupied(old));EXPECT_TRUE(occupied(behind));
}

TEST_F(FusionTest,BoundedFastMotionCanClearReliableInterior) {
  update({obstacle},1);
  // 0.56m/s over 0.1s: +/-2.8cm transverse origin extent, plus model error.
  Eigen::Vector3d extent(.010,.038,.010);
  for(int t=2;t<9;++t)ASSERT_TRUE(f->update(points({far}),sensor,sensor,t,true,extent));
  EXPECT_FALSE(occupied(obstacle));EXPECT_TRUE(occupied(far));
  EXPECT_GT(f->stats().origin_uncertainty,0.);
}
TEST_F(FusionTest,OriginEnvelopeThatCouldMissVoxelCannotClear) {
  update({obstacle},1);
  for(int t=2;t<12;++t)f->update(points({far}),sensor,sensor,t,true,Eigen::Vector3d(0,.06,0));
  EXPECT_TRUE(occupied(obstacle));EXPECT_GT(f->stats().uncertain_cells,0u);
}
TEST_F(FusionTest,LongitudinalMotionStillClearsBeyondBlindZone) {
  update({obstacle},1);
  for(int t=2;t<9;++t)f->update(points({far}),sensor,sensor,t,true,Eigen::Vector3d(.038,.01,.01));
  EXPECT_FALSE(occupied(obstacle));
}
TEST_F(FusionTest,UncertainOriginsExpandBlindProtection) {
  update({obstacle},1);
  Eigen::Vector3d near=obstacle-Eigen::Vector3d(.52,0,0);
  for(int t=2;t<12;++t)f->update(points({far}),near,near,t,true,Eigen::Vector3d(.038,.01,.01));
  EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,CurrentHitWinsWithMovingOriginEnvelope) {
  update({obstacle},1);
  for(int t=2;t<10;++t)f->update(points({obstacle,far}),sensor,sensor,t,true,Eigen::Vector3d(.01,.038,.01));
  EXPECT_TRUE(occupied(obstacle));EXPECT_EQ(count[id(obstacle)],0);
}
TEST_F(FusionTest,InvalidEnvelopeRejectedWithoutChangingHistory) {
  update({obstacle},1);
  EXPECT_FALSE(f->update(points({far}),sensor,sensor,2,true,Eigen::Vector3d(-.01,0,0)));
  EXPECT_FALSE(f->update(points({far}),sensor,sensor,2,true,Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())));
  EXPECT_TRUE(occupied(obstacle));
}
TEST_F(FusionTest,ClippedRayDoesNotUnderestimateOriginUncertainty) {
  // Nominal origin outside map; uncertainty must contract towards measured endpoint,
  // not towards the artificial map-entry or max-range clipping point.
  Eigen::Vector3d body(.025,.025,2.4),lidar(.025,.025,2.55);
  Eigen::Vector3d old(.025,.025,1.925),behind(.025,.025,.925);
  f->update(points({old}),lidar,body,1);
  for(int t=2;t<12;++t)f->update(points({behind}),lidar,body,t,true,Eigen::Vector3d(.06,0,0));
  EXPECT_TRUE(occupied(old));
}
TEST_F(FusionTest,MaxRangeClippingDoesNotShrinkUncertaintyTooEarly) {
  o.max_ray_length=1.5;rebuild();update({obstacle},1);
  for(int t=2;t<12;++t)f->update(points({far}),sensor,sensor,t,true,Eigen::Vector3d(0,.06,0));
  EXPECT_TRUE(occupied(obstacle));
}

int main(int argc,char**argv) {
  ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
