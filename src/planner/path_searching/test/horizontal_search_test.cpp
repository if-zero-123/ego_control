#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>

class HeightSearchTest : public ::testing::Test {
protected:
  GridMap::Ptr map;
  std::unique_ptr<AStar> search;
  Eigen::Vector3d start{-1.,0.,1.25}, goal{1.,0.,1.25};
  void SetUp() override {
    ros::NodeHandle nh("~");
    nh.setParam("grid_map/cloud_fusion",false);
    nh.setParam("grid_map/resolution",.05);
    nh.setParam("grid_map/map_size_x",6.);
    nh.setParam("grid_map/map_size_y",6.);
    nh.setParam("grid_map/map_size_z",2.5);
    nh.setParam("grid_map/ground_height",-.5);
    nh.setParam("path_searching/prefer_horizontal",true);
    map.reset(new GridMap);map->initMap(nh);
    search.reset(new AStar);search->initGridMap(map,Eigen::Vector3i(60,60,40));
    search->setReferenceHeight(1.25);
  }
  void box(double xmin,double xmax,double ymin,double ymax,double zmin,double zmax) {
    for(double x=xmin;x<=xmax+1e-8;x+=.05)
      for(double y=ymin;y<=ymax+1e-8;y+=.05)
        for(double z=zmin;z<=zmax+1e-8;z+=.05)map->setOccupied(Eigen::Vector3d(x,y,z));
  }
  void clearEdges() {
    auto path=search->getPath();ASSERT_FALSE(path.empty());
    for(size_t i=1;i<path.size();++i)
      for(int j=0;j<=20;++j)
        ASSERT_EQ(map->getInflateOccupancy(path[i-1]+(path[i]-path[i-1])*(j/20.)),0);
  }
};
TEST_F(HeightSearchTest,OpenRouteStaysAtTargetHeight) {
  ASSERT_TRUE(search->AstarSearch(.1,start,goal));EXPECT_EQ(search->lastSearchStage(),1);
  for(auto p:search->getPath())EXPECT_NEAR(p.z(),1.25,1e-8);
  clearEdges();
}
TEST_F(HeightSearchTest,PillarUsesHorizontalDetourInsteadOfUnderOrOver) {
  box(-.175,.175,-.725,.725,.875,1.575);
  ASSERT_TRUE(search->AstarSearch(.1,start,goal));EXPECT_EQ(search->lastSearchStage(),1);
  double lateral=0.;
  for(auto p:search->getPath()) {EXPECT_LE(std::abs(p.z()-1.25),.15+1e-8);lateral=std::max(lateral,std::abs(p.y()));}
  EXPECT_GT(lateral,.7);clearEdges();
}
TEST_F(HeightSearchTest,BlockedHeightBandFallsBackToThreeDimensions) {
  box(-.175,.175,-2.975,2.975,.925,1.575);
  ASSERT_TRUE(search->AstarSearch(.1,start,goal));EXPECT_EQ(search->lastSearchStage(),2);
  double dz=0.;for(auto p:search->getPath())dz=std::max(dz,std::abs(p.z()-1.25));
  EXPECT_GT(dz,.15);clearEdges();
}
TEST_F(HeightSearchTest,HorizontalPreferenceChangesTheLegacyVerticalShortcut) {
  box(-.175,.175,-.725,.725,.875,1.575);
  ASSERT_TRUE(search->AstarSearch(.1,start,goal));EXPECT_EQ(search->lastSearchStage(),1);
  ros::NodeHandle nh("~");nh.setParam("path_searching/prefer_horizontal",false);
  AStar legacy;legacy.initGridMap(map,Eigen::Vector3i(60,60,40));legacy.setReferenceHeight(1.25);
  ASSERT_TRUE(legacy.AstarSearch(.1,start,goal));
  double dz=0.;for(auto p:legacy.getPath())dz=std::max(dz,std::abs(p.z()-1.25));
  EXPECT_GT(dz,.15);
}
TEST_F(HeightSearchTest,BlockedMapFailsWithinBudgetAndReturnsNoStalePath) {
  ASSERT_TRUE(search->AstarSearch(.1,start,goal));
  box(-.175,.175,-2.975,2.975,-.475,1.975);
  auto began=std::chrono::steady_clock::now();
  EXPECT_FALSE(search->AstarSearch(.1,start,goal));EXPECT_TRUE(search->getPath().empty());
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),.7);
}
TEST_F(HeightSearchTest,ReplanningRetainsRealEndpointsWithDifferentHeights) {
  start.z()=1.05;goal.z()=1.45;search->setReferenceHeight(goal.z());
  ASSERT_TRUE(search->AstarSearch(.1,start,goal));auto path=search->getPath();
  EXPECT_NEAR(path.front().z(),start.z(),1e-8);EXPECT_NEAR(path.back().z(),goal.z(),1e-8);clearEdges();
}
TEST_F(HeightSearchTest,ThinOccupiedCellsCannotBeSkippedByCoarseSearchSteps) {
  start.x()=-.525;goal.x()=.475;
  box(.025,.025,-2.975,2.975,-.475,1.975);
  EXPECT_FALSE(search->AstarSearch(.1,start,goal));EXPECT_TRUE(search->getPath().empty());
}
TEST_F(HeightSearchTest,RepeatedSearchUsesFreshOccupancyAndQueue) {
  for(int i=0;i<5;++i)ASSERT_TRUE(search->AstarSearch(.1,start,goal));
  box(-.175,.175,-.325,.325,.875,1.675);
  ASSERT_TRUE(search->AstarSearch(.1,start,goal));clearEdges();
}
TEST_F(HeightSearchTest,InvalidInputAndOccupiedCoincidentEndpointsFailCleanly) {
  EXPECT_FALSE(search->AstarSearch(0.,start,goal));
  box(-1.025,-.975,-.025,.025,1.225,1.275);
  EXPECT_FALSE(search->AstarSearch(.1,start,start));EXPECT_TRUE(search->getPath().empty());
}
int main(int argc,char**argv) {
  ros::init(argc,argv,"horizontal_search_test");
  ::testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
