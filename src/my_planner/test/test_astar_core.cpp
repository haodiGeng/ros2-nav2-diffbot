#include "my_planner/astar_core.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <queue>
#include <random>
using namespace my_planner;

Grid blank(int w,int h,double res=1.){return Grid{w,h,res,std::vector<uint8_t>(w*h,0)};}
// Independent reference implementation for small-map objective / optimality checks.
double reference(const Grid&g,int start,int goal,bool eight,double weight) {
  using Q=std::pair<double,int>;
  std::priority_queue<Q,std::vector<Q>,std::greater<Q>> q;
  std::vector<double> d(g.costs.size(),INFINITY);d[start]=0.;q.push({0.,start});
  auto free=[&](int x,int y){return g.inside(x,y)&&g.costs[y*g.width+x]<253;};
  while(!q.empty()) {
    auto [cost,id]=q.top();q.pop();if(cost>d[id])continue;if(id==goal)return cost;
    int x=id%g.width,y=id/g.width;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
      if((!dx&&!dy)||(!eight&&dx&&dy)||!free(x+dx,y+dy))continue;
      if(dx&&dy&&(!free(x+dx,y)||!free(x,y+dy)))continue;
      int next=(y+dy)*g.width+x+dx;
      double candidate=cost+std::hypot(dx,dy)*g.resolution*(1.+weight*(g.costs[id]+g.costs[next])/504.);
      if(candidate<d[next]){d[next]=candidate;q.push({candidate,next});}
    }
  }
  return INFINITY;
}
TEST(Search, EightAndFourConnectivity) {
  auto g=blank(5,5,.1);Options o;o.cost_weight=0.;
  auto r=search(g,0,24,o);ASSERT_EQ(r.status,Status::Success);EXPECT_NEAR(r.cost,.4*std::sqrt(2.),1e-10);
  o.eight_connected=false;r=search(g,0,24,o);EXPECT_EQ(r.status,Status::Success);EXPECT_NEAR(r.cost,.8,1e-10);
}
TEST(Search, DiagonalPinchIsForbidden) {
  auto g=blank(2,2);g.costs[1]=254;g.costs[2]=253;
  auto r=search(g,0,3,Options{});EXPECT_EQ(r.status,Status::NoPath);EXPECT_TRUE(r.cells.empty());
  EXPECT_FALSE(segment_safe(g,r.traversable,.5,.5,1.5,1.5));
}
TEST(Search, BarrierAndBlockedEndpoints) {
  auto g=blank(7,5);for(int y=0;y<5;++y)g.costs[y*7+3]=254;
  EXPECT_EQ(search(g,15,19,Options{}).status,Status::NoPath);
  EXPECT_EQ(search(g,3,19,Options{}).status,Status::Invalid);
  EXPECT_EQ(search(g,15,3,Options{}).status,Status::Invalid);
}
TEST(Search, UnknownPolicy) {
  auto g=blank(5,1);g.costs={0,255,255,255,0};Options o;
  EXPECT_EQ(search(g,0,4,o).status,Status::NoPath);
  o.allow_unknown=true;auto r=search(g,0,4,o);EXPECT_EQ(r.status,Status::Success);
  EXPECT_NEAR(r.cost,4.+3.*o.cost_weight*o.unknown_cost/252.,1e-9);
}
TEST(Search, SameCellAndEndpointSegments) {
  auto g=blank(3,3);auto r=search(g,4,4,Options{});
  EXPECT_EQ(r.status,Status::Success);EXPECT_EQ(r.cells.size(),1u);EXPECT_DOUBLE_EQ(r.cost,0.);
  EXPECT_TRUE(segment_safe(g,r.traversable,1.1,1.2,1.8,1.9));
}
TEST(Search, WeightChangesRoute) {
  auto g=blank(7,5);for(int x=1;x<6;++x)g.costs[2*7+x]=200;
  Options o;o.cost_weight=0.;auto a=search(g,14,20,o);EXPECT_NEAR(a.cost,6.,1e-9);
  o.cost_weight=4.;auto b=search(g,14,20,o);ASSERT_EQ(b.status,Status::Success);
  EXPECT_TRUE(std::none_of(b.cells.begin(),b.cells.end(),[&](int i){return g.costs[i]==200;}));
  EXPECT_NEAR(b.cost,reference(g,14,20,true,4.),1e-9);
}
TEST(Search, RandomMapsMatchIndependentDijkstra) {
  std::mt19937 rng(20261123);
  for(int trial=0;trial<48;++trial) {
    auto g=blank(13,11,.05);
    for(auto&c:g.costs){unsigned v=rng()%10;c=v<2?254:(v<5?100:0);}g.costs.front()=0;g.costs.back()=0;
    for(bool eight:{false,true})for(double weight:{0.,2.5}) {
      Options o;o.eight_connected=eight;o.cost_weight=weight;
      auto r=search(g,0,142,o);double expected=reference(g,0,142,eight,weight);
      if(std::isfinite(expected)){ASSERT_EQ(r.status,Status::Success);EXPECT_NEAR(r.cost,expected,1e-9);}
      else EXPECT_EQ(r.status,Status::NoPath);
      for(size_t i=1;i<r.cells.size();++i) {
        int a=r.cells[i-1],b=r.cells[i];
        EXPECT_TRUE(segment_safe(g,r.traversable,a%13+.5,a/13+.5,b%13+.5,b/13+.5));
      }
    }
  }
}
TEST(Clearance, RejectsNarrowCorridor) {
  auto g=blank(31,31,.1);for(int x=0;x<31;++x){g.costs[13*31+x]=254;g.costs[17*31+x]=254;}
  Options o;EXPECT_EQ(search(g,15*31+5,15*31+25,o).status,Status::Success);
  o.robot_radius=.1;o.min_clearance=.05;
  EXPECT_NE(search(g,15*31+5,15*31+25,o).status,Status::Success);
}
TEST(Clearance, DoesNotTreatInflationAsPhysicalObstacleAgain) {
  auto g=blank(21,21,.1);g.costs[10*21+10]=254;g.costs[10*21+11]=253;
  Options o;o.robot_radius=.1;o.min_clearance=.1;
  auto r=search(g,3*21+3,17*21+17,o);ASSERT_EQ(r.status,Status::Success);
  EXPECT_TRUE(r.traversable[10*21+14]);EXPECT_FALSE(r.traversable[10*21+11]);
}
TEST(Clearance, DistanceTransformMatchesBruteForceBound) {
  std::mt19937 rng(91);
  for(int trial=0;trial<10;++trial) {
    auto g=blank(9,8,.2);for(auto&c:g.costs){auto v=rng()%10;c=v==0?254:(v==1?255:0);}
    for(bool allow:{false,true}){
      Options o;o.allow_unknown=allow;o.min_clearance=.1;
      auto r=search(g,10,61,o);ASSERT_EQ(r.traversable.size(),g.costs.size());
      for(int y=0;y<8;++y)for(int x=0;x<9;++x) {
        double d=INFINITY;
        for(int j=0;j<72;++j)if(g.costs[j]==254||(g.costs[j]==255&&!allow))d=std::min(d,std::hypot(double(x-j%9),double(y-j/9))*.2);
        bool free=(g.costs[y*9+x]==255?allow:g.costs[y*9+x]<253);
        bool expected=free&&d-std::sqrt(2.)*.2+1e-12>=.1&&std::min({x,y,8-x,7-y})*.2+1e-12>=.1;
        EXPECT_EQ(bool(r.traversable[y*9+x]),expected);
      }
    }
  }
}
TEST(Clearance, NoObstacleAndMapBorder) {
  auto g=blank(10,10,.1);Options o;o.min_clearance=.1;
  auto r=search(g,22,77,o);ASSERT_EQ(r.status,Status::Success);
  EXPECT_FALSE(r.traversable[0]);EXPECT_TRUE(r.traversable[44]);
}
TEST(Segments, LongLinesBoundariesAndNonFinite) {
  auto g=blank(5,5);std::vector<uint8_t> mask(25,1);mask[2*5+2]=0;
  EXPECT_FALSE(segment_safe(g,mask,.5,2.5,4.5,2.5));
  EXPECT_FALSE(segment_safe(g,mask,2.,2.5,2.,3.5));
  EXPECT_TRUE(segment_safe(g,mask,.5,.5,4.5,.5));
  EXPECT_FALSE(segment_safe(g,mask,0.,.5,1.5,.5));
  EXPECT_FALSE(segment_safe(g,mask,NAN,.5,1.,1.));
  EXPECT_FALSE(segment_safe(g,mask,1.,1.,5.,1.));
}
TEST(Search, TimeBudgetAndCancellationLeaveNoPartialPath) {
  auto g=blank(20,20);Options o;
  auto r=search(g,0,399,o,[]{return true;});EXPECT_EQ(r.status,Status::Cancelled);EXPECT_TRUE(r.cells.empty());
  o.max_time=1e-12;r=search(g,0,399,o);EXPECT_EQ(r.status,Status::Timeout);EXPECT_TRUE(r.cells.empty());
}
TEST(Search, InvalidInputs) {
  auto g=blank(2,2);Options o;
  EXPECT_EQ(search(g,-1,3,o).status,Status::Invalid);
  EXPECT_EQ(search(g,0,4,o).status,Status::Invalid);
  o.cost_weight=-1.;EXPECT_EQ(search(g,0,3,o).status,Status::Invalid);o.cost_weight=NAN;
  EXPECT_EQ(search(g,0,3,o).status,Status::Invalid);o=Options{};
  g.costs.pop_back();EXPECT_EQ(search(g,0,3,o).status,Status::Invalid);
  g=blank(2,2);g.resolution=0;EXPECT_EQ(search(g,0,3,o).status,Status::Invalid);
}
