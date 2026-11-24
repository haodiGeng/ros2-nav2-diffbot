#include "my_planner/astar_core.hpp"
#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>

namespace my_planner {
namespace {
constexpr double INF=std::numeric_limits<double>::infinity();
bool valid_grid(const Grid &g) {
  return g.width>0 && g.height>0 && std::isfinite(g.resolution) && g.resolution>0 &&
    static_cast<int64_t>(g.width)*g.height<=std::numeric_limits<int>::max() &&
    static_cast<size_t>(g.width)*g.height==g.costs.size();
}
// Exact squared Euclidean distance transform of one dimension. Skip infinite sites.
std::vector<double> edt1(const std::vector<double>&f) {
  const int n=static_cast<int>(f.size());
  std::vector<int> v(n);std::vector<double> z(n+1),out(n,INF);
  int k=-1;
  for(int q=0;q<n;++q) {
    if(!std::isfinite(f[q]))continue;
    if(k<0){k=0;v[0]=q;z[0]=-INF;z[1]=INF;continue;}
    auto intersection=[&](){const int p=v[k];return ((f[q]+double(q)*q)-(f[p]+double(p)*p))/(2.0*(q-p));};
    double s=intersection();
    while(k>0 && s<=z[k]){--k;s=intersection();}
    ++k;v[k]=q;z[k]=s;z[k+1]=INF;
  }
  if(k<0)return out;
  int a=0;
  for(int q=0;q<n;++q){while(a<k && z[a+1]<q)++a;const double d=q-v[a];out[q]=d*d+f[v[a]];}
  return out;
}
}  // namespace

Result search(const Grid &g,int start,int goal,const Options &o,
              const std::function<bool()>&cancel) {
  const auto began=std::chrono::steady_clock::now();
  Result r;
  auto finish=[&](Status status,const std::string &reason) {
    r.status=status;r.reason=reason;
    r.elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    return r;
  };
  auto interrupt=[&](){
    if(cancel && cancel()){r.status=Status::Cancelled;r.reason="cancelled";return true;}
    if(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()>=o.max_time){
      r.status=Status::Timeout;r.reason="search time budget exceeded";return true;}
    return false;
  };
  if(!valid_grid(g))return finish(Status::Invalid,"invalid grid");
  if(!std::isfinite(o.cost_weight)||o.cost_weight<0 || !std::isfinite(o.max_time)||o.max_time<=0 ||
     !std::isfinite(o.robot_radius)||o.robot_radius<0 || !std::isfinite(o.min_clearance)||o.min_clearance<0 ||
     o.unknown_cost<0||o.unknown_cost>252)return finish(Status::Invalid,"invalid options");
  const int count=static_cast<int>(g.costs.size());
  if(start<0||goal<0||start>=count||goal>=count)return finish(Status::Invalid,"endpoint outside grid");
  if(interrupt())return finish(r.status,r.reason);
  r.traversable.resize(count);
  for(int i=0;i<count;++i)
    r.traversable[i]=g.costs[i]==255 ? o.allow_unknown : g.costs[i]<253;
  if(o.min_clearance>0) {
    std::vector<double> distances(count,INF);
    for(int y=0;y<g.height;++y) {
      if(interrupt())return finish(r.status,r.reason);
      std::vector<double> f(g.width,INF);
      for(int x=0;x<g.width;++x) {
        const auto c=g.costs[y*g.width+x];
        if(c==254 || (c==255&&!o.allow_unknown))f[x]=0.;
      }
      const auto d=edt1(f);
      std::copy(d.begin(),d.end(),distances.begin()+y*g.width);
    }
    for(int x=0;x<g.width;++x) {
      if(interrupt())return finish(r.status,r.reason);
      std::vector<double> f(g.height);
      for(int y=0;y<g.height;++y)f[y]=distances[y*g.width+x];
      const auto d=edt1(f);
      for(int y=0;y<g.height;++y)distances[y*g.width+x]=d[y];
    }
    const double required=o.robot_radius+o.min_clearance;
    for(int y=0;y<g.height;++y) {
      if(interrupt())return finish(r.status,r.reason);
      for(int x=0;x<g.width;++x) {
        const int i=y*g.width+x;
        // Bound clearance for ANY point in the whole cell to occupied cell area.
        // Two half-diagonals are subtracted: obstacle cell and traversed cell.
        const double obstacle_bound=std::sqrt(distances[i])*g.resolution-std::sqrt(2.)*g.resolution;
        const double border_bound=std::min({x,y,g.width-1-x,g.height-1-y})*g.resolution;
        if(obstacle_bound+1e-12<required || border_bound+1e-12<required)r.traversable[i]=0;
      }
    }
  }
  if(!r.traversable[start])return finish(Status::Invalid,"start blocked by cost or clearance");
  if(!r.traversable[goal])return finish(Status::Invalid,"goal blocked by cost or clearance");
  struct Entry {double f,g;int id;};
  struct Later {bool operator()(const Entry&a,const Entry&b)const {
    if(a.f!=b.f)return a.f>b.f;
    if(a.g!=b.g)return a.g<b.g;
    return a.id>b.id;
  }};
  std::priority_queue<Entry,std::vector<Entry>,Later> open;
  std::vector<double> best(count,INF);std::vector<int> parent(count,-1);
  std::vector<uint8_t> closed(count,0);
  const int gx=goal%g.width,gy=goal/g.width;
  auto heuristic=[&](int i){
    const double dx=std::abs(i%g.width-gx),dy=std::abs(i/g.width-gy);
    return g.resolution*(o.eight_connected ? std::max(dx,dy)+(std::sqrt(2.)-1.)*std::min(dx,dy) : dx+dy);
  };
  auto cost=[&](int i){return (g.costs[i]==255?o.unknown_cost:g.costs[i])/252.0;};
  auto free=[&](int x,int y){return g.inside(x,y)&&r.traversable[y*g.width+x];};
  best[start]=0.;open.push({heuristic(start),0.,start});
  const int dxs[8]={1,-1,0,0,1,1,-1,-1};
  const int dys[8]={0,0,1,-1,1,-1,1,-1};
  size_t iterations=0;
  while(!open.empty()) {
    if((iterations++&127)==0 && interrupt())return finish(r.status,r.reason);
    const auto cur=open.top();open.pop();
    if(closed[cur.id] || cur.g>best[cur.id])continue;
    closed[cur.id]=1;++r.expanded;
    if(cur.id==goal) {
      for(int i=goal;i!=-1;i=parent[i])r.cells.push_back(i);
      std::reverse(r.cells.begin(),r.cells.end());r.cost=best[goal];
      return finish(Status::Success,"success");
    }
    const int x=cur.id%g.width,y=cur.id/g.width;
    for(int k=0;k<(o.eight_connected?8:4);++k) {
      const int dx=dxs[k],dy=dys[k],nx=x+dx,ny=y+dy;
      if(!free(nx,ny))continue;
      // Never squeeze diagonally between blocked / clearance-excluded cells.
      if(dx&&dy&&(!free(x+dx,y)||!free(x,y+dy)))continue;
      const int next=ny*g.width+nx;
      if(closed[next])continue;
      const double length=g.resolution*(dx&&dy?std::sqrt(2.):1.);
      const double candidate=best[cur.id]+length*(1.+o.cost_weight*.5*(cost(cur.id)+cost(next)));
      if(candidate<best[next]) {
        best[next]=candidate;parent[next]=cur.id;open.push({candidate+heuristic(next),candidate,next});
      }
    }
  }
  return finish(Status::NoPath,"no path");
}

bool segment_safe(const Grid &g,const std::vector<uint8_t>&mask,double ax,double ay,double bx,double by) {
  if(!valid_grid(g)||mask.size()!=g.costs.size()||!std::isfinite(ax)||!std::isfinite(ay)||
     !std::isfinite(bx)||!std::isfinite(by))return false;
  if(ax<0||ay<0||bx<0||by<0||ax>=g.width||bx>=g.width||ay>=g.height||by>=g.height)return false;
  auto free=[&](int x,int y){return g.inside(x,y)&&mask[y*g.width+x];};
  auto point_safe=[&](double px,double py){
    const int cx=static_cast<int>(std::floor(px)),cy=static_cast<int>(std::floor(py));
    const bool vx=std::abs(px-std::round(px))<1e-12,hy=std::abs(py-std::round(py))<1e-12;
    return free(cx,cy)&&(!vx||free(cx-1,cy))&&(!hy||free(cx,cy-1))&&(!(vx&&hy)||free(cx-1,cy-1));
  };
  if(!point_safe(ax,ay)||!point_safe(bx,by))return false;
  int x=static_cast<int>(std::floor(ax)),y=static_cast<int>(std::floor(ay));
  const int ex=static_cast<int>(std::floor(bx)),ey=static_cast<int>(std::floor(by));
  const double dx=bx-ax,dy=by-ay;
  const int sx=(dx>0)-(dx<0),sy=(dy>0)-(dy<0);
  const bool on_vertical=sx==0&&std::abs(ax-std::round(ax))<1e-12;
  const bool on_horizontal=sy==0&&std::abs(ay-std::round(ay))<1e-12;
  auto safe_cell=[&](){return free(x,y)&&(!on_vertical||free(x-1,y))&&(!on_horizontal||free(x,y-1));};
  double tx=sx?((sx>0?x+1.-ax:ax-x)/std::abs(dx)):INF;
  double ty=sy?((sy>0?y+1.-ay:ay-y)/std::abs(dy)):INF;
  const double stepx=sx?1./std::abs(dx):INF,stepy=sy?1./std::abs(dy):INF;
  for(int64_t steps=0;steps<=static_cast<int64_t>(g.width)+g.height+2;++steps) {
    if(!safe_cell())return false;
    if(x==ex&&y==ey)return true;
    if(std::abs(tx-ty)<1e-12) {
      if(!free(x+sx,y)||!free(x,y+sy))return false;
      x+=sx;y+=sy;tx+=stepx;ty+=stepy;
    } else if(tx<ty){x+=sx;tx+=stepx;}else{y+=sy;ty+=stepy;}
  }
  return false;
}
}  // namespace my_planner
