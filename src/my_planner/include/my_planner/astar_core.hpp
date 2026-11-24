#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace my_planner {
struct Grid {
  int width{0}, height{0};
  double resolution{1.0};
  std::vector<uint8_t> costs;
  bool inside(int x,int y) const {return x>=0 && y>=0 && x<width && y<height;}
};
struct Options {
  bool eight_connected{true}, allow_unknown{false};
  int unknown_cost{128};
  double cost_weight{2.0}, robot_radius{0.0}, min_clearance{0.0}, max_time{1.0};
};
enum class Status {Success, NoPath, Invalid, Timeout, Cancelled};
struct Result {
  Status status{Status::Invalid};
  std::string reason;
  std::vector<int> cells;
  std::vector<uint8_t> traversable;
  size_t expanded{0};
  double cost{std::numeric_limits<double>::infinity()}, elapsed_ms{0.0};
};
Result search(const Grid &grid,int start,int goal,const Options &options,
              const std::function<bool()> &cancel = {});
// Coordinates are in grid units. Supercover includes cells touched at corners.
bool segment_safe(const Grid &grid,const std::vector<uint8_t>&mask,
                  double ax,double ay,double bx,double by);
}  // namespace my_planner
