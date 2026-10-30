// Copyright 2026 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROSBOT_NAVIGATION_ROSBOT_NAVIGATION_MAP_ERASER_HPP
#define ROSBOT_NAVIGATION_ROSBOT_NAVIGATION_MAP_ERASER_HPP

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include <nav2_msgs/srv/clear_costmap_around_pose.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rosbot_navigation {

using EraseSrv = nav2_msgs::srv::ClearCostmapAroundPose;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

// World cells, keyed independently of the grid: slam_toolbox moves the
// map origin and grows the grid as it explores.
int64_t CellKey(int64_t ix, int64_t iy);

// What the live lidar says about each world cell, recently. Free evidence
// decays with `decay` (s); a cell counts as cleared once its decayed free
// evidence reaches `min_free` and no beam ended in it, or within
// `hit_radius` cells of it, for `hit_hold` (s). The neighbourhood is what
// keeps a real wall: slam draws it a few cells thick, and the live beams
// that end on its face pass through the smeared outer cells.
class LiveEvidence {
public:
  struct Params {
    double resolution = 0.05;
    double min_free = 20.0;
    double hit_hold = 30.0;
    double decay = 60.0;
    int hit_radius = 2;
  };

  explicit LiveEvidence(const Params &p) : p_(p) {}

  // Every cell from (x0, y0) up to (x1, y1) gets free evidence; with `hit`
  // the cell at (x1, y1) records a hit instead.
  void AddRay(double x0, double y0, double x1, double y1, bool hit, double t);
  bool Cleared(int64_t ix, int64_t iy, double now) const;
  void Prune(double now);
  void Clear() { cells_.clear(); }
  size_t Size() const { return cells_.size(); }
  const Params &params() const { return p_; }

private:
  struct Cell {
    double free = 0.0;
    double free_t = 0.0;
    double hit_t = -1e9;
  };
  void AddFree(int64_t ix, int64_t iy, double t);
  bool RecentHit(int64_t ix, int64_t iy, double now) const;

  Params p_;
  std::unordered_map<int64_t, Cell> cells_;
};

// Sits between slam_toolbox (`slam_map`) and everything that reads `map`.
// slam_toolbox re-renders its grid from every scan it ever kept, so a wall
// that is gone fades only after ~1/occupancy_threshold pass-throughs. A cell
// slam still calls occupied is published as unknown when it was erased by
// hand (until slam first calls it free; then it is released for good) or,
// with auto_clear, while the live lidar keeps seeing through it.
class MapEraserNode : public rclcpp::Node {
public:
  MapEraserNode(const std::string &node_name,
                const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  static OccupancyGrid Apply(const OccupancyGrid &map,
                             std::unordered_set<int64_t> &erased,
                             int8_t occupied_threshold,
                             const LiveEvidence *live = nullptr,
                             double now = 0.0, size_t *auto_cleared = nullptr);

private:
  void MapCB(const OccupancyGrid::SharedPtr msg);
  void ScanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void EraseCB(const std::shared_ptr<EraseSrv::Request> request,
               std::shared_ptr<EraseSrv::Response> response);
  void ResetCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void Publish();

  double max_radius_;
  int8_t occupied_threshold_;
  std::unordered_set<int64_t> erased_;
  OccupancyGrid::SharedPtr last_map_;

  bool auto_clear_;
  double clear_range_;
  double hit_margin_;
  double hit_margin_per_m_;
  int beam_step_;
  std::unique_ptr<LiveEvidence> live_;
  double last_scan_t_ = 0.0;
  double last_prune_t_ = 0.0;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr metadata_pub_;
  rclcpp::Service<EraseSrv>::SharedPtr erase_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

} // namespace rosbot_navigation

#endif // ROSBOT_NAVIGATION_ROSBOT_NAVIGATION_MAP_ERASER_HPP
