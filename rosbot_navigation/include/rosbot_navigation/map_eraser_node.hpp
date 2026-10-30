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
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include <nav2_msgs/srv/clear_costmap_around_pose.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rosbot_navigation {

using EraseSrv = nav2_msgs::srv::ClearCostmapAroundPose;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

// Sits between slam_toolbox (`slam_map`) and everything that reads `map`.
// slam_toolbox re-renders its grid from every scan it ever kept, so a wall
// that is gone fades only after ~1/occupancy_threshold pass-throughs. An
// erased cell that slam still calls occupied is published as unknown; the
// first time slam calls it free, it is released for good, so a real obstacle
// that appears there later shows up again.
class MapEraserNode : public rclcpp::Node {
public:
  MapEraserNode(const std::string &node_name,
                const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  // World cells, keyed independently of the grid: slam_toolbox moves the
  // map origin and grows the grid as it explores.
  static int64_t CellKey(int64_t ix, int64_t iy);
  static OccupancyGrid Apply(const OccupancyGrid &map,
                             std::unordered_set<int64_t> &erased,
                             int8_t occupied_threshold);

private:
  void MapCB(const OccupancyGrid::SharedPtr msg);
  void EraseCB(const std::shared_ptr<EraseSrv::Request> request,
               std::shared_ptr<EraseSrv::Response> response);
  void ResetCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void Publish();

  double max_radius_;
  int8_t occupied_threshold_;
  std::unordered_set<int64_t> erased_;
  OccupancyGrid::SharedPtr last_map_;

  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr metadata_pub_;
  rclcpp::Service<EraseSrv>::SharedPtr erase_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

} // namespace rosbot_navigation

#endif // ROSBOT_NAVIGATION_ROSBOT_NAVIGATION_MAP_ERASER_HPP
