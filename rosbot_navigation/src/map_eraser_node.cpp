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

#include "rosbot_navigation/map_eraser_node.hpp"

#include <cmath>

namespace rosbot_navigation {

MapEraserNode::MapEraserNode(const std::string &node_name,
                             const rclcpp::NodeOptions &options)
    : Node(node_name, options) {
  max_radius_ = this->declare_parameter<double>("max_radius", 5.0);
  // nav2 map_saver's occupied_thresh is 0.65: the same line here.
  occupied_threshold_ = static_cast<int8_t>(
      this->declare_parameter<int>("occupied_threshold", 65));

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  map_pub_ = this->create_publisher<OccupancyGrid>("map", latched);
  metadata_pub_ = this->create_publisher<nav_msgs::msg::MapMetaData>(
      "map_metadata", latched);
  map_sub_ = this->create_subscription<OccupancyGrid>(
      "slam_map", latched,
      std::bind(&MapEraserNode::MapCB, this, std::placeholders::_1));

  erase_srv_ = this->create_service<EraseSrv>(
      "map_eraser/erase",
      std::bind(&MapEraserNode::EraseCB, this, std::placeholders::_1,
                std::placeholders::_2));
  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "map_eraser/reset",
      std::bind(&MapEraserNode::ResetCB, this, std::placeholders::_1,
                std::placeholders::_2));
}

int64_t MapEraserNode::CellKey(int64_t ix, int64_t iy) {
  return (ix << 32) ^ (iy & 0xffffffff);
}

OccupancyGrid MapEraserNode::Apply(const OccupancyGrid &map,
                                   std::unordered_set<int64_t> &erased,
                                   int8_t occupied_threshold) {
  OccupancyGrid out = map;
  if (erased.empty()) {
    return out;
  }
  const double res = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;
  const int64_t w = map.info.width;
  const int64_t h = map.info.height;

  for (int64_t gy = 0; gy < h; ++gy) {
    for (int64_t gx = 0; gx < w; ++gx) {
      const double cx = ox + (gx + 0.5) * res;
      const double cy = oy + (gy + 0.5) * res;
      const int64_t key = CellKey(static_cast<int64_t>(std::floor(cx / res)),
                                  static_cast<int64_t>(std::floor(cy / res)));
      auto it = erased.find(key);
      if (it == erased.end()) {
        continue;
      }
      int8_t &cell = out.data[gy * w + gx];
      if (cell == 0) {
        erased.erase(it);
      } else if (cell >= occupied_threshold) {
        cell = -1;
      }
    }
  }
  return out;
}

void MapEraserNode::MapCB(const OccupancyGrid::SharedPtr msg) {
  last_map_ = msg;
  Publish();
}

void MapEraserNode::Publish() {
  if (!last_map_) {
    return;
  }
  auto out = Apply(*last_map_, erased_, occupied_threshold_);
  metadata_pub_->publish(out.info);
  map_pub_->publish(std::move(out));
}

void MapEraserNode::EraseCB(const std::shared_ptr<EraseSrv::Request> request,
                            std::shared_ptr<EraseSrv::Response>) {
  if (!last_map_) {
    RCLCPP_WARN(get_logger(), "erase refused: no map from slam_map yet");
    return;
  }
  const auto &frame = request->pose.header.frame_id;
  if (!frame.empty() && frame != last_map_->header.frame_id) {
    RCLCPP_WARN_STREAM(get_logger(), "erase refused: pose frame '"
                                         << frame << "' is not the map frame '"
                                         << last_map_->header.frame_id << "'");
    return;
  }
  const double r = request->reset_distance;
  if (!(r > 0.0) || r > max_radius_) {
    RCLCPP_WARN_STREAM(get_logger(), "erase refused: radius "
                                         << r << " m outside (0, "
                                         << max_radius_ << "]");
    return;
  }

  const double res = last_map_->info.resolution;
  const double x = request->pose.pose.position.x;
  const double y = request->pose.pose.position.y;
  const int64_t ixmin = static_cast<int64_t>(std::floor((x - r) / res));
  const int64_t ixmax = static_cast<int64_t>(std::floor((x + r) / res));
  const int64_t iymin = static_cast<int64_t>(std::floor((y - r) / res));
  const int64_t iymax = static_cast<int64_t>(std::floor((y + r) / res));
  size_t added = 0;
  for (int64_t iy = iymin; iy <= iymax; ++iy) {
    for (int64_t ix = ixmin; ix <= ixmax; ++ix) {
      const double dx = (ix + 0.5) * res - x;
      const double dy = (iy + 0.5) * res - y;
      if (dx * dx + dy * dy <= r * r) {
        added += erased_.insert(CellKey(ix, iy)).second;
      }
    }
  }
  RCLCPP_INFO_STREAM(get_logger(), "erased r=" << r << " m at (" << x << ", "
                                               << y << "): " << added
                                               << " new cells, "
                                               << erased_.size() << " held");
  Publish();
}

void MapEraserNode::ResetCB(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  const size_t n = erased_.size();
  erased_.clear();
  Publish();
  response->success = true;
  response->message = "released " + std::to_string(n) + " erased cells";
}

} // namespace rosbot_navigation
