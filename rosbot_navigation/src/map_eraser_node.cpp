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
#include <limits>

namespace rosbot_navigation {

int64_t CellKey(int64_t ix, int64_t iy) {
  return (ix << 32) ^ (iy & 0xffffffff);
}

void LiveEvidence::AddFree(int64_t ix, int64_t iy, double t) {
  Cell &c = cells_[CellKey(ix, iy)];
  c.free = c.free * std::exp(-(t - c.free_t) / p_.decay) + 1.0;
  c.free_t = t;
}

void LiveEvidence::AddRay(double x0, double y0, double x1, double y1, bool hit,
                          double t) {
  const double res = p_.resolution;
  int64_t ix = static_cast<int64_t>(std::floor(x0 / res));
  int64_t iy = static_cast<int64_t>(std::floor(y0 / res));
  const int64_t ex = static_cast<int64_t>(std::floor(x1 / res));
  const int64_t ey = static_cast<int64_t>(std::floor(y1 / res));

  // Amanatides & Woo grid traversal, in cell units.
  const double dx = (x1 - x0) / res;
  const double dy = (y1 - y0) / res;
  const int sx = dx > 0 ? 1 : -1;
  const int sy = dy > 0 ? 1 : -1;
  const double inf = std::numeric_limits<double>::infinity();
  const double tdx = dx != 0 ? std::abs(1.0 / dx) : inf;
  const double tdy = dy != 0 ? std::abs(1.0 / dy) : inf;
  const double fx = x0 / res - ix;
  const double fy = y0 / res - iy;
  double tmx = dx != 0 ? (sx > 0 ? 1.0 - fx : fx) * tdx : inf;
  double tmy = dy != 0 ? (sy > 0 ? 1.0 - fy : fy) * tdy : inf;

  while (ix != ex || iy != ey) {
    AddFree(ix, iy, t);
    if (tmx < tmy) {
      tmx += tdx;
      ix += sx;
    } else {
      tmy += tdy;
      iy += sy;
    }
    if (tmx > 1.0 && tmy > 1.0) {
      break;
    }
  }
  if (hit) {
    Cell &c = cells_[CellKey(ex, ey)];
    c.hit_t = t;
    c.free = 0.0;
    c.free_t = t;
  } else {
    AddFree(ex, ey, t);
  }
}

bool LiveEvidence::RecentHit(int64_t ix, int64_t iy, double now) const {
  for (int64_t dy = -p_.hit_radius; dy <= p_.hit_radius; ++dy) {
    for (int64_t dx = -p_.hit_radius; dx <= p_.hit_radius; ++dx) {
      auto it = cells_.find(CellKey(ix + dx, iy + dy));
      if (it != cells_.end() && now - it->second.hit_t < p_.hit_hold) {
        return true;
      }
    }
  }
  return false;
}

bool LiveEvidence::Cleared(int64_t ix, int64_t iy, double now) const {
  auto it = cells_.find(CellKey(ix, iy));
  if (it == cells_.end()) {
    return false;
  }
  const Cell &c = it->second;
  return c.free * std::exp(-(now - c.free_t) / p_.decay) >= p_.min_free &&
         !RecentHit(ix, iy, now);
}

void LiveEvidence::Prune(double now) {
  for (auto it = cells_.begin(); it != cells_.end();) {
    const Cell &c = it->second;
    const bool stale = c.free * std::exp(-(now - c.free_t) / p_.decay) < 0.5 &&
                       now - c.hit_t >= p_.hit_hold;
    it = stale ? cells_.erase(it) : std::next(it);
  }
}

MapEraserNode::MapEraserNode(const std::string &node_name,
                             const rclcpp::NodeOptions &options)
    : Node(node_name, options) {
  max_radius_ = this->declare_parameter<double>("max_radius", 5.0);
  // nav2 map_saver's occupied_thresh is 0.65: the same line here.
  occupied_threshold_ = static_cast<int8_t>(
      this->declare_parameter<int>("occupied_threshold", 65));

  auto_clear_ = this->declare_parameter<bool>("auto_clear.enabled", false);
  clear_range_ = this->declare_parameter<double>("auto_clear.range", 6.0);
  // A beam's last few centimetres are not free evidence: pose noise would
  // otherwise eat into the face of every real wall.
  hit_margin_ = this->declare_parameter<double>("auto_clear.hit_margin", 0.1);
  // Plus this share of the range: a few mrad of pose or timing error while
  // turning moves a 6 m return by ~10 cm.
  hit_margin_per_m_ =
      this->declare_parameter<double>("auto_clear.hit_margin_per_m", 0.03);
  beam_step_ = std::max<int64_t>(
      1, this->declare_parameter<int>("auto_clear.beam_step", 2));
  LiveEvidence::Params lp;
  lp.min_free = this->declare_parameter<double>("auto_clear.min_free", 20.0);
  lp.hit_hold = this->declare_parameter<double>("auto_clear.hit_hold", 30.0);
  lp.decay = this->declare_parameter<double>("auto_clear.decay", 60.0);
  lp.hit_radius = static_cast<int>(
      this->declare_parameter<int>("auto_clear.hit_radius_cells", 2));
  const auto scan_topic = this->declare_parameter<std::string>(
      "auto_clear.scan_topic", "scan_filtered");

  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  map_pub_ = this->create_publisher<OccupancyGrid>("map", latched);
  metadata_pub_ = this->create_publisher<nav_msgs::msg::MapMetaData>(
      "map_metadata", latched);
  map_sub_ = this->create_subscription<OccupancyGrid>(
      "slam_map", latched,
      std::bind(&MapEraserNode::MapCB, this, std::placeholders::_1));

  if (auto_clear_) {
    live_ = std::make_unique<LiveEvidence>(lp);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&MapEraserNode::ScanCB, this, std::placeholders::_1));
  }

  erase_srv_ = this->create_service<EraseSrv>(
      "map_eraser/erase",
      std::bind(&MapEraserNode::EraseCB, this, std::placeholders::_1,
                std::placeholders::_2));
  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "map_eraser/reset",
      std::bind(&MapEraserNode::ResetCB, this, std::placeholders::_1,
                std::placeholders::_2));
}

OccupancyGrid MapEraserNode::Apply(const OccupancyGrid &map,
                                   std::unordered_set<int64_t> &erased,
                                   int8_t occupied_threshold,
                                   const LiveEvidence *live, double now,
                                   size_t *auto_cleared) {
  OccupancyGrid out = map;
  if (erased.empty() && !live) {
    return out;
  }
  const double res = map.info.resolution;
  const double ox = map.info.origin.position.x;
  const double oy = map.info.origin.position.y;
  const int64_t w = map.info.width;
  const int64_t h = map.info.height;
  size_t cleared = 0;

  for (int64_t gy = 0; gy < h; ++gy) {
    for (int64_t gx = 0; gx < w; ++gx) {
      int8_t &cell = out.data[gy * w + gx];
      const double cx = ox + (gx + 0.5) * res;
      const double cy = oy + (gy + 0.5) * res;
      const int64_t ix = static_cast<int64_t>(std::floor(cx / res));
      const int64_t iy = static_cast<int64_t>(std::floor(cy / res));
      const int64_t key = CellKey(ix, iy);
      auto it = erased.find(key);
      if (it != erased.end()) {
        if (cell == 0) {
          erased.erase(it);
        } else if (cell >= occupied_threshold) {
          cell = -1;
        }
      } else if (live && cell >= occupied_threshold &&
                 live->Cleared(ix, iy, now)) {
        cell = -1;
        ++cleared;
      }
    }
  }
  if (auto_cleared) {
    *auto_cleared = cleared;
  }
  return out;
}

void MapEraserNode::MapCB(const OccupancyGrid::SharedPtr msg) {
  last_map_ = msg;
  Publish();
}

void MapEraserNode::ScanCB(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!last_map_) {
    return;
  }
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(last_map_->header.frame_id,
                                     msg->header.frame_id, msg->header.stamp,
                                     rclcpp::Duration::from_seconds(0.05));
  } catch (const tf2::TransformException &) {
    return;
  }
  if (live_->params().resolution != last_map_->info.resolution) {
    LiveEvidence::Params p = live_->params();
    p.resolution = last_map_->info.resolution;
    live_ = std::make_unique<LiveEvidence>(p);
  }
  const double t = rclcpp::Time(msg->header.stamp).seconds();
  const double x0 = tf.transform.translation.x;
  const double y0 = tf.transform.translation.y;
  const auto &q = tf.transform.rotation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double max_free = std::min<double>(clear_range_, msg->range_max);

  for (size_t i = 0; i < msg->ranges.size(); i += beam_step_) {
    const double r = msg->ranges[i];
    if (std::isnan(r) || r < msg->range_min) {
      continue;
    }
    const double a = yaw + msg->angle_min + i * msg->angle_increment;
    const double c = std::cos(a), s = std::sin(a);
    // No return within range: the beam crossed max_free of free space.
    const bool hit = std::isfinite(r) && r < msg->range_max && r <= max_free;
    const double free_len =
        hit ? r - hit_margin_ - hit_margin_per_m_ * r : max_free;
    if (free_len > 0.0) {
      live_->AddRay(x0, y0, x0 + free_len * c, y0 + free_len * s, false, t);
    }
    if (hit) {
      const double hx = x0 + r * c, hy = y0 + r * s;
      live_->AddRay(hx, hy, hx, hy, true, t);
    }
  }
  last_scan_t_ = t;
  if (t - last_prune_t_ > 10.0) {
    live_->Prune(t);
    last_prune_t_ = t;
  }
}

void MapEraserNode::Publish() {
  if (!last_map_) {
    return;
  }
  size_t cleared = 0;
  auto out = Apply(*last_map_, erased_, occupied_threshold_, live_.get(),
                   last_scan_t_, &cleared);
  if (cleared) {
    RCLCPP_DEBUG_STREAM(get_logger(),
                        "auto_clear: " << cleared << " occupied cells unknown");
  }
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
  if (live_) {
    live_->Clear();
  }
  Publish();
  response->success = true;
  response->message = "released " + std::to_string(n) + " erased cells";
}

} // namespace rosbot_navigation
