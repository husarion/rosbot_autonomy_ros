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

#include <gtest/gtest.h>

#include "rosbot_navigation/map_eraser_node.hpp"

using rosbot_navigation::MapEraserNode;
using rosbot_navigation::OccupancyGrid;

namespace {

OccupancyGrid Grid(double ox, double oy, std::vector<int8_t> data, int w) {
  OccupancyGrid g;
  g.info.resolution = 1.0;
  g.info.width = w;
  g.info.height = data.size() / w;
  g.info.origin.position.x = ox;
  g.info.origin.position.y = oy;
  g.data = std::move(data);
  return g;
}

} // namespace

TEST(MapEraser, MasksOccupiedKeepsFreeAndUnknown) {
  std::unordered_set<int64_t> erased = {MapEraserNode::CellKey(0, 0),
                                        MapEraserNode::CellKey(1, 0),
                                        MapEraserNode::CellKey(2, 0)};
  auto out = MapEraserNode::Apply(Grid(0, 0, {100, -1, 0, 100}, 4), erased, 65);
  EXPECT_EQ(out.data, (std::vector<int8_t>{-1, -1, 0, 100}));
}

TEST(MapEraser, FreeCellIsReleasedForGood) {
  std::unordered_set<int64_t> erased = {MapEraserNode::CellKey(0, 0)};
  MapEraserNode::Apply(Grid(0, 0, {0}, 1), erased, 65);
  EXPECT_TRUE(erased.empty());
  auto out = MapEraserNode::Apply(Grid(0, 0, {100}, 1), erased, 65);
  EXPECT_EQ(out.data[0], 100);
}

TEST(MapEraser, FollowsCellsWhenOriginMoves) {
  std::unordered_set<int64_t> erased = {MapEraserNode::CellKey(3, 1)};
  // origin (2, 1): world cell (3, 1) is grid (1, 0)
  auto out = MapEraserNode::Apply(Grid(2, 1, {100, 100}, 2), erased, 65);
  EXPECT_EQ(out.data, (std::vector<int8_t>{100, -1}));
}
