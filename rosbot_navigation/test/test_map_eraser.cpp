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

using rosbot_navigation::CellKey;
using rosbot_navigation::LiveEvidence;
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
  std::unordered_set<int64_t> erased = {CellKey(0, 0), CellKey(1, 0),
                                        CellKey(2, 0)};
  auto out = MapEraserNode::Apply(Grid(0, 0, {100, -1, 0, 100}, 4), erased, 65);
  EXPECT_EQ(out.data, (std::vector<int8_t>{-1, -1, 0, 100}));
}

TEST(MapEraser, FreeCellIsReleasedForGood) {
  std::unordered_set<int64_t> erased = {CellKey(0, 0)};
  MapEraserNode::Apply(Grid(0, 0, {0}, 1), erased, 65);
  EXPECT_TRUE(erased.empty());
  auto out = MapEraserNode::Apply(Grid(0, 0, {100}, 1), erased, 65);
  EXPECT_EQ(out.data[0], 100);
}

TEST(MapEraser, FollowsCellsWhenOriginMoves) {
  std::unordered_set<int64_t> erased = {CellKey(3, 1)};
  // origin (2, 1): world cell (3, 1) is grid (1, 0)
  auto out = MapEraserNode::Apply(Grid(2, 1, {100, 100}, 2), erased, 65);
  EXPECT_EQ(out.data, (std::vector<int8_t>{100, -1}));
}

namespace {

LiveEvidence::Params UnitParams() {
  LiveEvidence::Params p;
  p.resolution = 1.0;
  p.min_free = 3.0;
  p.hit_hold = 10.0;
  p.decay = 1e9;
  p.hit_radius = 0;
  return p;
}

} // namespace

TEST(LiveEvidence, RayFreesEveryCellOnTheWay) {
  LiveEvidence e(UnitParams());
  for (int i = 0; i < 3; ++i) {
    e.AddRay(0.5, 0.5, 4.5, 0.5, false, 100.0);
  }
  for (int x = 0; x <= 4; ++x) {
    EXPECT_TRUE(e.Cleared(x, 0, 100.0)) << x;
  }
  EXPECT_FALSE(e.Cleared(5, 0, 100.0));
  EXPECT_FALSE(e.Cleared(2, 1, 100.0));
}

TEST(LiveEvidence, DiagonalRayVisitsConnectedCells) {
  LiveEvidence e(UnitParams());
  for (int i = 0; i < 3; ++i) {
    e.AddRay(0.5, 0.5, 3.5, 2.5, false, 100.0);
  }
  EXPECT_TRUE(e.Cleared(0, 0, 100.0));
  EXPECT_TRUE(e.Cleared(3, 2, 100.0));
  EXPECT_EQ(e.Size(), 6u);
}

TEST(LiveEvidence, RecentHitBlocksClearing) {
  LiveEvidence e(UnitParams());
  e.AddRay(2.5, 0.5, 2.5, 0.5, true, 100.0);
  for (int i = 0; i < 5; ++i) {
    e.AddRay(0.5, 0.5, 4.5, 0.5, false, 101.0);
  }
  EXPECT_FALSE(e.Cleared(2, 0, 105.0));
  EXPECT_TRUE(e.Cleared(2, 0, 111.0));
}

TEST(LiveEvidence, FreeEvidenceDecays) {
  auto p = UnitParams();
  p.decay = 10.0;
  LiveEvidence e(p);
  for (int i = 0; i < 4; ++i) {
    e.AddRay(0.5, 0.5, 1.5, 0.5, false, 100.0);
  }
  EXPECT_TRUE(e.Cleared(0, 0, 100.0));
  EXPECT_FALSE(e.Cleared(0, 0, 120.0));
}

TEST(MapEraser, AutoClearBlanksOccupiedSeenThrough) {
  LiveEvidence e(UnitParams());
  for (int i = 0; i < 3; ++i) {
    e.AddRay(0.5, 0.5, 3.5, 0.5, false, 100.0);
  }
  std::unordered_set<int64_t> erased;
  size_t n = 0;
  auto out = MapEraserNode::Apply(Grid(0, 0, {100, 0, 100, -1, 100}, 5), erased,
                                  65, &e, 100.0, &n);
  EXPECT_EQ(out.data, (std::vector<int8_t>{-1, 0, -1, -1, 100}));
  EXPECT_EQ(n, 2u);
}

TEST(LiveEvidence, HitNextDoorKeepsTheCell) {
  auto p = UnitParams();
  p.hit_radius = 1;
  LiveEvidence e(p);
  for (int i = 0; i < 5; ++i) {
    e.AddRay(0.5, 0.5, 2.5, 0.5, false, 100.0);
  }
  e.AddRay(3.5, 0.5, 3.5, 0.5, true, 100.0);
  EXPECT_TRUE(e.Cleared(1, 0, 100.0));
  EXPECT_FALSE(e.Cleared(2, 0, 100.0));
  EXPECT_TRUE(e.Cleared(2, 0, 111.0));
}
