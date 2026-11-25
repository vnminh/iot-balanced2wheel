#pragma once
// M9: A* 8 hướng trên center_layer. Chi phí = độ dài × (1 + edge_weight × phạt gần mép).
#include <Eigen/Core>
#include <string>
#include <vector>

#include "slam/config.h"
#include "slam/traversed_grid.h"

namespace slam {

struct PlanResult {
  bool ok = false;
  std::string why;
  std::vector<Eigen::Vector2d> path;       // dày (tâm ô)
  std::vector<Eigen::Vector2d> waypoints;  // thưa ~waypoint_spacing, điểm cuối = đích (đã chọn ô trống)
  double length = 0;
  Eigen::Vector2d goal{0, 0};              // đích thực dùng (có thể đã dời về ô trống gần nhất)
  double goal_snap = 0, start_snap = 0;    // m đã dời
};

PlanResult planPath(const TraversedGrid &g, const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
                    const PlannerCfg &cfg, double center_tol);

// Ô trống gần nhất trong bán kính max_dist (m). false nếu không có.
bool nearestFree(const TraversedGrid &g, const Eigen::Vector2d &p, double max_dist, int &ci, int &cj);

}  // namespace slam
