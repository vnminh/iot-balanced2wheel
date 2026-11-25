#include "slam/planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <queue>

namespace slam {

bool nearestFree(const TraversedGrid &g, const Eigen::Vector2d &p, double max_dist, int &ci, int &cj) {
  if (g.empty()) return false;
  int i0, j0;
  g.toCell(p, i0, j0);
  const int R = int(std::ceil(max_dist / g.res)) + 1;
  double best = std::numeric_limits<double>::infinity();
  for (int dj = -R; dj <= R; dj++)
    for (int di = -R; di <= R; di++) {
      const int i = i0 + di, j = j0 + dj;
      if (!g.free(i, j)) continue;
      const double d = (g.cellCenter(i, j) - p).norm();
      if (d < best) best = d, ci = i, cj = j;
    }
  return best <= max_dist + g.res;
}

PlanResult planPath(const TraversedGrid &g, const Eigen::Vector2d &start, const Eigen::Vector2d &goal,
                    const PlannerCfg &cfg, double center_tol) {
  PlanResult r;
  if (g.empty() || g.freeCells() == 0) {
    r.why = "Chưa có bản đồ vùng đã đi";
    return r;
  }
  int si, sj, gi, gj;
  if (!nearestFree(g, start, cfg.max_snap, si, sj)) {
    r.why = "Xe đang ở ngoài vùng đã đi";
    return r;
  }
  if (!nearestFree(g, goal, cfg.max_snap, gi, gj)) {
    r.why = "Đích nằm ngoài vùng xe đã đi qua (xa hơn " + std::to_string(cfg.max_snap).substr(0, 4) + " m)";
    return r;
  }
  r.start_snap = (g.cellCenter(si, sj) - start).norm();
  r.goal = g.cellCenter(gi, gj);
  r.goal_snap = (r.goal - goal).norm();

  // Phạt gần mép: khoảng cách tới ô chặn gần nhất (distance transform), mong muốn ≥ center_tol + 1 ô
  cv::Mat dist;
  cv::Mat padded;
  cv::copyMakeBorder(g.center, padded, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);
  cv::distanceTransform(padded, dist, cv::DIST_L2, 3);
  const double pref = center_tol + g.res;
  auto penalty = [&](int i, int j) {
    const double d = dist.at<float>(j + 1, i + 1) * g.res;
    return std::max(0.0, 1.0 - d / pref);
  };

  const int N = g.nx * g.ny;
  std::vector<double> cost(N, std::numeric_limits<double>::infinity());
  std::vector<int> parent(N, -1);
  std::vector<char> closed(N, 0);
  using QE = std::pair<double, int>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;
  auto idx = [&](int i, int j) { return j * g.nx + i; };
  auto h = [&](int i, int j) { return std::hypot(double(i - gi), double(j - gj)) * g.res; };
  cost[idx(si, sj)] = 0;
  open.push({h(si, sj), idx(si, sj)});
  static const int DI[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DJ[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  bool found = false;
  while (!open.empty()) {
    const int cur = open.top().second;
    open.pop();
    if (closed[cur]) continue;
    closed[cur] = 1;
    const int ci = cur % g.nx, cj = cur / g.nx;
    if (ci == gi && cj == gj) {
      found = true;
      break;
    }
    for (int k = 0; k < 8; k++) {
      const int ni = ci + DI[k], nj = cj + DJ[k];
      if (!g.free(ni, nj)) continue;
      if (k >= 4 && (!g.free(ci + DI[k], cj) || !g.free(ci, cj + DJ[k]))) continue;  // không cắt góc
      const int n = idx(ni, nj);
      if (closed[n]) continue;
      const double step = (k < 4 ? 1.0 : M_SQRT2) * g.res * (1.0 + cfg.edge_weight * penalty(ni, nj));
      if (cost[cur] + step < cost[n]) {
        cost[n] = cost[cur] + step;
        parent[n] = cur;
        open.push({cost[n] + h(ni, nj), n});
      }
    }
  }
  if (!found) {
    r.why = "Không có đường nối trong vùng đã đi (hai vùng tách rời)";
    return r;
  }
  for (int c = idx(gi, gj); c >= 0; c = parent[c]) r.path.push_back(g.cellCenter(c % g.nx, c / g.nx));
  std::reverse(r.path.begin(), r.path.end());

  // Làm thưa thành waypoint cách nhau ~waypoint_spacing
  double acc = 0;
  for (size_t k = 1; k < r.path.size(); k++) {
    const double d = (r.path[k] - r.path[k - 1]).norm();
    r.length += d;
    acc += d;
    if (acc >= cfg.waypoint_spacing) {
      r.waypoints.push_back(r.path[k]);
      acc = 0;
    }
  }
  if (r.waypoints.empty() || (r.waypoints.back() - r.path.back()).norm() > 1e-9) r.waypoints.push_back(r.path.back());
  r.ok = true;
  return r;
}

}  // namespace slam
