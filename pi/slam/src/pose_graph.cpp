#include "slam/pose_graph.h"

#include <ceres/autodiff_manifold.h>
#include <ceres/ceres.h>

#include <Eigen/Cholesky>
#include <array>
#include <chrono>

namespace slam {

namespace {

template <typename T>
T normalizeAngle(const T &a) {
  const T two_pi(2.0 * M_PI);
  return a - two_pi * ceres::floor((a + T(M_PI)) / two_pi);
}

struct Pose2Plus {
  template <typename T>
  bool Plus(const T *x, const T *d, T *out) const {
    out[0] = x[0] + d[0];
    out[1] = x[1] + d[1];
    out[2] = normalizeAngle(x[2] + d[2]);
    return true;
  }
  template <typename T>
  bool Minus(const T *y, const T *x, T *d) const {
    d[0] = y[0] - x[0];
    d[1] = y[1] - x[1];
    d[2] = normalizeAngle(y[2] - x[2]);
    return true;
  }
};

// Sai số between (theo ví dụ pose_graph_2d của Ceres):
//   [R(θa)ᵀ(p_b − p_a) − p_ab ; wrap(θb − θa − θab)], nhân căn bậc hai ma trận thông tin
struct BetweenCost {
  BetweenCost(const Pose2 &m, const Eigen::Matrix3d &sqrt_info) : m_(m), L_(sqrt_info) {}
  template <typename T>
  bool operator()(const T *a, const T *b, T *res) const {
    const T c = ceres::cos(a[2]), s = ceres::sin(a[2]);
    const T dx = b[0] - a[0], dy = b[1] - a[1];
    Eigen::Matrix<T, 3, 1> r;
    r(0) = c * dx + s * dy - T(m_.x);
    r(1) = -s * dx + c * dy - T(m_.y);
    r(2) = normalizeAngle(b[2] - a[2] - T(m_.th));
    Eigen::Map<Eigen::Matrix<T, 3, 1>> out(res);
    out = L_.cast<T>() * r;
    return true;
  }
  Pose2 m_;
  Eigen::Matrix3d L_;
};

Eigen::Matrix3d sqrtInfo(const Eigen::Matrix3d &cov) {
  Eigen::Matrix3d c = 0.5 * (cov + cov.transpose());
  c.diagonal().array() += 1e-12;
  Eigen::LLT<Eigen::Matrix3d> llt(c.inverse());
  return llt.matrixL().transpose();  // Lᵀ với info = L·Lᵀ  ->  ||Lᵀ r||² = rᵀ info r
}

Eigen::Vector3d residual(const Pose2 &a, const Pose2 &b, const Pose2 &m) {
  const double c = std::cos(a.th), s = std::sin(a.th), dx = b.x - a.x, dy = b.y - a.y;
  return {c * dx + s * dy - m.x, -s * dx + c * dy - m.y, wrapAngle(b.th - a.th - m.th)};
}

}  // namespace

double edgeChi2(const std::vector<Pose2> &nodes, const GraphEdge &e) {
  const Eigen::Vector3d r = residual(nodes[e.a], nodes[e.b], e.meas);
  return r.dot(e.cov.inverse() * r);
}

OptimizeResult optimizePoseGraph(std::vector<Pose2> &nodes, const std::vector<GraphEdge> &edges,
                                 const std::vector<int> &fixed, double huber_k, int max_iterations) {
  OptimizeResult out;
  if (nodes.size() < 2 || edges.empty()) {
    out.ok = true;
    return out;
  }
  const auto t0 = std::chrono::steady_clock::now();
  std::vector<std::array<double, 3>> x(nodes.size());
  for (size_t i = 0; i < nodes.size(); i++) x[i] = {nodes[i].x, nodes[i].y, nodes[i].th};

  ceres::Problem::Options popt;
  popt.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem problem(popt);
  ceres::AutoDiffManifold<Pose2Plus, 3, 3> manifold;
  for (const auto &e : edges) {
    auto *cost = new ceres::AutoDiffCostFunction<BetweenCost, 3, 3, 3>(new BetweenCost(e.meas, sqrtInfo(e.cov)));
    ceres::LossFunction *loss = e.robust ? new ceres::HuberLoss(huber_k) : nullptr;
    problem.AddResidualBlock(cost, loss, x[e.a].data(), x[e.b].data());
  }
  for (size_t i = 0; i < nodes.size(); i++)
    if (problem.HasParameterBlock(x[i].data())) problem.SetManifold(x[i].data(), &manifold);
  bool any_fixed = false;
  for (int f : fixed)
    if (f >= 0 && f < int(nodes.size()) && problem.HasParameterBlock(x[f].data())) {
      problem.SetParameterBlockConstant(x[f].data());
      any_fixed = true;
    }
  if (!any_fixed && problem.HasParameterBlock(x[0].data())) problem.SetParameterBlockConstant(x[0].data());

  ceres::Solver::Options opt;
  opt.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  if (!ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::SUITE_SPARSE) &&
      !ceres::IsSparseLinearAlgebraLibraryTypeAvailable(ceres::EIGEN_SPARSE))
    opt.linear_solver_type = ceres::DENSE_QR;
  opt.max_num_iterations = max_iterations;
  opt.num_threads = 1;
  opt.logging_type = ceres::SILENT;
  ceres::Solver::Summary sum;
  ceres::Solve(opt, &problem, &sum);

  for (size_t i = 0; i < nodes.size(); i++) nodes[i] = Pose2(x[i][0], x[i][1], x[i][2]);
  out.ok = sum.IsSolutionUsable();
  out.iterations = int(sum.iterations.size());
  out.initial_cost = sum.initial_cost;
  out.final_cost = sum.final_cost;
  out.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return out;
}

OptimizeResult optimizeWithRejection(std::vector<Pose2> &nodes, const std::vector<GraphEdge> &edges,
                                     const std::vector<int> &fixed, double huber_k, double reject_chi2,
                                     const std::vector<bool> &rejectable, std::vector<size_t> &rejected) {
  rejected.clear();
  const std::vector<Pose2> init = nodes;
  std::vector<char> active(edges.size(), 1);
  for (;;) {
    std::vector<GraphEdge> use;
    for (size_t i = 0; i < edges.size(); i++)
      if (active[i]) use.push_back(edges[i]);
    nodes = init;
    OptimizeResult r = optimizePoseGraph(nodes, use, fixed, huber_k);
    if (!r.ok) return r;
    double worst = reject_chi2;
    long wi = -1;
    for (size_t i = 0; i < edges.size(); i++)
      if (active[i] && i < rejectable.size() && rejectable[i]) {
        const double c = edgeChi2(nodes, edges[i]);
        if (c > worst) worst = c, wi = long(i);
      }
    if (wi < 0) return r;
    active[size_t(wi)] = 0;
    rejected.push_back(size_t(wi));
  }
}

}  // namespace slam
