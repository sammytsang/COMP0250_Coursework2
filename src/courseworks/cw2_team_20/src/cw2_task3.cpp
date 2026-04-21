#include <cw2_class.h>
#include <pcl/common/centroid.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

struct DetectedShape
{
  Eigen::Vector4f centroid;   // base/world frame
  std::string     type;       // "nought" or "cross"
  double          bbox_dim;   // measured outer dimension (m)
  double          arm_off;    // size-snapped arm offset (m)
};

// ── Estimate XY bounding box (average of X and Y extents) ────────────────────
static double estimateBboxDim(PointCPtr cloud)
{
  float xmin =  1e6f, xmax = -1e6f;
  float ymin =  1e6f, ymax = -1e6f;
  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
    xmin = std::min(xmin, pt.x);  xmax = std::max(xmax, pt.x);
    ymin = std::min(ymin, pt.y);  ymax = std::max(ymax, pt.y);
  }
  if (xmin >= xmax || ymin >= ymax) return 0.100;   // fallback: smallest nought tier
  return static_cast<double>((xmax - xmin + ymax - ymin) * 0.5f);
}

// ── Size-snap bbox to nearest known tier and return exact arm_off ─────────────
//
// Size-snapping table (bar width x):
//   Nought outer = 5×x  →  100mm(x=20), 150mm(x=30), 200mm(x=40)
//   Nought arm_off = 2×x →  40mm,        60mm,        80mm
//
//   Cross  outer = 3×x  →   60mm(x=20),  90mm(x=30), 120mm(x=40)
//   Cross  arm_off = 1.5×x→  30mm,        45mm,        60mm
static double snapArmOff(const std::string & type, double measured_bbox,
                          double & snapped_bbox_out)
{
  if (type == "nought") {
    constexpr double tiers[]    = {0.100, 0.150, 0.200};
    constexpr double arm_offs[] = {0.040, 0.060, 0.080};
    int best = 0;
    double best_d = std::abs(measured_bbox - tiers[0]);
    for (int i = 1; i < 3; ++i) {
      const double d = std::abs(measured_bbox - tiers[i]);
      if (d < best_d) { best_d = d; best = i; }
    }
    snapped_bbox_out = tiers[best];
    return arm_offs[best];
  } else {
    constexpr double tiers[]    = {0.060, 0.090, 0.120};
    constexpr double arm_offs[] = {0.030, 0.045, 0.060};
    int best = 0;
    double best_d = std::abs(measured_bbox - tiers[0]);
    for (int i = 1; i < 3; ++i) {
      const double d = std::abs(measured_bbox - tiers[i]);
      if (d < best_d) { best_d = d; best = i; }
    }
    snapped_bbox_out = tiers[best];
    return arm_offs[best];
  }
}

// ── Material-search yaw for nought (robust for any size; PCA is degenerate) ───
//   Scans [0°,90°) in 1° steps, counting cloud points within SEARCH_R of the
//   four face-normal points (centroid ± arm_off along θ and θ+90°).
//   Matches Task 1's exact approach.
static double detectNoughtYaw(PointCPtr cloud, float arm_off)
{
  if (!cloud || cloud->size() < 10) return 0.0;
  const float search_r = 0.025f;   // 25mm capture radius (matches Task 1)

  Eigen::Vector4f cen;
  pcl::compute3DCentroid(*cloud, cen);
  const float cx = cen[0], cy = cen[1];

  int    best_cnt   = -1;
  double best_theta = 0.0;
  for (int i = 0; i < 90; ++i) {
    const double theta = i * (M_PI / 2.0) / 90.0;
    int cnt = 0;
    for (int k = 0; k < 4; ++k) {
      const double t  = theta + k * M_PI / 2.0;
      const float  tx = cx + arm_off * static_cast<float>(std::cos(t));
      const float  ty = cy + arm_off * static_cast<float>(std::sin(t));
      for (const auto & pt : cloud->points) {
        const float dx = pt.x - tx, dy = pt.y - ty;
        if (dx*dx + dy*dy < search_r * search_r) ++cnt;
      }
    }
    if (cnt > best_cnt) { best_cnt = cnt; best_theta = theta; }
  }
  return best_theta;
}

// ── Keep only shape-coloured points (R>0.50 OR B>0.50) ───────────────────────
//   Catches purple, red, and blue shapes while rejecting black obstacles and
//   green ground tiles.
static PointCPtr filterShapeColour(PointCPtr cloud)
{
  PointCPtr out(new PointC);
  for (const auto & pt : cloud->points) {
    const float r = pt.r / 255.0f;
    const float b = pt.b / 255.0f;
    if (r > 0.50f || b > 0.50f) out->points.push_back(pt);
  }
  out->width    = static_cast<uint32_t>(out->points.size());
  out->height   = 1;
  out->is_dense = true;
  return out;
}

}  // namespace

void cw2::t3_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> /*request*/,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Started =====");
  response->total_num_shapes      = 0;
  response->num_most_common_shape = 0;
  response->most_common_shape_vector.clear();

  arm_group_->stop();
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Retry homing up to 30 times to survive sim startup lag.
  {
    bool homed = false;
    for (int i = 0; i < 30 && !homed; ++i) {
      arm_group_->setStartStateToCurrentState();
      if (moveArmToNamedTarget("ready")) {
        homed = true;
      } else {
        RCLCPP_WARN(node_->get_logger(),
          "Task 3: waiting for arm ready (attempt %d/30)...", i+1);
        rclcpp::sleep_for(std::chrono::seconds(2));
      }
    }
    if (!homed)
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: could not home arm — proceeding anyway");
  }

  // ══════════════════════════════════════════════════════════════════════════
  // PHASE 1 — COARSE SCAN
  // Visit all 9 grid positions at safe heights, accumulate a single combined
  // point cloud, then return to "ready" before any processing.
  // ══════════════════════════════════════════════════════════════════════════

  PointCPtr combined(new PointC);
  const std::vector<std::tuple<double,double,double>> scan_pts = {
    {-0.35, -0.45, 0.65}, {-0.35,  0.00, 0.80}, {-0.35, +0.45, 0.65},
    {+0.05, -0.45, 0.65}, {+0.05,  0.00, 0.80}, {+0.05, +0.45, 0.65},
    {+0.40, -0.45, 0.65}, {+0.40,  0.00, 0.80}, {+0.40, +0.45, 0.65}
  };
  for (const auto & [sx, sy, sz] : scan_pts) {
    arm_group_->setStartStateToCurrentState();
    if (!moveArmToPose(makeDownwardPose(sx, sy, sz, 0.0))) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: scan move failed (%.2f,%.2f) — skipping", sx, sy);
      continue;
    }
    PointCPtr raw = waitForFreshCloud(3000);
    if (!raw || raw->empty()) continue;
    PointCPtr base = transformCloudToBaseFrame(raw);
    if (base && !base->empty()) *combined += *base;
  }

  // Return to ready before processing.
  moveArmToNamedTarget("ready");

  if (combined->empty()) {
    RCLCPP_WARN(node_->get_logger(), "Task 3: no cloud data — aborting");
    return;
  }

  // Z-crop to < 0.15m: strips the arm, keeps ground-level objects.
  {
    PointCPtr ground_only(new PointC);
    for (const auto & pt : combined->points)
      if (std::isfinite(pt.z) && pt.z < 0.15f) ground_only->points.push_back(pt);
    ground_only->width  = static_cast<uint32_t>(ground_only->points.size());
    ground_only->height = 1; ground_only->is_dense = true;
    combined = ground_only;
  }

  // Voxel downsample at 5mm.
  combined = voxelDownsample(combined, 0.005f);
  RCLCPP_INFO(node_->get_logger(),
    "Task 3: combined cloud after z-crop+downsample: %zu points", combined->size());

  // ── Obstacle detection → MoveIt collision boxes ──────────────────────────
  std::vector<std::string> obstacle_ids;
  {
    PointCPtr dark = filterByColourRange(combined,
                       0.0f, 0.15f, 0.0f, 0.15f, 0.0f, 0.15f);
    PointCPtr dark_up(new PointC);
    for (const auto & pt : dark->points)
      if (std::isfinite(pt.z) && pt.z > 0.010f) dark_up->points.push_back(pt);
    dark_up->width = static_cast<uint32_t>(dark_up->points.size());
    dark_up->height = 1; dark_up->is_dense = true;
    auto obs_cl = euclideanCluster(dark_up, 0.03f, 30, 20000);

    for (size_t i = 0; i < obs_cl.size(); ++i) {
      const auto & c = obs_cl[i];
      if (!c || c->empty()) continue;
      Eigen::Vector4f cen = getCloudCentroid(c);
      float xmin=1e6f, xmax=-1e6f, ymin=1e6f, ymax=-1e6f;
      for (const auto & pt : c->points) {
        xmin=std::min(xmin,pt.x); xmax=std::max(xmax,pt.x);
        ymin=std::min(ymin,pt.y); ymax=std::max(ymax,pt.y);
      }
      const float osx = (xmax-xmin) + 0.04f;
      const float osy = (ymax-ymin) + 0.04f;
      const std::string id = "t3_obs_" + std::to_string(i);
      addBoxCollisionObject(id, cen.x(), cen.y(), GROUND_Z+0.10, osx, osy, 0.22);
      obstacle_ids.push_back(id);
      RCLCPP_INFO(node_->get_logger(),
        "Task 3: obstacle %zu at (%.3f,%.3f) box(%.0f×%.0fmm)",
        i, cen.x(), cen.y(), osx*1000.0f, osy*1000.0f);
    }
  }
  auto cleanupObstacles = [&]() {
    for (const auto & id : obstacle_ids) removeCollisionObject(id);
  };

  // Basket candidates — defined early so they can also be used to exclude
  // basket-wall clusters from coarse position finding.
  const std::vector<std::pair<double,double>> basket_candidates = {
    {-0.41, -0.36}, {-0.41, 0.36}
  };

  // ── Coarse shape positions ────────────────────────────────────────────────
  // Find approximate cluster centroids from the combined cloud.
  // Apply bbox and density guards but do NOT classify type here.
  std::vector<std::pair<double,double>> coarse_positions;
  {
    PointCPtr shape_col = filterShapeColour(combined);
    PointCPtr shape_up(new PointC);
    for (const auto & pt : shape_col->points)
      if (std::isfinite(pt.z) && pt.z > 0.010f) shape_up->points.push_back(pt);
    shape_up->width = static_cast<uint32_t>(shape_up->points.size());
    shape_up->height = 1; shape_up->is_dense = true;
    auto clusters = euclideanCluster(shape_up, 0.03f, 50, 30000);

    for (const auto & cl : clusters) {
      if (!cl || cl->size() < 30) continue;
      const double bbox = estimateBboxDim(cl);
      if (bbox < 0.055 || bbox > 0.25) continue;
      const Eigen::Vector4f cen = getCloudCentroid(cl);
      if (std::hypot(cen.x(), cen.y()) < 0.15f) continue;

      float max_z = -1e6f;
      for (const auto & pt : cl->points) max_z = std::max(max_z, pt.z);
      if (max_z < 0.040f || max_z > 0.080f) continue;
      max_z = std::min(max_z, 0.070f);
      PointCPtr top_face(new PointC);
      for (const auto & pt : cl->points)
        if (pt.z > max_z - 0.008f) top_face->points.push_back(pt);
      top_face->width = static_cast<uint32_t>(top_face->points.size());
      top_face->height = 1; top_face->is_dense = true;
      if (top_face->size() < 10) continue;

      // Density guard.
      const float min_pts = static_cast<float>(bbox * bbox) * 5000.0f;
      if (static_cast<float>(top_face->size()) < min_pts) {
        RCLCPP_WARN(node_->get_logger(),
          "Task 3 coarse: cluster at (%.3f,%.3f) rejected — low density "
          "(top_face=%zu < required=%.0f)",
          cen.x(), cen.y(), top_face->size(), static_cast<double>(min_pts));
        continue;
      }

      // Proximity guard: skip if within 0.32m of a basket candidate.
      bool near_basket = false;
      for (const auto & [bx, by] : basket_candidates)
        if (std::hypot(cen.x() - bx, cen.y() - by) < 0.32) { near_basket = true; break; }
      if (near_basket) continue;

      coarse_positions.push_back({static_cast<double>(cen.x()),
                                   static_cast<double>(cen.y())});
      RCLCPP_INFO(node_->get_logger(),
        "Task 3 coarse: cluster at (%.3f,%.3f) bbox=%.0fmm top_face=%zu",
        cen.x(), cen.y(), bbox*1000.0, top_face->size());
    }
  }
  RCLCPP_INFO(node_->get_logger(),
    "Task 3: %zu coarse positions found", coarse_positions.size());

  // ── Basket detection ──────────────────────────────────────────────────────
  double basket_x = basket_candidates[0].first;
  double basket_y = basket_candidates[0].second;
  {
    int best = 0;
    for (const auto & [bx, by] : basket_candidates) {
      int cnt = 0;
      for (const auto & pt : combined->points) {
        if (std::abs(pt.x - static_cast<float>(bx)) < 0.22f &&
            std::abs(pt.y - static_cast<float>(by)) < 0.22f) ++cnt;
      }
      RCLCPP_INFO(node_->get_logger(),
        "Task 3: basket candidate (%.2f,%.2f) → %d pts", bx, by, cnt);
      if (cnt > best) { best = cnt; basket_x = bx; basket_y = by; }
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Task 3: basket at (%.2f,%.2f)", basket_x, basket_y);

  addBoxCollisionObject("t3_bsk_N", basket_x+0.19, basket_y,      GROUND_Z+0.025, 0.02, 0.37, 0.06);
  addBoxCollisionObject("t3_bsk_S", basket_x-0.19, basket_y,      GROUND_Z+0.025, 0.02, 0.37, 0.06);
  addBoxCollisionObject("t3_bsk_E", basket_x,      basket_y+0.19, GROUND_Z+0.025, 0.37, 0.02, 0.06);
  addBoxCollisionObject("t3_bsk_W", basket_x,      basket_y-0.19, GROUND_Z+0.025, 0.37, 0.02, 0.06);

  // ══════════════════════════════════════════════════════════════════════════
  // PHASE 2 — INDIVIDUAL CONFIRMATION SCAN (Task 2's exact method)
  // For each coarse position: move 0.50m above, classify from close-up cloud,
  // return to "ready" after each one.
  // ══════════════════════════════════════════════════════════════════════════

  constexpr double CLOSEUP_HEIGHT = 0.50;
  constexpr double CLOSEUP_CROP   = 0.15;

  std::vector<DetectedShape> detected;
  for (const auto & [px, py] : coarse_positions) {
    if (!moveArmToPose(makeDownwardPose(px, py, CLOSEUP_HEIGHT, 0.0))) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: confirmation move failed at (%.3f,%.3f) — skipping", px, py);
      moveArmToNamedTarget("ready");
      continue;
    }

    PointCPtr raw = waitForFreshCloud(5000);
    if (!raw || raw->empty()) {
      moveArmToNamedTarget("ready");
      continue;
    }

    PointCPtr base     = transformCloudToBaseFrame(raw);
    PointCPtr cropped  = cropBoxAroundPoint(base, px, py, 0.0, CLOSEUP_CROP);
    PointCPtr no_ground = removeGroundPlane(cropped, 0.012f);

    if (!no_ground || no_ground->empty()) {
      moveArmToNamedTarget("ready");
      continue;
    }

    // Top face: keep points within 12mm of the highest z.
    float cf_max_z = -1e6f;
    for (const auto & p : no_ground->points)
      if (std::isfinite(p.z)) cf_max_z = std::max(cf_max_z, p.z);
    PointCPtr top_face(new PointC);
    for (const auto & p : no_ground->points)
      if (std::isfinite(p.z) && p.z > cf_max_z - 0.012f)
        top_face->points.push_back(p);
    top_face->width  = static_cast<uint32_t>(top_face->points.size());
    top_face->height = 1; top_face->is_dense = true;

    if (top_face->size() < 30) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: too few top-face points (%zu) at (%.3f,%.3f) — skipping",
        top_face->size(), px, py);
      moveArmToNamedTarget("ready");
      continue;
    }

    const std::string type = classifyShape(top_face);
    if (type != "nought" && type != "cross") {
      moveArmToNamedTarget("ready");
      continue;
    }

    Eigen::Vector4f cen    = getCloudCentroid(top_face);
    const double accurate_cx = static_cast<double>(cen.x());
    const double accurate_cy = static_cast<double>(cen.y());
    const double bbox        = estimateBboxDim(top_face);
    double snapped_bbox      = bbox;
    const double arm_off     = snapArmOff(type, bbox, snapped_bbox);

    detected.push_back({cen, type, snapped_bbox, arm_off});
    RCLCPP_INFO(node_->get_logger(),
      "Task3 confirmed: %s at (%.3f,%.3f) bbox=%.0fmm arm_off=%.0fmm",
      type.c_str(), accurate_cx, accurate_cy,
      snapped_bbox*1000.0, arm_off*1000.0);

    moveArmToNamedTarget("ready");
  }

  // Basket proximity post-filter (0.32m).
  detected.erase(
    std::remove_if(detected.begin(), detected.end(),
      [&](const DetectedShape & s) {
        for (const auto & [bx, by] : basket_candidates)
          if (std::hypot(s.centroid.x() - bx, s.centroid.y() - by) < 0.32)
            return true;
        return false;
      }),
    detected.end());

  // Count and report.
  int n_nought = 0, n_cross = 0;
  for (const auto & s : detected) {
    if (s.type == "nought") ++n_nought; else ++n_cross;
  }
  const int         total      = n_nought + n_cross;
  const std::string common     = (n_nought >= n_cross) ? "nought" : "cross";
  const int         num_common = (n_nought >= n_cross) ? n_nought : n_cross;

  response->total_num_shapes      = total;
  response->num_most_common_shape = num_common;

  RCLCPP_INFO(node_->get_logger(),
    "Task 3: noughts=%d crosses=%d total=%d → most_common=%s(%d)",
    n_nought, n_cross, total, common.c_str(), num_common);

  // ══════════════════════════════════════════════════════════════════════════
  // PHASE 3 — PICK AND PLACE (Task 1's exact method)
  // ══════════════════════════════════════════════════════════════════════════

  // Sort candidates of the most common type by distance from base, nearest first.
  std::vector<DetectedShape *> candidates;
  for (auto & s : detected)
    if (s.type == common) candidates.push_back(&s);
  std::sort(candidates.begin(), candidates.end(),
    [](const DetectedShape * a, const DetectedShape * b) {
      return std::hypot(a->centroid.x(), a->centroid.y()) <
             std::hypot(b->centroid.x(), b->centroid.y());
    });

  bool pick_success = false;
  for (DetectedShape * shape : candidates) {
    const double cx     = static_cast<double>(shape->centroid.x());
    const double cy     = static_cast<double>(shape->centroid.y());
    const double arm_off = shape->arm_off;

    if (std::hypot(cx, cy) < 0.20) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: skipping %s at (%.3f,%.3f) — too close to base",
        shape->type.c_str(), cx, cy);
      continue;
    }

    // Move 0.50m above the confirmed centroid and take a fresh cloud.
    if (!moveArmToPose(makeDownwardPose(cx, cy, CLOSEUP_HEIGHT, 0.0))) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: pick scan move failed for %s at (%.3f,%.3f) — skipping",
        shape->type.c_str(), cx, cy);
      moveArmToNamedTarget("ready");
      continue;
    }

    PointCPtr raw2 = waitForFreshCloud(5000);
    if (!raw2 || raw2->empty()) {
      moveArmToNamedTarget("ready");
      continue;
    }

    PointCPtr base2     = transformCloudToBaseFrame(raw2);
    PointCPtr cropped2  = cropBoxAroundPoint(base2, cx, cy, 0.0, CLOSEUP_CROP);
    PointCPtr no_ground2 = removeGroundPlane(cropped2, 0.012f);

    // Grasp yaw from no_ground cloud (Task 1 exact method).
    double grasp_yaw = 0.0;
    double pick_cx   = cx;
    double pick_cy   = cy;

    if (no_ground2 && no_ground2->size() >= 10) {
      if (shape->type == "nought") {
        // Recompute centroid from no_ground for maximum positional accuracy.
        Eigen::Vector4f ng_cen = getCloudCentroid(no_ground2);
        pick_cx = static_cast<double>(ng_cen.x());
        pick_cy = static_cast<double>(ng_cen.y());
        grasp_yaw = detectNoughtYaw(no_ground2, static_cast<float>(arm_off));
      } else {
        float raw_yaw = detectShapeYaw(no_ground2);
        grasp_yaw = static_cast<double>(raw_yaw);
        while (grasp_yaw <  0.0)        grasp_yaw += M_PI / 2.0;
        while (grasp_yaw >= M_PI / 2.0) grasp_yaw -= M_PI / 2.0;
      }
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: sparse pick cloud for %s at (%.3f,%.3f) — using grasp_yaw=0",
        shape->type.c_str(), cx, cy);
    }

    geometry_msgs::msg::PointStamped grasp_pt;
    grasp_pt.header.frame_id = "panda_link0";
    grasp_pt.point.x = pick_cx + arm_off * std::cos(grasp_yaw);
    grasp_pt.point.y = pick_cy + arm_off * std::sin(grasp_yaw);
    grasp_pt.point.z = GROUND_Z;

    const double pick_yaw = (shape->type == "nought")
      ? grasp_yaw + M_PI / 4.0
      : grasp_yaw + 3.0 * M_PI / 4.0;

    if (shape->type == "nought") {
      RCLCPP_INFO(node_->get_logger(),
        "Nought: grasp=(%.3f,%.3f) offset=%.3fm edge_normal=%.1fdeg pick_yaw=%.1fdeg",
        grasp_pt.point.x, grasp_pt.point.y, arm_off,
        grasp_yaw * 180.0 / M_PI, pick_yaw * 180.0 / M_PI);
    } else {
      RCLCPP_INFO(node_->get_logger(),
        "Cross: grasp=(%.3f,%.3f) offset=%.3fm across_arm=%.1fdeg pick_yaw=%.1fdeg",
        grasp_pt.point.x, grasp_pt.point.y, arm_off,
        (grasp_yaw + M_PI / 2.0) * 180.0 / M_PI, pick_yaw * 180.0 / M_PI);
    }

    if (pickObject(grasp_pt, pick_yaw)) {
      // Lift straight up first (Task 1 exact method) to avoid wrist spin.
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      arm_group_->setStartStateToCurrentState();
      {
        auto cur_pose = arm_group_->getCurrentPose().pose;
        const double safe_z = std::max(cur_pose.position.z + 0.40, 0.65);
        geometry_msgs::msg::Pose lift_pose =
          makeDownwardPose(cur_pose.position.x, cur_pose.position.y,
                           safe_z, pick_yaw);
        if (!moveArmToPose(lift_pose))
          RCLCPP_WARN(node_->get_logger(),
            "Task 3: vertical lift failed — attempting place anyway");
      }

      // Place into basket (Task 1 exact method).
      geometry_msgs::msg::PointStamped basket_pt;
      basket_pt.header.frame_id = "panda_link0";
      double place_yaw;
      if (shape->type == "cross") {
        place_yaw        = 3.0 * M_PI / 4.0;
        basket_pt.point.x = basket_x + 0.060;  // ARM_OFFSET for cross
        basket_pt.point.y = basket_y;
      } else {
        place_yaw         = pick_yaw;           // nought: keep pick direction
        basket_pt.point.x = basket_x + arm_off * std::cos(grasp_yaw);
        basket_pt.point.y = basket_y + arm_off * std::sin(grasp_yaw);
      }
      basket_pt.point.z = GROUND_Z;

      if (!placeObject(basket_pt, place_yaw))
        RCLCPP_WARN(node_->get_logger(), "Task 3: place failed");

      pick_success = true;
      break;
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: pick failed for %s at (%.3f,%.3f) — trying next candidate",
        shape->type.c_str(), pick_cx, pick_cy);
    }
  }

  if (!pick_success)
    RCLCPP_WARN(node_->get_logger(), "Task 3: all pick attempts failed");

  // ══════════════════════════════════════════════════════════════════════════
  // PHASE 4 — CLEANUP
  // ══════════════════════════════════════════════════════════════════════════

  removeCollisionObject("t3_bsk_N"); removeCollisionObject("t3_bsk_S");
  removeCollisionObject("t3_bsk_E"); removeCollisionObject("t3_bsk_W");
  cleanupObstacles();
  moveArmToNamedTarget("ready");
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Complete =====");
}
