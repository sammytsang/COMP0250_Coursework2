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

  // Ensure the arm is at the home ("ready") position before starting grid scan.
  // On macOS, sim_time can lag wall-time, causing MoveIt's robot state monitor
  // to report stale state.  Moving to a named joint target first forces MoveIt
  // to accept the current joint state, after which Cartesian planning works.
  // Retry up to 30 times (≈60 s) so we don't race against sim startup.
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
    if (!homed) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: could not home arm — proceeding anyway");
    }
  }

  // ── STEP 1: Systematic 3×3 grid scan covering the full workspace ─────────
  //   Scan ALL 9 positions FIRST, then return to "ready" before processing.
  //   Centre/near positions use z=0.80m (elbow stays high, no collision risk).
  //   Corner/edge positions (Y=±0.45) use z=0.65m because the Panda's reachable
  //   horizontal radius at z=0.80 is insufficient for those lateral extents.
  //
  //   Grid (X, Y, Z):
  //     corners/edges z=0.65:  (-0.35,±0.45), (+0.05,±0.45), (+0.40,±0.45)
  //     centre row    z=0.80:  (-0.35,0), (+0.05,0), (+0.40,0)
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

  // Return to ready BEFORE any processing or picking
  moveArmToNamedTarget("ready");

  if (combined->empty()) {
    RCLCPP_WARN(node_->get_logger(), "Task 3: no cloud data — aborting");
    moveArmToNamedTarget("ready");
    return;
  }

  // Z-crop to <0.15m: strips the robot arm from the cloud, keeping only
  // ground-level objects (shapes, obstacles, basket, floor tiles).
  {
    PointCPtr ground_only(new PointC);
    for (const auto & pt : combined->points) {
      if (std::isfinite(pt.z) && pt.z < 0.15f) ground_only->points.push_back(pt);
    }
    ground_only->width    = static_cast<uint32_t>(ground_only->points.size());
    ground_only->height   = 1;
    ground_only->is_dense = true;
    combined = ground_only;
  }

  // Voxel-downsample the combined cloud (5mm leaf) to deduplicate overlapping
  // voxels from the 12 scan positions and reduce noise in bbox estimation.
  combined = voxelDownsample(combined, 0.005f);
  RCLCPP_INFO(node_->get_logger(),
    "Task 3: combined cloud after z-crop+downsample: %zu points",
    combined->size());

  // ── STEP 2: Detect obstacles → add MoveIt collision boxes ────────────────
  std::vector<std::string> obstacle_ids;
  {
    PointCPtr dark = filterByColourRange(combined,
                       0.0f, 0.15f, 0.0f, 0.15f, 0.0f, 0.15f);
    // After dark-colour filter the floor is already excluded (floor tiles are
    // green, R/G/B ≈ 0.0/0.8/0.0 — not all-dark).  Use a Z > 0.010m threshold
    // instead of removeGroundPlane: RANSAC would otherwise fit to the flat top
    // face of the obstacle and remove it from the cluster.
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
      // Tight bounding box + 20mm safety buffer each side
      const float sx = (xmax-xmin) + 0.04f;
      const float sy = (ymax-ymin) + 0.04f;
      const std::string id = "t3_obs_" + std::to_string(i);
      addBoxCollisionObject(id, cen.x(), cen.y(), GROUND_Z+0.10, sx, sy, 0.22);
      obstacle_ids.push_back(id);
      RCLCPP_INFO(node_->get_logger(),
        "Task 3: obstacle %zu at (%.3f,%.3f) box(%.0f×%.0fmm)",
        i, cen.x(), cen.y(), sx*1000.0f, sy*1000.0f);
    }
  }
  auto cleanupObstacles = [&]() {
    for (const auto & id : obstacle_ids) removeCollisionObject(id);
  };

  // ── STEP 3: Detect shapes → classify and size-snap ───────────────────────
  // KEY 1: do NOT call removeGroundPlane on the shape-coloured cloud.
  // After filterShapeColour (R>0.5 OR B>0.5) there are no green floor tiles;
  // RANSAC would fit through the flat top faces of the shapes themselves,
  // removing them and leaving only vertical side faces — which inverts
  // hole-detection: noughts appear solid, crosses appear to have holes.
  //
  // KEY 2: before classifyShape(), filter each cluster to its TOP FACE ONLY
  // (z > cluster_max_z − 10mm).  The 12-position combined cloud includes
  // oblique-angle side-face points; projecting side faces into XY can create
  // false enclosed regions in crosses and fill in the central hole of noughts.
  // The top face alone gives a clean 2D projection for hole detection.
  // Basket candidates defined early so they can filter basket-wall clusters
  // from shape detection.  The basket (RGB≈[0.5,0.2,0.2]) passes the R>0.50
  // colour filter and its ~185mm bbox passes the cross→nought reclassification
  // rule, producing a spurious "nought" near the basket position.
  const std::vector<std::pair<double,double>> basket_candidates = {
    {-0.41, -0.36}, {-0.41,  0.36}
  };

  std::vector<DetectedShape> detected;
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
      // Valid shape range: cross x=20mm → 60mm outer, nought x=40mm → 200mm outer.
      // Allow ±50% around smallest/largest tiers: [30mm, 250mm].
      if (bbox < 0.055 || bbox > 0.25) continue;
      const Eigen::Vector4f cen = getCloudCentroid(cl);
      // Discard clusters whose centroid is < 0.15m from the robot base: these
      // are artefacts from robot-body reflections, not real shapes.
      if (std::hypot(cen.x(), cen.y()) < 0.15f) continue;

      // Build a top-face-only view of the cluster for classification.
      // Upright shapes have ground at z≈0.020 and top at z≈0.060 (height=0.040).
      // Reject clusters outside the plausible range:
      //   max_z < 0.040 → too flat (tilted/sideways shape fragment or ground noise)
      //   max_z > 0.080 → too tall (obstacle bleed-through or basket rim)
      float max_z = -1e6f;
      for (const auto & pt : cl->points) max_z = std::max(max_z, pt.z);
      if (max_z < 0.040f || max_z > 0.080f) continue;
      max_z = std::min(max_z, 0.070f);   // clamp for top-face slice
      PointCPtr top_face(new PointC);
      for (const auto & pt : cl->points)
        if (pt.z > max_z - 0.008f) top_face->points.push_back(pt);
      top_face->width = static_cast<uint32_t>(top_face->points.size());
      top_face->height = 1; top_face->is_dense = true;
      RCLCPP_INFO(node_->get_logger(),
        "Task 3: cluster at (%.3f,%.3f) max_z=%.3f top_face_pts=%zu total_pts=%zu",
        cen.x(), cen.y(), max_z, top_face->size(), cl->size());
      if (top_face->size() < 10) continue;   // too few top-face points

      // Density guard: ghost/noise clusters have very few points relative to
      // their measured bounding box.  A real shape of diameter D has point
      // density roughly proportional to D².  Threshold: top_face_pts should
      // be ≥ 5000 × D² (SI units).  For a genuine 200mm nought this is ~200 pts
      // (actual ~1200).  For a noise cluster with D=221mm but only 132 pts it
      // computes 244 required → ghost is rejected.
      {
        const float min_pts = static_cast<float>(bbox * bbox) * 5000.0f;
        if (static_cast<float>(top_face->size()) < min_pts) {
          RCLCPP_WARN(node_->get_logger(),
            "Task 3: cluster at (%.3f,%.3f) rejected — low density "
            "(top_face=%zu < required=%.0f)", cen.x(), cen.y(),
            top_face->size(), static_cast<double>(min_pts));
          continue;
        }
      }

      std::string type = classifyShape(top_face);
      if (type != "nought" && type != "cross") continue;

      // Similarly: nought outer = 5×bar, max 5×40=200mm.  A "nought" with bbox
      // much larger than 230mm (>15% above max) is likely two merged shapes — skip.
      if (type == "nought" && bbox > 0.230) {
        RCLCPP_WARN(node_->get_logger(),
          "Task 3: cluster at (%.3f,%.3f) skipped — nought bbox=%.0fmm > 230mm"
          " (likely merged clusters)", cen.x(), cen.y(), bbox*1000.0);
        continue;
      }

      double snapped_bbox = bbox;
      const double arm_off = snapArmOff(type, bbox, snapped_bbox);
      detected.push_back({cen, type, snapped_bbox, arm_off});
      RCLCPP_INFO(node_->get_logger(),
        "Task 3: %s at (%.3f,%.3f) raw_bbox=%.0fmm → snapped=%.0fmm arm_off=%.0fmm",
        type.c_str(), cen.x(), cen.y(),
        bbox*1000.0, snapped_bbox*1000.0, arm_off*1000.0);
    }
  }

  // Post-filter: remove any detected cluster whose centroid is within 0.32m of
  // a basket candidate.  Basket walls pass the colour and bbox filters and can
  // be misclassified as noughts; this removes them before counting.
  detected.erase(
    std::remove_if(detected.begin(), detected.end(),
      [&](const DetectedShape & s) {
        for (const auto & [bx, by] : basket_candidates) {
          if (std::hypot(s.centroid.x() - bx,
                         s.centroid.y() - by) < 0.32)
            return true;
        }
        return false;
      }),
    detected.end());

  // ── STEP 4: Count and report ─────────────────────────────────────────────
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

  // ── STEP 5: Detect basket (2 known candidate positions) ──────────────────
  // basket_candidates already defined above for shape post-filtering.
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

  // ── STEP 6: Per-shape pick loop ───────────────────────────────────────────
  // Sort candidates of the most common type by distance from base, nearest first
  // (shorter arm extension → more stable grasp).
  std::vector<DetectedShape *> candidates;
  for (auto & s : detected) {
    if (s.type == common) candidates.push_back(&s);
  }
  std::sort(candidates.begin(), candidates.end(),
    [](const DetectedShape * a, const DetectedShape * b) {
      return std::hypot(a->centroid.x(), a->centroid.y()) <
             std::hypot(b->centroid.x(), b->centroid.y());
    });

  bool pick_success = false;
  for (DetectedShape * shape : candidates) {
    const double cx = shape->centroid.x();
    const double cy = shape->centroid.y();

    // Shapes < 0.20m from base are in a singularity zone — skip them
    if (std::hypot(cx, cy) < 0.20) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: skipping %s at (%.3f,%.3f) — too close to base",
        shape->type.c_str(), cx, cy);
      continue;
    }

    // ── Phase 5a: Individual close-up scan ──────────────────────────────────
    //   Move 0.50m above the wide-scan centroid, take a fresh cloud, crop
    //   0.15m half-side (matching Task 1).  This is large enough to contain a
    //   200mm (x=40mm) nought even with ~50mm centroid error from the wide scan.
    //   Using 0.10m was too small: a 200mm shape barely fits, and any centroid
    //   error clips the ring, corrupting the material-search and arm_off estimate.
    constexpr double CLOSEUP_HEIGHT  = 0.50;   // above ground (matches Task 1)
    constexpr double CLOSEUP_CROP    = 0.15;   // crop half-side (m) — matches Task 1

    double grasp_yaw  = 0.0;
    double accurate_cx = cx, accurate_cy = cy;
    double arm_off = shape->arm_off;   // start with wide-scan snapped value

    if (moveArmToPose(makeDownwardPose(cx, cy, CLOSEUP_HEIGHT, 0.0))) {
      PointCPtr raw = waitForFreshCloud(5000);
      if (raw && !raw->empty()) {
        PointCPtr base    = transformCloudToBaseFrame(raw);
        PointCPtr cropped = cropBoxAroundPoint(base, cx, cy, 0.0, CLOSEUP_CROP);
        PointCPtr col = filterShapeColour(cropped);
        // z > 0.025m: eliminates floor-bleed coloured pixels (which appear at
        // z≈0–0.020m due to edge interpolation at 0.5m range) while keeping
        // the shape body (bottom at z=0.020m, top at z=0.060m).  Cutting the
        // bottom 5mm slice (z=0.020–0.025m) does not affect the XY bbox because
        // the outer ring radius is the same at all heights.  Critically, with
        // floor bleed excluded there is no path for the Euclidean clusterer to
        // bridge from floor noise into the shape, so the cluster stays tight.
        PointCPtr col_up(new PointC);
        for (const auto & pt : col->points)
          if (std::isfinite(pt.z) && pt.z > 0.025f) col_up->points.push_back(pt);
        col_up->width = static_cast<uint32_t>(col_up->points.size());
        col_up->height = 1; col_up->is_dense = true;
        PointCPtr ds = voxelDownsample(col_up, 0.005f);

        if (ds && ds->size() >= 10) {
          auto cls = euclideanCluster(ds, 0.015f, 10, 30000);
          PointCPtr best_cl;
          double best_d = 1e9;
          for (const auto & cl : cls) {
            if (!cl || cl->empty()) continue;
            Eigen::Vector4f c = getCloudCentroid(cl);
            const double d = std::hypot(c.x() - cx, c.y() - cy);
            if (d < best_d) { best_d = d; best_cl = cl; }
          }
          if (!best_cl || best_d > 0.15) best_cl = ds;  // fallback to full DS cloud

          // ── Type verification: classify close-up top face and compare ──────
          // This guards against the nearest cluster belonging to a neighbouring
          // shape whose wide-scan centroid was slightly off.
          float cu_max_z = -1e6f;
          for (const auto & p : best_cl->points)
            if (std::isfinite(p.z)) cu_max_z = std::max(cu_max_z, p.z);
          PointCPtr cu_top(new PointC);
          for (const auto & p : best_cl->points)
            if (std::isfinite(p.z) && p.z > cu_max_z - 0.012f)
              cu_top->points.push_back(p);
          cu_top->width = static_cast<uint32_t>(cu_top->points.size());
          cu_top->height = 1; cu_top->is_dense = true;

          const std::string closeup_type =
            (cu_top->size() >= 30) ? classifyShape(cu_top) : "unknown";

          if (closeup_type != shape->type && closeup_type != "unknown") {
            RCLCPP_WARN(node_->get_logger(),
              "Task 3 closeup: type mismatch — wide=%s, closeup=%s. "
              "Using wide-scan values.",
              shape->type.c_str(), closeup_type.c_str());
            // Keep accurate_cx/cy, arm_off, grasp_yaw at their wide-scan values.
          } else {
            // Type confirmed (or indeterminate) — use close-up values.
            Eigen::Vector4f close_cen = getCloudCentroid(best_cl);
            accurate_cx = close_cen.x();
            accurate_cy = close_cen.y();

            // Re-snap arm_off from close-up bbox for maximum precision
            const double close_bbox = estimateBboxDim(best_cl);
            double snapped_close = close_bbox;
            arm_off = snapArmOff(shape->type, close_bbox, snapped_close);
            RCLCPP_INFO(node_->get_logger(),
              "Task 3 closeup: snapped arm_off=%.0fmm (close_bbox=%.0fmm → snapped=%.0fmm)",
              arm_off*1000.0, close_bbox*1000.0, snapped_close*1000.0);

            // ── Phase 5b: Task 1 exact yaw method ──────────────────────────
            // Filter best_cl to top face only before yaw detection —
            // side-face points corrupt the angle search (same filter as Task 1).
            float yaw_max_z = -1e6f;
            for (const auto & p : best_cl->points)
              if (std::isfinite(p.z)) yaw_max_z = std::max(yaw_max_z, p.z);
            PointCPtr yaw_cloud(new PointC);
            for (const auto & p : best_cl->points)
              if (std::isfinite(p.z) && p.z > yaw_max_z - 0.008f)
                yaw_cloud->points.push_back(p);
            yaw_cloud->width  = static_cast<uint32_t>(yaw_cloud->points.size());
            yaw_cloud->height = 1; yaw_cloud->is_dense = true;

            if (shape->type == "nought") {
              grasp_yaw = detectNoughtYaw(yaw_cloud, static_cast<float>(arm_off));
              RCLCPP_INFO(node_->get_logger(),
                "Task 3 closeup: nought yaw=%.1f° arm_off=%.0fmm "
                "pos=(%.3f,%.3f) bbox=%.0fmm→%.0fmm",
                grasp_yaw*180.0/M_PI, arm_off*1000.0,
                accurate_cx, accurate_cy,
                close_bbox*1000.0, snapped_close*1000.0);
            } else {
              // Cross: PCA dominant arm direction (Task 1 exact method)
              float raw_yaw = detectShapeYaw(yaw_cloud);
              grasp_yaw = static_cast<double>(raw_yaw);
              while (grasp_yaw <  0.0)       grasp_yaw += M_PI / 2.0;
              while (grasp_yaw >= M_PI / 2.0) grasp_yaw -= M_PI / 2.0;
              RCLCPP_INFO(node_->get_logger(),
                "Task 3 closeup: cross yaw=%.1f° arm_off=%.0fmm "
                "pos=(%.3f,%.3f) bbox=%.0fmm→%.0fmm",
                grasp_yaw*180.0/M_PI, arm_off*1000.0,
                accurate_cx, accurate_cy,
                close_bbox*1000.0, snapped_close*1000.0);
            }
          }
        }
      }
    }

    // ── Phase 5c: Compute exact grasp point ──────────────────────────────────
    //   grasp_point = centroid + arm_off × [cos(grasp_yaw), sin(grasp_yaw)]
    //   pick_yaw: nought → grasp_yaw + π/4  (radial pinch, matches Task 1)
    //             cross  → grasp_yaw + 3π/4 (transverse pinch, matches Task 1)
    geometry_msgs::msg::PointStamped grasp_pt;
    grasp_pt.header.frame_id = "panda_link0";
    grasp_pt.point.x = accurate_cx + arm_off * std::cos(grasp_yaw);
    grasp_pt.point.y = accurate_cy + arm_off * std::sin(grasp_yaw);
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

    RCLCPP_INFO(node_->get_logger(),
      "Task 3: eef_link = '%s'",
      arm_group_->getEndEffectorLink().c_str());

    if (pickObject(grasp_pt, pick_yaw)) {
      // Lift straight up first (Task 1 exact method) before going to basket.
      // This avoids MoveIt spinning the wrist during transit.
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      arm_group_->setStartStateToCurrentState();
      {
        auto cur_pose = arm_group_->getCurrentPose().pose;
        const double safe_z = std::max(cur_pose.position.z + 0.40, 0.65);
        geometry_msgs::msg::Pose lift_pose =
          makeDownwardPose(cur_pose.position.x, cur_pose.position.y,
                           safe_z, pick_yaw);
        if (!moveArmToPose(lift_pose)) {
          RCLCPP_WARN(node_->get_logger(),
            "Task 3: vertical lift failed — attempting place anyway");
        }
      }

      // Place into basket.
      //   Cross: always place with arm at 0° (along X). EEF place_yaw = 3π/4.
      //          Pass basket_x - 0.08 so placeObject's +0.08 lands at basket centre.
      //   Nought: rotationally symmetric — keep same direction as pick.
      geometry_msgs::msg::PointStamped basket_pt;
      basket_pt.header.frame_id = "panda_link0";
      double place_yaw = pick_yaw;
      if (shape->type == "cross") {
        place_yaw = 3.0 * M_PI / 4.0;
        basket_pt.point.x = basket_x - 0.08;  // placeObject adds +0.08 → net = basket_x
        basket_pt.point.y = basket_y;
      } else {
        // Nought: rotationally symmetric, keep same pick direction
        basket_pt.point.x = basket_x - 0.08;  // placeObject adds +0.08 → net = basket_x
        basket_pt.point.y = basket_y;
      }
      basket_pt.point.z = GROUND_Z;
      if (!placeObject(basket_pt, place_yaw))
        RCLCPP_WARN(node_->get_logger(), "Task 3: place failed");
      pick_success = true;
      break;
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Task 3: pick failed for %s at (%.3f,%.3f) — trying next candidate",
        shape->type.c_str(), accurate_cx, accurate_cy);
    }
  }

  if (!pick_success)
    RCLCPP_WARN(node_->get_logger(), "Task 3: all pick attempts failed");

  // ── STEP 7: Cleanup ──────────────────────────────────────────────────────
  removeCollisionObject("t3_bsk_N"); removeCollisionObject("t3_bsk_S");
  removeCollisionObject("t3_bsk_E"); removeCollisionObject("t3_bsk_W");
  cleanupObstacles();
  moveArmToNamedTarget("ready");
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Complete =====");
}
