#include <cw2_class.h>
#include <pcl/common/centroid.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{

static double estimateBboxDim(PointCPtr cloud)
{
  float xmin =  1e6f, xmax = -1e6f;
  float ymin =  1e6f, ymax = -1e6f;
  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
    xmin = std::min(xmin, pt.x); xmax = std::max(xmax, pt.x);
    ymin = std::min(ymin, pt.y); ymax = std::max(ymax, pt.y);
  }
  if (xmin >= xmax || ymin >= ymax) return 0.100;
  return static_cast<double>((xmax - xmin + ymax - ymin) * 0.5f);
}

static double snapArmOff(const std::string & type, double measured_bbox,
                          double & snapped_out)
{
  if (type == "nought") {
    constexpr double tiers[]    = {0.100, 0.150, 0.200};
    constexpr double arm_offs[] = {0.040, 0.060, 0.080};
    int best = 0;
    double best_d = std::abs(measured_bbox - tiers[0]);
    for (int i = 1; i < 3; ++i) {
      double d = std::abs(measured_bbox - tiers[i]);
      if (d < best_d) { best_d = d; best = i; }
    }
    snapped_out = tiers[best];
    return arm_offs[best];
  } else {
    constexpr double tiers[]    = {0.060, 0.090, 0.120};
    constexpr double arm_offs[] = {0.040, 0.055, 0.060};
    int best = 0;
    double best_d = std::abs(measured_bbox - tiers[0]);
    for (int i = 1; i < 3; ++i) {
      double d = std::abs(measured_bbox - tiers[i]);
      if (d < best_d) { best_d = d; best = i; }
    }
    snapped_out = tiers[best];
    return arm_offs[best];
  }
}

// HOLE DETECTION classifier: nought has hole → centre points are green tile.
static std::string classifyByHole(PointCPtr cloud, double arm_off)
{
  if (!cloud || cloud->size() < 10) return "unknown";

  Eigen::Vector4f cen;
  pcl::compute3DCentroid(*cloud, cen);
  const float cx = cen[0], cy = cen[1];

  const float r = static_cast<float>(arm_off * 0.40);
  const float r2 = r * r;

  int total = 0, green_pts = 0;
  for (const auto & pt : cloud->points) {
    const float dx = pt.x - cx, dy = pt.y - cy;
    if (dx*dx + dy*dy > r2) continue;
    ++total;
    const float R = pt.r / 255.0f;
    const float G = pt.g / 255.0f;
    const float B = pt.b / 255.0f;
    if (G > R && G > B && G > 0.35f) ++green_pts;
  }

  if (total < 3) return "nought";

  const float green_ratio = static_cast<float>(green_pts) /
                             static_cast<float>(total);
  return (green_ratio > 0.30f) ? "nought" : "cross";
}

static PointCPtr filterShapeColour(PointCPtr cloud)
{
  PointCPtr out(new PointC);
  for (const auto & pt : cloud->points) {
    const float r = pt.r / 255.0f;
    const float b = pt.b / 255.0f;
    if (r > 0.50f || b > 0.50f) out->points.push_back(pt);
  }
  out->width  = static_cast<uint32_t>(out->points.size());
  out->height = 1; out->is_dense = true;
  return out;
}

struct DetectedShape {
  Eigen::Vector4f centroid;
  std::string     type;
  double          bbox_dim;
  double          arm_off;
};

}  // namespace


void cw2::t3_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> /*req*/,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Started =====");
  response->total_num_shapes      = 0;
  response->num_most_common_shape = 0;
  response->most_common_shape_vector.clear();

  arm_group_->stop();
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  for (int i = 0; i < 30; ++i) {
    arm_group_->setStartStateToCurrentState();
    if (moveArmToNamedTarget("ready")) break;
    RCLCPP_WARN(node_->get_logger(),
      "Task3: waiting for ready (attempt %d/30)...", i + 1);
    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  // ── PHASE 1 — FULL MAT SCAN: 5×5 grid at z=0.65 ─────────────────────────
  PointCPtr combined(new PointC);
  const std::vector<double> xs = {-0.55, -0.30, 0.00, 0.30, 0.55};
  const std::vector<double> ys = {-0.50, -0.25, 0.00, 0.25, 0.50};

  for (double sx : xs) {
    for (double sy : ys) {
      arm_group_->setStartStateToCurrentState();
      if (!moveArmToPose(makeDownwardPose(sx, sy, 0.65, 0.0))) {
        RCLCPP_WARN(node_->get_logger(),
          "Task3: scan move failed (%.2f,%.2f) — skipping", sx, sy);
        continue;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(800));
      PointCPtr raw = waitForFreshCloud(5000);
      if (!raw || raw->empty()) continue;
      PointCPtr base = transformCloudToBaseFrame(raw);
      if (base && !base->empty()) *combined += *base;
    }
  }
  moveArmToNamedTarget("ready");

  if (combined->empty()) {
    RCLCPP_WARN(node_->get_logger(), "Task3: no cloud data — aborting");
    return;
  }

  {
    PointCPtr g(new PointC);
    for (const auto & pt : combined->points)
      if (std::isfinite(pt.z) && pt.z < 0.15f) g->points.push_back(pt);
    g->width = static_cast<uint32_t>(g->points.size());
    g->height = 1; g->is_dense = true;
    combined = g;
  }
  combined = voxelDownsample(combined, 0.005f);
  RCLCPP_INFO(node_->get_logger(),
    "Task3: combined cloud %zu pts", combined->size());

  // ── PHASE 2 — OBSTACLE DETECTION ────────────────────────────────────────
  std::vector<std::string> obstacle_ids;
  {
    PointCPtr dark = filterByColourRange(combined,
                                         0.00f, 0.20f,
                                         0.00f, 0.20f,
                                         0.00f, 0.20f);
    PointCPtr dark_up(new PointC);
    for (const auto & pt : dark->points)
      if (pt.z > 0.010f) dark_up->points.push_back(pt);
    dark_up->width  = static_cast<uint32_t>(dark_up->points.size());
    dark_up->height = 1; dark_up->is_dense = true;

    auto obs_clusters = euclideanCluster(dark_up, 0.03f, 30, 20000);
    int n = 0;
    for (auto & c : obs_clusters) {
      Eigen::Vector4f cen = getCloudCentroid(c);
      float xmin=1e6f,xmax=-1e6f,ymin=1e6f,ymax=-1e6f;
      for (const auto & pt : c->points) {
        xmin=std::min(xmin,pt.x); xmax=std::max(xmax,pt.x);
        ymin=std::min(ymin,pt.y); ymax=std::max(ymax,pt.y);
      }
      const double osx = (xmax - xmin) + 0.10;
      const double osy = (ymax - ymin) + 0.10;
      const std::string id = "t3_obs_" + std::to_string(n++);
      addBoxCollisionObject(id, cen[0], cen[1], GROUND_Z + 0.15, osx, osy, 0.35);
      obstacle_ids.push_back(id);
      RCLCPP_INFO(node_->get_logger(),
        "Task3 obstacle '%s': pos=(%.3f,%.3f) box=(%.3f x %.3f)",
        id.c_str(), cen[0], cen[1], osx, osy);
    }
    RCLCPP_INFO(node_->get_logger(),
      "Task3: %zu obstacles detected and registered", obstacle_ids.size());
  }
  auto cleanupObstacles = [&]() {
    for (const auto & id : obstacle_ids) removeCollisionObject(id);
  };

  // ── PHASE 3 — BASKET DETECTION ──────────────────────────────────────────
  double basket_x = 0.0, basket_y = 0.0;
  {
    const std::vector<std::pair<double,double>> candidates = {
      {-0.41, -0.36}, {-0.41, +0.36}
    };
    int best = -1;
    for (const auto & c : candidates) {
      int cnt = 0;
      for (const auto & pt : combined->points)
        if (std::abs(pt.x - c.first) < 0.22 &&
            std::abs(pt.y - c.second) < 0.22) ++cnt;
      if (cnt > best) { best = cnt; basket_x = c.first; basket_y = c.second; }
    }
    RCLCPP_INFO(node_->get_logger(),
      "Task3 basket: (%.3f, %.3f)", basket_x, basket_y);
    addBoxCollisionObject("t3_bsk_N",basket_x+0.19,basket_y,        GROUND_Z+0.025,0.02,0.37,0.06);
    addBoxCollisionObject("t3_bsk_S",basket_x-0.19,basket_y,        GROUND_Z+0.025,0.02,0.37,0.06);
    addBoxCollisionObject("t3_bsk_E",basket_x,      basket_y+0.19,  GROUND_Z+0.025,0.37,0.02,0.06);
    addBoxCollisionObject("t3_bsk_W",basket_x,      basket_y-0.19,  GROUND_Z+0.025,0.37,0.02,0.06);
  }

  // ── PHASE 4 — COARSE SHAPE POSITIONS ────────────────────────────────────
  std::vector<std::tuple<double,double,double>> coarse_positions;
  {
    PointCPtr shape_col = filterShapeColour(combined);
    PointCPtr shape_up(new PointC);
    for (const auto & pt : shape_col->points)
      if (pt.z > 0.010f) shape_up->points.push_back(pt);
    shape_up->width  = static_cast<uint32_t>(shape_up->points.size());
    shape_up->height = 1; shape_up->is_dense = true;

    auto clusters = euclideanCluster(shape_up, 0.03f, 30, 30000);
    for (auto & c : clusters) {
      if (c->size() < 20) continue;
      const double bbox = estimateBboxDim(c);
      if (bbox < 0.040 || bbox > 0.28) continue;

      Eigen::Vector4f cen = getCloudCentroid(c);
      if (std::hypot(cen[0], cen[1]) < 0.22) continue;
      if (std::hypot(cen[0]-basket_x, cen[1]-basket_y) < 0.20) continue;

      float max_z = -1e6f;
      for (const auto & pt : c->points) max_z = std::max(max_z, pt.z);
      if (max_z < 0.020f || max_z > 0.090f) continue;

      coarse_positions.emplace_back(cen[0], cen[1], bbox);
      RCLCPP_INFO(node_->get_logger(),
        "Task3 coarse: (%.3f, %.3f) bbox=%.0fmm",
        cen[0], cen[1], bbox * 1000.0);
    }
  }
  RCLCPP_INFO(node_->get_logger(),
    "Task3: %zu coarse positions", coarse_positions.size());

  // ── PHASE 5 — CLOSE-UP SCAN + HOLE DETECTION ─────────────────────────────
  std::vector<DetectedShape> detected;

  for (const auto & p : coarse_positions) {
    const double px         = std::get<0>(p);
    const double py         = std::get<1>(p);
    const double coarse_bbox = std::get<2>(p);

    double closeup_h;
    if      (coarse_bbox >= 0.12) closeup_h = 0.50;
    else if (coarse_bbox >= 0.07) closeup_h = 0.40;
    else                          closeup_h = 0.35;

    moveArmToNamedTarget("ready");
    if (!moveArmToPose(makeDownwardPose(px, py, closeup_h, 0.0))) {
      RCLCPP_WARN(node_->get_logger(),
        "Task3: close-up move failed at (%.3f,%.3f)", px, py);
      moveArmToNamedTarget("ready");
      continue;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(600));

    PointCPtr raw = waitForFreshCloud(5000);
    if (!raw || raw->empty()) { moveArmToNamedTarget("ready"); continue; }

    PointCPtr base      = transformCloudToBaseFrame(raw);
    PointCPtr cropped   = cropBoxAroundPoint(base, px, py, 0.0, 0.15);
    PointCPtr no_ground = removeGroundPlane(cropped, 0.012f);
    if (!no_ground || no_ground->empty()) {
      moveArmToNamedTarget("ready"); continue;
    }

    float max_z = -1e6f;
    for (const auto & pt : no_ground->points) max_z = std::max(max_z, pt.z);
    PointCPtr top_face(new PointC);
    for (const auto & pt : no_ground->points)
      if (pt.z > max_z - 0.015f) top_face->points.push_back(pt);
    top_face->width  = static_cast<uint32_t>(top_face->points.size());
    top_face->height = 1; top_face->is_dense = true;

    if (top_face->size() < 5) {
      RCLCPP_WARN(node_->get_logger(),
        "Task3: too few top-face pts (%zu) at (%.3f,%.3f)",
        top_face->size(), px, py);
      moveArmToNamedTarget("ready"); continue;
    }

    const double bbox = estimateBboxDim(top_face);
    double snapped_bbox = bbox;

    double arm_off_nought = 0.0, arm_off_cross = 0.0;
    {
      double tmp;
      arm_off_nought = snapArmOff("nought", bbox, tmp);
      arm_off_cross  = snapArmOff("cross",  bbox, tmp);
    }

    std::string type;
    {
      const double probe = arm_off_nought > 0 ? arm_off_nought : arm_off_cross;
      type = classifyByHole(cropped, probe);
    }

    if (type != "nought" && type != "cross") {
      moveArmToNamedTarget("ready"); continue;
    }

    const double arm_off = snapArmOff(type, bbox, snapped_bbox);
    Eigen::Vector4f cen  = getCloudCentroid(top_face);

    detected.push_back(DetectedShape{cen, type, snapped_bbox, arm_off});
    RCLCPP_INFO(node_->get_logger(),
      "Task3 confirmed: %s at (%.3f,%.3f) bbox=%.0fmm arm_off=%.0fmm h=%.2fm",
      type.c_str(), cen[0], cen[1],
      snapped_bbox * 1000.0, arm_off * 1000.0, closeup_h);

    moveArmToNamedTarget("ready");
  }

  detected.erase(
    std::remove_if(detected.begin(), detected.end(),
      [&](const DetectedShape & d) {
        return std::hypot(d.centroid[0]-basket_x,
                          d.centroid[1]-basket_y) < 0.20;
      }),
    detected.end());

  int n_nought = 0, n_cross = 0;
  for (const auto & d : detected) {
    if      (d.type == "nought") ++n_nought;
    else if (d.type == "cross")  ++n_cross;
  }
  const int total      = n_nought + n_cross;
  const std::string common = (n_nought >= n_cross) ? "nought" : "cross";
  const int num_common = std::max(n_nought, n_cross);

  response->total_num_shapes      = total;
  response->num_most_common_shape = num_common;

  RCLCPP_INFO(node_->get_logger(), "========================================");
  RCLCPP_INFO(node_->get_logger(), "Task3 SCAN SUMMARY:");
  RCLCPP_INFO(node_->get_logger(), "  Total shapes    : %d", total);
  RCLCPP_INFO(node_->get_logger(), "  Noughts         : %d", n_nought);
  RCLCPP_INFO(node_->get_logger(), "  Crosses         : %d", n_cross);
  RCLCPP_INFO(node_->get_logger(), "  Obstacles       : %zu", obstacle_ids.size());
  RCLCPP_INFO(node_->get_logger(), "  Most common     : %s (%d)",
    common.c_str(), num_common);
  for (const auto & d : detected) {
    RCLCPP_INFO(node_->get_logger(),
      "    %s at (%.3f,%.3f) bbox=%.0fmm arm_off=%.0fmm",
      d.type.c_str(), d.centroid[0], d.centroid[1],
      d.bbox_dim*1000.0, d.arm_off*1000.0);
  }
  RCLCPP_INFO(node_->get_logger(), "========================================");

  // ── PHASE 6 — PICK AND PLACE ────────────────────────────────────────────
  std::vector<DetectedShape *> candidates;
  for (auto & d : detected)
    if (d.type == common) candidates.push_back(&d);

  std::sort(candidates.begin(), candidates.end(),
    [](const DetectedShape * a, const DetectedShape * b) {
      return std::hypot(a->centroid[0], a->centroid[1])
           < std::hypot(b->centroid[0], b->centroid[1]);
    });

  bool pick_success = false;

  for (DetectedShape * shape : candidates) {
    const double cx      = shape->centroid[0];
    const double cy      = shape->centroid[1];
    const double arm_off = shape->arm_off;
    const std::string & type = shape->type;

    if (std::hypot(cx, cy) < 0.22) continue;

    moveArmToNamedTarget("ready");
    if (!moveArmToPose(makeDownwardPose(cx, cy, 0.50, 0.0))) {
      moveArmToNamedTarget("ready"); continue;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    PointCPtr raw = waitForFreshCloud(5000);
    if (!raw || raw->empty()) { moveArmToNamedTarget("ready"); continue; }
    PointCPtr base      = transformCloudToBaseFrame(raw);
    PointCPtr cropped   = cropBoxAroundPoint(base, cx, cy, 0.0, 0.15);
    PointCPtr no_ground = removeGroundPlane(cropped, 0.012f);

    double grasp_yaw = 0.0;
    double pick_cx = cx, pick_cy = cy;

    if (no_ground && no_ground->size() >= 10) {
      if (type == "nought") {
        Eigen::Vector4f cen4;
        pcl::compute3DCentroid(*no_ground, cen4);
        pick_cx = cen4[0]; pick_cy = cen4[1];

        const float ARM_OFF  = static_cast<float>(arm_off);
        const float SEARCH_R = static_cast<float>(arm_off * 0.31);
        int best_cnt = -1; double best_theta = 0.0;
        for (int i = 0; i < 90; ++i) {
          const double theta = i * (M_PI / 2.0) / 90.0;
          int cnt = 0;
          for (int k = 0; k < 4; ++k) {
            const double t = theta + k * M_PI / 2.0;
            const float tx = static_cast<float>(pick_cx)
                             + ARM_OFF * static_cast<float>(std::cos(t));
            const float ty = static_cast<float>(pick_cy)
                             + ARM_OFF * static_cast<float>(std::sin(t));
            for (const auto & pt : no_ground->points) {
              float dx = pt.x - tx, dy = pt.y - ty;
              if (dx*dx + dy*dy < SEARCH_R * SEARCH_R) ++cnt;
            }
          }
          if (cnt > best_cnt) { best_cnt = cnt; best_theta = theta; }
        }
        grasp_yaw = best_theta;
        RCLCPP_INFO(node_->get_logger(),
          "Nought yaw: face_normal=%.1fdeg score=%d arm_off=%.0fmm",
          grasp_yaw*180.0/M_PI, best_cnt, arm_off*1000.0);

      } else {
        float raw_yaw = detectShapeYaw(no_ground);
        double yaw = static_cast<double>(raw_yaw);
        while (yaw <  0.0)        yaw += M_PI / 2.0;
        while (yaw >= M_PI / 2.0) yaw -= M_PI / 2.0;
        grasp_yaw = yaw;
        RCLCPP_INFO(node_->get_logger(),
          "Cross yaw: raw=%.1fdeg grasp=%.1fdeg arm_off=%.0fmm",
          raw_yaw*180.0f/static_cast<float>(M_PI),
          grasp_yaw*180.0/M_PI, arm_off*1000.0);
      }
    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Task3: sparse cloud — using grasp_yaw=0");
    }

    geometry_msgs::msg::PointStamped grasp_pt;
    grasp_pt.header.frame_id = "panda_link0";
    grasp_pt.point.x = pick_cx + arm_off * std::cos(grasp_yaw);
    grasp_pt.point.y = pick_cy + arm_off * std::sin(grasp_yaw);
    grasp_pt.point.z = GROUND_Z;

    double pick_yaw;
    if (type == "nought") {
      pick_yaw = grasp_yaw + M_PI / 4.0;
      RCLCPP_INFO(node_->get_logger(),
        "Nought: grasp=(%.3f,%.3f) offset=%.3fm "
        "edge_normal=%.1fdeg finger_world=%.1fdeg pick_yaw=%.1fdeg",
        grasp_pt.point.x, grasp_pt.point.y, arm_off,
        grasp_yaw*180.0/M_PI,
        (pick_yaw-M_PI/4.0)*180.0/M_PI,
        pick_yaw*180.0/M_PI);
    } else {
      pick_yaw = grasp_yaw + 3.0 * M_PI / 4.0;
      RCLCPP_INFO(node_->get_logger(),
        "Cross: grasp=(%.3f,%.3f) offset=%.3fm "
        "across_arm=%.1fdeg finger_world=%.1fdeg pick_yaw=%.1fdeg",
        grasp_pt.point.x, grasp_pt.point.y, arm_off,
        (grasp_yaw+M_PI/2.0)*180.0/M_PI,
        (pick_yaw-M_PI/4.0)*180.0/M_PI,
        pick_yaw*180.0/M_PI);
    }

    if (pickObject(grasp_pt, pick_yaw)) {
      rclcpp::sleep_for(std::chrono::milliseconds(1500));
      arm_group_->setStartStateToCurrentState();
      auto cur = arm_group_->getCurrentPose().pose;
      const double safe_z = std::max(cur.position.z + 0.40, 0.65);
      moveArmToPose(makeDownwardPose(
        cur.position.x, cur.position.y, safe_z, pick_yaw));

      geometry_msgs::msg::PointStamped place_pt;
      place_pt.header.frame_id = "panda_link0";
      place_pt.point.z = GROUND_Z;
      double place_yaw;

      if (type == "cross") {
        place_yaw = 3.0 * M_PI / 4.0;
        place_pt.point.x = basket_x + arm_off;
        place_pt.point.y = basket_y;
      } else {
        place_yaw = pick_yaw;
        place_pt.point.x = basket_x + arm_off * std::cos(grasp_yaw);
        place_pt.point.y = basket_y + arm_off * std::sin(grasp_yaw);
      }

      RCLCPP_INFO(node_->get_logger(),
        "Task3 place EEF at (%.3f,%.3f) place_yaw=%.1fdeg → basket (%.3f,%.3f)",
        place_pt.point.x, place_pt.point.y,
        place_yaw*180.0/M_PI, basket_x, basket_y);

      placeObject(place_pt, place_yaw);
      pick_success = true;
      break;

    } else {
      RCLCPP_WARN(node_->get_logger(),
        "Task3: pick failed for %s at (%.3f,%.3f) — trying next",
        type.c_str(), cx, cy);
    }
  }

  if (!pick_success)
    RCLCPP_WARN(node_->get_logger(), "Task3: all pick attempts failed");

  removeCollisionObject("t3_bsk_N"); removeCollisionObject("t3_bsk_S");
  removeCollisionObject("t3_bsk_E"); removeCollisionObject("t3_bsk_W");
  cleanupObstacles();
  moveArmToNamedTarget("ready");
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Complete =====");
}
