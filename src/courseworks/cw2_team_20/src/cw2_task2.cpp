#include <cw2_class.h>

void cw2::t2_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 2 Started =====");

  // FIX 2: Stabilise arm before any motion to clear lingering MoveIt goals
  arm_group_->stop();
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  arm_group_->setStartStateToCurrentState();

  if (request->ref_object_points.size() < 2) {
    RCLCPP_ERROR(node_->get_logger(),
      "Task 2: expected 2 ref points, got %zu. Defaulting to 1.",
      request->ref_object_points.size());
    response->mystery_object_num = 1;
    return;
  }

  static constexpr double OBSERVE_HEIGHT = 0.5;

  // Move above shape, grab fresh cloud, transform to base frame, classify
  auto observeShape = [&](const geometry_msgs::msg::PointStamped & pt) -> std::string
  {
    geometry_msgs::msg::Pose observe_pose =
      makeDownwardPose(pt.point.x, pt.point.y, pt.point.z + OBSERVE_HEIGHT, 0.0);
    bool moved = moveArmToPose(observe_pose);
    if (!moved)
      RCLCPP_WARN(node_->get_logger(),
        "Task 2: failed to move above (%.3f, %.3f), trying anyway",
        pt.point.x, pt.point.y);

    // Use the class's waitForFreshCloud — safe from service callback because
    // cloud_callback runs in a Reentrant group via MultiThreadedExecutor
    PointCPtr raw = waitForFreshCloud(5000);
    if (!raw || raw->empty()) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 2: no cloud for (%.3f, %.3f)", pt.point.x, pt.point.y);
      return "unknown";
    }

    PointCPtr cloud_base = transformCloudToBaseFrame(raw);

    // FIX 1: Crop to ±0.15m around target, isolate top face, use member classifyShape
    PointCPtr cropped = cropBoxAroundPoint(
      cloud_base, pt.point.x, pt.point.y, pt.point.z, 0.15);
    if (!cropped || cropped->empty()) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 2: empty crop for (%.3f, %.3f)", pt.point.x, pt.point.y);
      moveArmToNamedTarget("ready");
      return "unknown";
    }

    float max_z = -1e6f;
    for (const auto & p : cropped->points) max_z = std::max(max_z, p.z);
    PointCPtr top_face(new PointC);
    for (const auto & p : cropped->points)
      if (p.z > max_z - 0.012f) top_face->points.push_back(p);
    top_face->width  = static_cast<uint32_t>(top_face->points.size());
    top_face->height = 1; top_face->is_dense = true;
    if (top_face->size() < 30) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 2: too few top-face points (%zu) for (%.3f, %.3f)",
        top_face->size(), pt.point.x, pt.point.y);
      moveArmToNamedTarget("ready");
      return "unknown";
    }

    std::string shape_type = classifyShape(top_face);
    RCLCPP_INFO(node_->get_logger(),
      "Task 2: shape at (%.3f, %.3f) → '%s'  (top_face_pts=%zu)",
      pt.point.x, pt.point.y, shape_type.c_str(), top_face->size());

    // FIX 3: Retract to ready between observations to avoid joint-limit violations
    moveArmToNamedTarget("ready");
    return shape_type;
  };

  std::string ref1_type    = observeShape(request->ref_object_points[0]);
  std::string ref2_type    = observeShape(request->ref_object_points[1]);
  std::string mystery_type = observeShape(request->mystery_object_point);

  moveArmToNamedTarget("ready");

  RCLCPP_INFO(node_->get_logger(),
    "Task 2: Ref1='%s' | Ref2='%s' | Mystery='%s'",
    ref1_type.c_str(), ref2_type.c_str(), mystery_type.c_str());

  if (mystery_type != "unknown" && mystery_type == ref1_type) {
    response->mystery_object_num = 1;
  } else if (mystery_type != "unknown" && mystery_type == ref2_type) {
    response->mystery_object_num = 2;
  } else {
    RCLCPP_WARN(node_->get_logger(),
      "Task 2: could not match mystery shape, defaulting to 1");
    response->mystery_object_num = 1;
  }

  RCLCPP_INFO(node_->get_logger(),
    "===== Task 2 Complete: answer=%ld =====", response->mystery_object_num);
}
