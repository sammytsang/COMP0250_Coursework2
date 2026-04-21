#include <cw2_class.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <cmath>

struct ShapeClassification {
  std::string type;       // "nought", "cross", or "unknown"
  float centre_ratio;
  int   total_pts;
};

// cloud must already be in the base/world frame
static ShapeClassification
classifyShape(PointCPtr cloud, double cx, double cy)
{
  ShapeClassification result{"unknown", 0.0f, 0};
  if (!cloud || cloud->empty()) return result;

  // Step 1: Crop ±150mm around known shape XY
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(cx - 0.15f, cy - 0.15f, -10.0f, 1.0f));
  crop.setMax(Eigen::Vector4f(cx + 0.15f, cy + 0.15f,  10.0f, 1.0f));
  PointCPtr cropped(new PointC);
  crop.filter(*cropped);
  if (cropped->empty()) return result;

  // Step 2: Keep top 30mm (ground removal, robust to 50mm noise)
  float max_z = -1e6f;
  for (const auto & pt : *cropped) max_z = std::max(max_z, pt.z);
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cropped);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(max_z - 0.03f, max_z + 0.01f);
  PointCPtr shape_cloud(new PointC);
  pass.filter(*shape_cloud);
  if (shape_cloud->empty()) return result;

  // Step 3: Centre void test (rotation-invariant)
  //   Nought: inner void 120×120mm → ~0% centre density
  //   Cross:  centre block 40×40mm → ~22% centre density
  const float centre_half = 0.03f;  // ±30mm
  int centre_count = 0;
  for (const auto & pt : *shape_cloud) {
    if (std::abs(pt.x - static_cast<float>(cx)) < centre_half &&
        std::abs(pt.y - static_cast<float>(cy)) < centre_half)
      centre_count++;
  }

  const int   total   = static_cast<int>(shape_cloud->size());
  const float c_ratio = (total > 0) ? static_cast<float>(centre_count) / total : 0.0f;

  result.centre_ratio = c_ratio;
  result.total_pts    = total;
  result.type         = (c_ratio > 0.15f) ? "cross" : "nought";
  return result;
}

void cw2::t2_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 2 Started =====");

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

    auto cls = ::classifyShape(cloud_base, pt.point.x, pt.point.y);
    RCLCPP_INFO(node_->get_logger(),
      "Task 2: shape at (%.3f, %.3f) → '%s'  (centre_ratio=%.2f, pts=%d)",
      pt.point.x, pt.point.y, cls.type.c_str(), cls.centre_ratio, cls.total_pts);
    return cls.type;
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
