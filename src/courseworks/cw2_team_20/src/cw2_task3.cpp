#include <cw2_class.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{
struct DetectedShape
{
  Eigen::Vector4f centroid;
  std::string type;
  double yaw;
};
}  // namespace

void cw2::t3_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> /*request*/,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Started =====");

  response->total_num_shapes = 0;
  response->num_most_common_shape = 0;
  response->most_common_shape_vector.clear();

  // --- STEP 1: Scan workspace ---
  PointCPtr combined_cloud(new PointC);
  const std::vector<std::pair<double, double>> scan_positions = {
    {0.3, 0.3},
    {0.3, -0.3},
    {-0.3, 0.3},
    {-0.3, -0.3}
  };

  for (const auto & scan_xy : scan_positions) {
    const double sx = scan_xy.first;
    const double sy = scan_xy.second;

    const geometry_msgs::msg::Pose scan_pose = makeDownwardPose(sx, sy, ARM_OVERHEAD_Z);
    if (!moveArmToPose(scan_pose)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Task 3: failed moving to scan pose (%.3f, %.3f), continuing",
        sx, sy);
    }

    PointCPtr fresh_cloud = waitForFreshCloud(3000);
    if (!fresh_cloud || fresh_cloud->empty()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Task 3: no cloud captured at scan pose (%.3f, %.3f)",
        sx, sy);
      continue;
    }

    PointCPtr base_cloud = transformCloudToBaseFrame(fresh_cloud);
    if (!base_cloud || base_cloud->empty()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Task 3: cloud transform failed at scan pose (%.3f, %.3f)",
        sx, sy);
      continue;
    }

    *combined_cloud += *base_cloud;
  }

  if (combined_cloud->empty()) {
    RCLCPP_WARN(node_->get_logger(), "Task 3: combined scan cloud is empty");
    moveArmToNamedTarget("ready");
    RCLCPP_INFO(node_->get_logger(), "===== Task 3 Complete =====");
    return;
  }

  // --- STEP 2: Detect shapes ---
  PointCPtr non_dark_cloud =
    filterByColourRange(combined_cloud, 40.0f, 255.0f, 0.0f, 255.0f, 0.0f, 255.0f);
  PointCPtr shape_no_ground = removeGroundPlane(non_dark_cloud, 0.012f);
  PointCPtr shape_ds = voxelDownsample(shape_no_ground, 0.005f);
  std::vector<PointCPtr> shape_clusters = euclideanCluster(shape_ds, 0.03f, 50, 10000);

  std::vector<DetectedShape> detected_shapes;
  detected_shapes.reserve(shape_clusters.size());

  for (const auto & cluster : shape_clusters) {
    if (!cluster || cluster->empty()) {
      continue;
    }

    const Eigen::Vector4f centroid = getCloudCentroid(cluster);
    const std::string type = classifyShape(cluster);
    const double yaw = detectShapeYaw(cluster);

    if (type != "nought" && type != "cross") {
      continue;
    }

    detected_shapes.push_back({centroid, type, yaw});
    RCLCPP_INFO(
      node_->get_logger(),
      "Task 3: detected %s at (%.3f, %.3f, %.3f), yaw=%.3f",
      type.c_str(), centroid.x(), centroid.y(), centroid.z(), yaw);
  }

  // --- STEP 3: Detect obstacles ---
  std::vector<std::string> obstacle_ids;
  PointCPtr dark_cloud =
    filterByColourRange(combined_cloud, 0.0f, 40.0f, 0.0f, 40.0f, 0.0f, 40.0f);
  std::vector<PointCPtr> obstacle_clusters = euclideanCluster(dark_cloud, 0.03f, 50, 10000);
  obstacle_ids.reserve(obstacle_clusters.size());

  for (size_t i = 0; i < obstacle_clusters.size(); ++i) {
    const auto & cluster = obstacle_clusters[i];
    if (!cluster || cluster->empty()) {
      continue;
    }

    const Eigen::Vector4f centroid = getCloudCentroid(cluster);
    const std::string id = "t3_obs_" + std::to_string(i);
    addBoxCollisionObject(id, centroid.x(), centroid.y(), 0.10, 0.10, 0.10, 0.20);
    obstacle_ids.push_back(id);
  }

  auto cleanupObstacles = [&]() {
      for (const auto & id : obstacle_ids) {
        removeCollisionObject(id);
      }
    };

  // --- STEP 4: Count shapes ---
  int nought_count = 0;
  int cross_count = 0;
  for (const auto & shape : detected_shapes) {
    if (shape.type == "nought") {
      ++nought_count;
    } else if (shape.type == "cross") {
      ++cross_count;
    }
  }

  const int total_num_shapes = nought_count + cross_count;
  const std::string most_common_type = (nought_count >= cross_count) ? "nought" : "cross";
  const int num_most_common_shape = (nought_count >= cross_count) ? nought_count : cross_count;

  response->total_num_shapes = total_num_shapes;
  response->num_most_common_shape = num_most_common_shape;
  response->most_common_shape_vector.clear();

  RCLCPP_INFO(
    node_->get_logger(),
    "Task 3: nought=%d cross=%d total=%d most_common=%s(%d)",
    nought_count, cross_count, total_num_shapes, most_common_type.c_str(), num_most_common_shape);

  // --- STEP 5: Pick and Place ---
  const auto target_it = std::find_if(
    detected_shapes.begin(), detected_shapes.end(),
    [&](const DetectedShape & shape) {return shape.type == most_common_type;});

  if (target_it == detected_shapes.end()) {
    RCLCPP_WARN(node_->get_logger(), "Task 3: no target shape found for pick-and-place");
  } else {
    geometry_msgs::msg::PointStamped pick_pt;
    pick_pt.header.frame_id = "panda_link0";
    pick_pt.point.x = target_it->centroid.x() + ((target_it->type == "nought") ? 0.08 : 0.0);
    pick_pt.point.y = target_it->centroid.y();
    pick_pt.point.z = target_it->centroid.z();

    const double pick_yaw = (target_it->type == "nought") ? 0.0 : (M_PI / 2.0);
    const bool pick_ok = pickObject(pick_pt, pick_yaw);

    geometry_msgs::msg::Point basket_point;
    basket_point.x = -0.41;
    basket_point.y = -0.36;
    basket_point.z = GROUND_Z;

    geometry_msgs::msg::PointStamped basket_pt;
    basket_pt.header.frame_id = "panda_link0";
    basket_pt.point.x = basket_point.x;
    basket_pt.point.y = basket_point.y;
    basket_pt.point.z = basket_point.z;

    if (!pick_ok) {
      RCLCPP_WARN(node_->get_logger(), "Task 3: pick failed");
    } else {
      const bool place_ok = placeObject(basket_pt, pick_yaw);
      if (!place_ok) {
        RCLCPP_WARN(node_->get_logger(), "Task 3: place failed");
      }
    }
  }

  // --- STEP 6: Cleanup ---
  cleanupObstacles();
  moveArmToNamedTarget("ready");

  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Complete =====");
}
