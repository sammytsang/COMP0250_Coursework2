#include <cw2_class.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <mutex>
#include <atomic>

static sensor_msgs::msg::PointCloud2::SharedPtr
waitForFreshCloud(rclcpp::Node::SharedPtr node,
                  const std::string & topic,
                  int timeout_ms = 5000)
{
  sensor_msgs::msg::PointCloud2::SharedPtr result;
  std::mutex mtx;
  std::atomic<bool> received{false};

  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, rclcpp::SensorDataQoS(),
    [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      if (!received.load()) {
        std::lock_guard<std::mutex> lock(mtx);
        result = msg;
        received.store(true);
      }
    });

  const auto deadline =
    node->now() + rclcpp::Duration::from_nanoseconds(
      static_cast<int64_t>(timeout_ms) * 1000000LL);

  rclcpp::Rate rate(50);
  while (rclcpp::ok() && !received.load() && node->now() < deadline) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  return result;
}

static std::string
classifyShape(const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
              double cx, double cy)
{
  if (!cloud_msg) return "unknown";

  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  if (cloud->empty()) return "unknown";

  // Step 1: Crop to a box around the shape (200x200mm = 0.2x0.2m + margin)
  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(cx - 0.15f, cy - 0.15f, -10.0f, 1.0f));
  crop.setMax(Eigen::Vector4f(cx + 0.15f, cy + 0.15f,  10.0f, 1.0f));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
  crop.filter(*cropped);

  if (cropped->empty()) return "unknown";

  // Step 2: Remove ground plane (keep only top 30mm of points)
  float max_z = -1e6f;
  for (const auto & pt : *cropped) max_z = std::max(max_z, pt.z);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cropped);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(max_z - 0.03f, max_z + 0.01f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr shape_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pass.filter(*shape_cloud);

  if (shape_cloud->empty()) return "unknown";

  // Step 3: Count points in centre region (inner 60x60mm = 0.06x0.06m)
  // Nought = hollow -> few centre points
  // Cross  = solid  -> many centre points
  int centre_count = 0;
  int total_count = static_cast<int>(shape_cloud->size());
  const float centre_half = 0.03f;  // 60mm / 2

  for (const auto & pt : *shape_cloud) {
    if (std::abs(pt.x - cx) < centre_half &&
        std::abs(pt.y - cy) < centre_half) {
      centre_count++;
    }
  }

  const float centre_ratio =
    (total_count > 0) ? static_cast<float>(centre_count) / total_count : 0.0f;

  // If more than 15% of points are in the centre -> cross
  // Otherwise -> nought (hollow centre)
  const std::string result = (centre_ratio > 0.15f) ? "cross" : "nought";
  return result;
}

void cw2::t2_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 2 Started =====");

  if (request->ref_object_points.size() < 2) {
    RCLCPP_ERROR(node_->get_logger(),
      "Task 2: expected 2 reference points, got %zu. Defaulting to 1.",
      request->ref_object_points.size());
    response->mystery_object_num = 1;
    return;
  }

  const std::string cloud_topic = "/r200/camera/depth_registered/points";

  // Height above shape to hover for observation (metres)
  static constexpr double OBSERVE_HEIGHT = 0.5;

  // Helper lambda: move arm above a point and classify the shape below
  auto observeShape = [&](const geometry_msgs::msg::PointStamped & pt) -> std::string
  {
    // Move arm above the shape
    geometry_msgs::msg::Pose observe_pose =
      makeDownwardPose(pt.point.x, pt.point.y,
                       pt.point.z + OBSERVE_HEIGHT, 0.0);
    bool moved = moveArmToPose(observe_pose);
    if (!moved) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 2: failed to move above (%.3f, %.3f), trying anyway",
        pt.point.x, pt.point.y);
    }

    // Wait for a fresh point cloud
    auto cloud = ::waitForFreshCloud(node_, cloud_topic, 5000);
    if (!cloud) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 2: no point cloud received for (%.3f, %.3f)",
        pt.point.x, pt.point.y);
      return "unknown";
    }

    // Classify the shape
    std::string shape = ::classifyShape(cloud, pt.point.x, pt.point.y);
    RCLCPP_INFO(node_->get_logger(),
      "Task 2: shape at (%.3f, %.3f) classified as '%s'",
      pt.point.x, pt.point.y, shape.c_str());
    return shape;
  };

  // Observe all three shapes
  std::string ref1_type    = observeShape(request->ref_object_points[0]);
  std::string ref2_type    = observeShape(request->ref_object_points[1]);
  std::string mystery_type = observeShape(request->mystery_object_point);

  RCLCPP_INFO(node_->get_logger(),
    "Task 2: Ref1='%s' | Ref2='%s' | Mystery='%s'",
    ref1_type.c_str(), ref2_type.c_str(), mystery_type.c_str());

  // Match mystery to reference
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
    "===== Task 2 Complete: answer=%ld =====",
    response->mystery_object_num);
}
