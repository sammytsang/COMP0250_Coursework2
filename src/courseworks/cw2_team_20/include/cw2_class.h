#ifndef CW2_CLASS_H_
#define CW2_CLASS_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/transforms.hpp>

#include <opencv2/opencv.hpp>

#include <cw2_world_spawner/srv/task1_service.hpp>
#include <cw2_world_spawner/srv/task2_service.hpp>
#include <cw2_world_spawner/srv/task3_service.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

struct ShapeInfo {
  Eigen::Vector4f centroid;
  std::string type;    // "nought" or "cross"
  float yaw_angle;     // orientation around Z in radians
};

class cw2
{
public:
  explicit cw2(rclcpp::Node::SharedPtr node);

  // Task callbacks
  void t1_callback(
    std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> req,
    std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> res);

  void t2_callback(
    std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> req,
    std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> res);

  void t3_callback(
    std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> req,
    std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> res);

  void cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // Motion helpers
  bool moveArmToPose(const geometry_msgs::msg::Pose & target_pose);
  bool moveArmToNamedTarget(const std::string & target_name);
  bool setGripper(double width);
  bool pickObject(const geometry_msgs::msg::PointStamped & object_point,
                  double grasp_yaw = 0.0);
  bool placeObject(const geometry_msgs::msg::PointStamped & goal_point);
  geometry_msgs::msg::Pose makeDownwardPose(double x, double y, double z, double yaw = 0.0);

  // Perception helpers
  PointCPtr waitForFreshCloud(int timeout_ms = 3000);
  PointCPtr transformCloudToBaseFrame(PointCPtr cloud);
  PointCPtr cropBoxAroundPoint(PointCPtr cloud, double cx, double cy, double cz, double half_side);
  PointCPtr removeGroundPlane(PointCPtr cloud, float dist_thresh = 0.012f);
  PointCPtr filterByColourRange(PointCPtr cloud,
                                float r_min, float r_max,
                                float g_min, float g_max,
                                float b_min, float b_max);
  PointCPtr voxelDownsample(PointCPtr cloud, float leaf = 0.005f);
  std::vector<PointCPtr> euclideanCluster(PointCPtr cloud,
                                          float tolerance = 0.03f,
                                          int min_size = 50,
                                          int max_size = 10000);
  std::string classifyShape(PointCPtr cloud);
  float detectShapeYaw(PointCPtr cloud);
  Eigen::Vector4f getCloudCentroid(PointCPtr cloud);

  // Planning scene helpers
  void addGroundCollisionPlane();
  void addBoxCollisionObject(const std::string & id,
                             double x, double y, double z,
                             double sx, double sy, double sz);
  void removeCollisionObject(const std::string & id);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<cw2_world_spawner::srv::Task1Service>::SharedPtr t1_service_;
  rclcpp::Service<cw2_world_spawner::srv::Task2Service>::SharedPtr t2_service_;
  rclcpp::Service<cw2_world_spawner::srv::Task3Service>::SharedPtr t3_service_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr color_cloud_sub_;
  rclcpp::CallbackGroup::SharedPtr pointcloud_callback_group_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex cloud_mutex_;
  PointCPtr g_cloud_ptr;
  std::uint64_t g_cloud_sequence_ = 0;
  std::string g_input_pc_frame_id_;
  std::string pointcloud_topic_;
  bool pointcloud_qos_reliable_ = false;

  static constexpr double GROUND_Z      = 0.020;
  static constexpr double SHAPE_HEIGHT  = 0.040;
  static constexpr double GRIPPER_OPEN  = 0.08;
  static constexpr double GRIPPER_CLOSED = 0.03;
  static constexpr double ARM_OVERHEAD_Z = 0.65;
};

#endif  // CW2_CLASS_H_
