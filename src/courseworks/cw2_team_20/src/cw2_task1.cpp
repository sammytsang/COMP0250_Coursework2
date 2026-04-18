#include <cw2_class.h>
#include <cmath>

void cw2::t1_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> /*response*/)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 1 Started =====");
  RCLCPP_INFO(node_->get_logger(), "Shape type: %s", request->shape_type.c_str());

  // 1. Extract positions
  const auto & obj_pt  = request->object_point.point;
  const auto & goal_pt = request->goal_point.point;
  RCLCPP_INFO(node_->get_logger(),
    "Object at (%.3f, %.3f, %.3f) | Goal at (%.3f, %.3f, %.3f)",
    obj_pt.x, obj_pt.y, obj_pt.z,
    goal_pt.x, goal_pt.y, goal_pt.z);

  // 2. Add basket walls as collision objects to protect arm
  // Basket is 350x350x50mm centred at goal_pt
  addBoxCollisionObject("t1_basket_N", goal_pt.x + 0.19, goal_pt.y,         GROUND_Z + 0.025, 0.02, 0.37, 0.06);
  addBoxCollisionObject("t1_basket_S", goal_pt.x - 0.19, goal_pt.y,         GROUND_Z + 0.025, 0.02, 0.37, 0.06);
  addBoxCollisionObject("t1_basket_E", goal_pt.x,         goal_pt.y + 0.19, GROUND_Z + 0.025, 0.37, 0.02, 0.06);
  addBoxCollisionObject("t1_basket_W", goal_pt.x,         goal_pt.y - 0.19, GROUND_Z + 0.025, 0.37, 0.02, 0.06);

  // 3. Move overhead and detect shape orientation from point cloud
  double grasp_yaw = 0.0;
  {
    auto view_pose = makeDownwardPose(obj_pt.x, obj_pt.y, ARM_OVERHEAD_Z);
    if (moveArmToPose(view_pose)) {
      rclcpp::sleep_for(std::chrono::milliseconds(800));
      PointCPtr raw   = waitForFreshCloud(3000);
      PointCPtr base  = transformCloudToBaseFrame(raw);
      PointCPtr crop  = cropBoxAroundPoint(base, obj_pt.x, obj_pt.y, obj_pt.z, 0.15);
      PointCPtr shape = removeGroundPlane(crop, 0.015f);
      if (shape && shape->size() > 30) {
        grasp_yaw = detectShapeYaw(shape);
        RCLCPP_INFO(node_->get_logger(), "Detected shape yaw: %.3f rad", grasp_yaw);
      }
    }
  }

  // 4. Pick the object
  // Use the detected grasp_yaw to rotate the arm offset.
  // We always grab the arm that is +0.08m from the centroid,
  // but the direction of that arm depends on the shape orientation.
  geometry_msgs::msg::PointStamped grasp_point = request->object_point;

  // Rotate the +0.08m offset by the detected yaw angle
  // so we approach along the actual arm direction
  const double ARM_OFFSET = 0.08;
  grasp_point.point.x += ARM_OFFSET * std::cos(grasp_yaw);
  grasp_point.point.y += ARM_OFFSET * std::sin(grasp_yaw);

  // Gripper orientation:
  //   For nought: fingers close perpendicular to the arm -> pick_yaw = grasp_yaw
  //   For cross:  fingers close perpendicular to the arm -> pick_yaw = grasp_yaw + PI/2
  double pick_yaw = 0.0;
  if (request->shape_type == "nought") {
    pick_yaw = grasp_yaw;
    RCLCPP_INFO(node_->get_logger(),
      "Nought: grasping arm at (%.3f, %.3f) grasp_yaw=%.3f pick_yaw=%.3f",
      grasp_point.point.x, grasp_point.point.y, grasp_yaw, pick_yaw);
  } else {
    // cross
    pick_yaw = grasp_yaw + M_PI / 2.0;
    RCLCPP_INFO(node_->get_logger(),
      "Cross: grasping arm at (%.3f, %.3f) grasp_yaw=%.3f pick_yaw=%.3f",
      grasp_point.point.x, grasp_point.point.y, grasp_yaw, pick_yaw);
  }

  bool pick_ok = pickObject(grasp_point, pick_yaw);
  if (!pick_ok) {
    RCLCPP_ERROR(node_->get_logger(), "Task 1: pick failed!");
  } else {
    // 5. Place in basket
    bool place_ok = placeObject(request->goal_point, pick_yaw);
    if (!place_ok) {
      RCLCPP_ERROR(node_->get_logger(), "Task 1: place failed!");
    }
  }

  // 6. Cleanup
  removeCollisionObject("t1_basket_N");
  removeCollisionObject("t1_basket_S");
  removeCollisionObject("t1_basket_E");
  removeCollisionObject("t1_basket_W");
  const bool returned_home = moveArmToNamedTarget("ready");
  if (!returned_home) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Task 1: non-critical failure returning to 'ready'; returning service response anyway");
  }
  RCLCPP_INFO(node_->get_logger(), "===== Task 1 Complete =====");
}
