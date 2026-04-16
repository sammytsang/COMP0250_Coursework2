#include <cw2_class.h>
#include <cmath>

// ---------------------------------------------------------------------------
// makeDownwardPose
// ---------------------------------------------------------------------------
geometry_msgs::msg::Pose
cw2::makeDownwardPose(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // 180 deg around X — end-effector pointing straight down (x=1,y=0,z=0,w=0)
  tf2::Quaternion base_q;
  base_q.setRPY(M_PI, 0.0, 0.0);
  tf2::Quaternion yaw_q;
  yaw_q.setRPY(0.0, 0.0, yaw);
  tf2::Quaternion result_q = yaw_q * base_q;
  result_q.normalize();

  pose.orientation = tf2::toMsg(result_q);
  return pose;
}

// ---------------------------------------------------------------------------
// moveArmToPose
// ---------------------------------------------------------------------------
bool cw2::moveArmToPose(const geometry_msgs::msg::Pose & target_pose)
{
  const std::string planning_frame = arm_group_->getPlanningFrame();
  const std::string eef_link = arm_group_->getEndEffectorLink();
  RCLCPP_INFO(
    node_->get_logger(),
    "moveArmToPose: target (x=%.4f, y=%.4f, z=%.4f), planning_frame='%s', eef_link='%s'",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z,
    planning_frame.c_str(),
    eef_link.empty() ? "<empty>" : eef_link.c_str());

  arm_group_->setPoseTarget(target_pose);
  arm_group_->setStartStateToCurrentState();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_group_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    arm_group_->setStartStateToCurrentState();
    success = (arm_group_->execute(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);
  }
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "moveArmToPose: planning or execution failed");
  }
  return success;
}

// ---------------------------------------------------------------------------
// moveArmToNamedTarget
// ---------------------------------------------------------------------------
bool cw2::moveArmToNamedTarget(const std::string & target_name)
{
  arm_group_->setNamedTarget(target_name);
  arm_group_->setStartStateToCurrentState();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_group_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    arm_group_->setStartStateToCurrentState();
    success = (arm_group_->execute(plan) ==
               moveit::core::MoveItErrorCode::SUCCESS);
  }
  if (!success) {
    RCLCPP_WARN(node_->get_logger(),
                "moveArmToNamedTarget('%s'): planning or execution failed",
                target_name.c_str());
  }
  return success;
}

// ---------------------------------------------------------------------------
// setGripper
// ---------------------------------------------------------------------------
bool cw2::setGripper(double width)
{
  const double target_finger_position = width / 2.0;
  const std::vector<double> before_positions = hand_group_->getCurrentJointValues();

  hand_group_->setJointValueTarget("panda_finger_joint1", target_finger_position);
  hand_group_->setJointValueTarget("panda_finger_joint2", target_finger_position);
  hand_group_->setStartStateToCurrentState();
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (hand_group_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);
  if (success) {
    hand_group_->setStartStateToCurrentState();
    const auto execute_result = hand_group_->execute(plan);
    success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success && execute_result.val == moveit_msgs::msg::MoveItErrorCodes::ABORT) {
      const std::vector<double> after_positions = hand_group_->getCurrentJointValues();
      if (before_positions.size() >= 2 && after_positions.size() >= 2) {
        static constexpr double MIN_POSITION_CHANGE = 0.0001;
        const double finger1_change = std::abs(after_positions[0] - before_positions[0]);
        const double finger2_change = std::abs(after_positions[1] - before_positions[1]);

        if (finger1_change > MIN_POSITION_CHANGE || finger2_change > MIN_POSITION_CHANGE) {
          RCLCPP_WARN(
            node_->get_logger(),
            "setGripper: execution ABORTED but fingers moved (target width %.4f); accepting partial grasp",
            width);
          success = true;
        }
      }
    }
  }
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "setGripper: planning or execution failed");
  }
  return success;
}

// ---------------------------------------------------------------------------
// pickObject
// ---------------------------------------------------------------------------
bool cw2::pickObject(const geometry_msgs::msg::PointStamped & object_point,
                     double grasp_yaw)
{
  double x = object_point.point.x;
  double y = object_point.point.y;
  static constexpr double CARTESIAN_MIN_FRACTION = 0.3;
  static constexpr double PRE_GRASP_LINK8_Z = 0.22;
  static constexpr double DESCEND_TARGET_LINK8_Z = 0.130;
  const std::string planning_frame = arm_group_->getPlanningFrame();
  const std::string eef_link = arm_group_->getEndEffectorLink();

  bool ok = true;

  // 1. Move to ready
  ok = moveArmToNamedTarget("ready") && ok;

  // 2. Open gripper
  ok = setGripper(GRIPPER_OPEN) && ok;

  RCLCPP_INFO(
    node_->get_logger(),
    "pickObject: planning_frame='%s', eef_link='%s' (no explicit gripper offset configured)",
    planning_frame.c_str(),
    eef_link.empty() ? "<empty>" : eef_link.c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "pickObject: pre-grasp hover z=%.4f, descend target z=%.4f (panda_link8 targets)",
    PRE_GRASP_LINK8_Z,
    DESCEND_TARGET_LINK8_Z);

  // 3. Move to pre-grasp via Cartesian path — 10 cm above shape
  geometry_msgs::msg::Pose pre_grasp =
    makeDownwardPose(x, y, PRE_GRASP_LINK8_Z, grasp_yaw);
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints = {pre_grasp};
  double fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction > CARTESIAN_MIN_FRACTION) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    arm_group_->setStartStateToCurrentState();
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "pickObject: pre-grasp Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  // 4. Descend via Cartesian path
  geometry_msgs::msg::Pose grasp_pose =
    makeDownwardPose(x, y, DESCEND_TARGET_LINK8_Z, grasp_yaw);
  RCLCPP_INFO(
    node_->get_logger(),
    "pickObject: descend target pose (x=%.4f, y=%.4f, z=%.4f)",
    grasp_pose.position.x,
    grasp_pose.position.y,
    grasp_pose.position.z);
  waypoints = {grasp_pose};
  fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction > CARTESIAN_MIN_FRACTION) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    arm_group_->setStartStateToCurrentState();
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "pickObject: descend Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  // 5. Close gripper
  ok = setGripper(GRIPPER_CLOSED) && ok;

  // 6. Lift via Cartesian path
  waypoints = {pre_grasp};
  fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction > CARTESIAN_MIN_FRACTION) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    arm_group_->setStartStateToCurrentState();
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "pickObject: lift Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  // 8. Return to ready
  ok = moveArmToNamedTarget("ready") && ok;

  return ok;
}

// ---------------------------------------------------------------------------
// placeObject
// ---------------------------------------------------------------------------
bool cw2::placeObject(const geometry_msgs::msg::PointStamped & goal_point)
{
  double x = goal_point.point.x;
  double y = goal_point.point.y;
  double z = goal_point.point.z;
  static constexpr double CARTESIAN_MIN_FRACTION = 0.3;
  const double pre_place_z = z + 0.195;
  const double place_target_z = z + 0.105;

  bool ok = true;

  // 1. Move to pre-place via Cartesian path — 15 cm above basket
  geometry_msgs::msg::Pose pre_place =
    makeDownwardPose(x, y, pre_place_z);
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints = {pre_place};
  double fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction > CARTESIAN_MIN_FRACTION) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    arm_group_->setStartStateToCurrentState();
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "placeObject: pre-place Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  // 2. Descend into basket via Cartesian
  geometry_msgs::msg::Pose place_pose =
    makeDownwardPose(x, y, place_target_z);
  waypoints = {place_pose};
  fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction > CARTESIAN_MIN_FRACTION) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    arm_group_->setStartStateToCurrentState();
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "placeObject: descend Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  // 3. Release
  ok = setGripper(GRIPPER_OPEN) && ok;

  // 4. Ascend via Cartesian
  waypoints = {pre_place};
  fraction = arm_group_->computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
  if (fraction > CARTESIAN_MIN_FRACTION) {
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    arm_group_->setStartStateToCurrentState();
    ok = (arm_group_->execute(cart_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS) && ok;
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "placeObject: ascend Cartesian fraction only %.2f", fraction);
    ok = false;
  }

  return ok;
}

// ---------------------------------------------------------------------------
// addGroundCollisionPlane
// ---------------------------------------------------------------------------
void cw2::addGroundCollisionPlane()
{
  addBoxCollisionObject("ground_plane", 0.0, 0.0, -0.005, 3.0, 3.0, 0.01);
}

// ---------------------------------------------------------------------------
// addBoxCollisionObject
// ---------------------------------------------------------------------------
void cw2::addBoxCollisionObject(const std::string & id,
                                double x, double y, double z,
                                double sx, double sy, double sz)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = "panda_link0";
  obj.id = id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {sx, sy, sz};

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  planning_scene_interface_.applyCollisionObject(obj);
}

// ---------------------------------------------------------------------------
// removeCollisionObject
// ---------------------------------------------------------------------------
void cw2::removeCollisionObject(const std::string & id)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  planning_scene_interface_.applyCollisionObject(obj);
}
