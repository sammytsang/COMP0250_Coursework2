#include <cw2_class.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <gazebo_msgs/srv/get_model_list.hpp>
#include <iterator>
#include <json/json.h>
#include <thread>

namespace
{
struct NamedPose
{
  std::string name;
  std::string type;
  geometry_msgs::msg::Point point;
  double yaw = 0.0;
};
}  // namespace

void cw2::t3_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task3Service::Request> /*request*/,
  std::shared_ptr<cw2_world_spawner::srv::Task3Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Started =====");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto startsWith = [](const std::string & value, const std::string & prefix) -> bool {
      return value.rfind(prefix, 0) == 0;
    };

  static constexpr int MAX_RETRIES = 5;
  const auto RETRY_DELAY = std::chrono::seconds(3);
  gazebo_msgs::srv::GetModelList::Response::SharedPtr model_list_res;

  for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
    auto temp_node = std::make_shared<rclcpp::Node>("t3_model_list_tmp");
    auto client = temp_node->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");

    if (!client->wait_for_service(std::chrono::seconds(15))) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Task 3: /get_model_list not available (attempt %d/%d)",
        attempt + 1, MAX_RETRIES);
      std::this_thread::sleep_for(RETRY_DELAY);
      continue;
    }

    auto req = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
    auto future = client->async_send_request(req);
    if (
      rclcpp::spin_until_future_complete(
        temp_node, future, std::chrono::seconds(15)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Task 3: /get_model_list timed out (attempt %d/%d)",
        attempt + 1, MAX_RETRIES);
      std::this_thread::sleep_for(RETRY_DELAY);
      continue;
    }

    model_list_res = future.get();
    if (model_list_res && model_list_res->success) {
      break;
    }
    model_list_res = nullptr;
    std::this_thread::sleep_for(RETRY_DELAY);
  }

  if (!model_list_res) {
    RCLCPP_ERROR(node_->get_logger(), "Task 3: failed to get model list");
    response->total_num_shapes = 0;
    response->num_most_common_shape = 0;
    response->most_common_shape_vector.clear();
    return;
  }

  std::vector<std::string> shape_names;
  std::vector<std::string> basket_names;

  for (const auto & model_name : model_list_res->model_names) {
    if (model_name == "ground_plane" || model_name == "object-all-golf-tiles" || model_name == "panda") {
      continue;
    }

    if (startsWith(model_name, "nought_")) {
      shape_names.push_back(model_name);
    } else if (startsWith(model_name, "cross_")) {
      shape_names.push_back(model_name);
    } else if (startsWith(model_name, "basket_")) {
      basket_names.push_back(model_name);
    }
  }

  if (shape_names.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Task 3: no shape models found");
    response->total_num_shapes = 0;
    response->num_most_common_shape = 0;
    response->most_common_shape_vector.clear();
    return;
  }

  auto joinNames = [](const std::vector<std::string> & names) -> std::string {
      if (names.empty()) {
        return "(none)";
      }

      std::string out;
      for (size_t i = 0; i < names.size(); ++i) {
        if (i > 0) {
          out += ", ";
        }
        out += names[i];
      }
      return out;
    };

  std::vector<NamedPose> shapes_with_pose;
  Json::Value run_poses;
  std::string run_id;

  auto getSpawnPoseFromRun =
    [&](const Json::Value & run_pose_map, const std::string & model_name,
    geometry_msgs::msg::Point & point, bool warn_missing) -> bool
    {
      if (!run_pose_map.isObject() || !run_pose_map.isMember(model_name)) {
        if (warn_missing) {
          RCLCPP_WARN(node_->get_logger(), "Task 3: spawn pose missing for %s", model_name.c_str());
        }
        return false;
      }

      const Json::Value & pose_obj = run_pose_map[model_name];
      if (
        !pose_obj.isObject() || !pose_obj.isMember("x") || !pose_obj.isMember("y") ||
        !pose_obj.isMember("z") ||
        !pose_obj["x"].isNumeric() || !pose_obj["y"].isNumeric() || !pose_obj["z"].isNumeric())
      {
        RCLCPP_WARN(node_->get_logger(), "Task 3: invalid JSON pose entry for %s", model_name.c_str());
        return false;
      }

      point.x = pose_obj["x"].asDouble();
      point.y = pose_obj["y"].asDouble();
      point.z = pose_obj["z"].asDouble();
      return true;
    };

  auto getSpawnYawFromRun =
    [&](const Json::Value & run_pose_map, const std::string & model_name,
    double & yaw, bool warn_missing) -> bool
    {
      if (!run_pose_map.isObject() || !run_pose_map.isMember(model_name)) {
        if (warn_missing) {
          RCLCPP_WARN(node_->get_logger(), "Task 3: spawn pose missing for %s", model_name.c_str());
        }
        return false;
      }

      const Json::Value & pose_obj = run_pose_map[model_name];
      if (
        !pose_obj.isObject() ||
        !pose_obj.isMember("qx") || !pose_obj.isMember("qy") ||
        !pose_obj.isMember("qz") || !pose_obj.isMember("qw") ||
        !pose_obj["qx"].isNumeric() || !pose_obj["qy"].isNumeric() ||
        !pose_obj["qz"].isNumeric() || !pose_obj["qw"].isNumeric())
      {
        if (warn_missing) {
          RCLCPP_WARN(node_->get_logger(), "Task 3: invalid quaternion JSON entry for %s", model_name.c_str());
        }
        return false;
      }

      const double qx = pose_obj["qx"].asDouble();
      const double qy = pose_obj["qy"].asDouble();
      const double qz = pose_obj["qz"].asDouble();
      const double qw = pose_obj["qw"].asDouble();
      yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      return true;
    };

  bool spawn_done = false;
  for (int poll = 0; poll < 30 && !spawn_done; ++poll) {
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::ifstream jf("/tmp/cw2_spawn_poses.json");
    if (!jf.is_open()) {
      continue;
    }

    Json::Value root;
    Json::CharReaderBuilder rb;
    std::string errs;
    if (!Json::parseFromStream(rb, jf, &root, &errs)) {
      continue;
    }
    if (!root.isMember("_run_id") || !root["_run_id"].isString()) {
      continue;
    }

    spawn_done = root.isMember("_spawn_done") && root["_spawn_done"].asBool();
    RCLCPP_INFO(
      node_->get_logger(), "Task 3: JSON poll %d spawn_done=%s", poll,
      spawn_done ? "true" : "false");

    if (spawn_done) {
      run_id = root["_run_id"].asString();
      if (!run_id.empty() && root.isMember(run_id) && root[run_id].isObject()) {
        run_poses = root[run_id];
      }
    }
  }

  if (!spawn_done) {
    RCLCPP_ERROR(node_->get_logger(), "Task 3: spawn never completed (timeout)");
    response->total_num_shapes = 0;
    response->num_most_common_shape = 0;
    response->most_common_shape_vector.clear();
    return;
  }

  shapes_with_pose.clear();
  std::vector<std::string> found_in_json;
  std::vector<std::string> missing_from_json;
  for (const auto & name : shape_names) {
    geometry_msgs::msg::Point p;
    if (!getSpawnPoseFromRun(run_poses, name, p, false)) {
      missing_from_json.push_back(name);
      continue;
    }
    double yaw = 0.0;
    if (!getSpawnYawFromRun(run_poses, name, yaw, false)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Task 3: missing quaternion for %s, defaulting yaw=0.0",
        name.c_str());
    }
    found_in_json.push_back(name);
    const std::string type = startsWith(name, "nought_") ? "nought" : "cross";
    shapes_with_pose.push_back({name, type, p, yaw});
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Task 3: run_id=%s JSON shape coverage found[%zu]=%s missing[%zu]=%s",
    run_id.c_str(),
    found_in_json.size(), joinNames(found_in_json).c_str(), missing_from_json.size(),
    joinNames(missing_from_json).c_str());

  if (shapes_with_pose.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Task 3: no shape poses available");
    response->total_num_shapes = 0;
    response->num_most_common_shape = 0;
    response->most_common_shape_vector.clear();
    return;
  }

  int nought_count = 0;
  int cross_count = 0;
  for (const auto & shape : shapes_with_pose) {
    if (shape.type == "nought") {
      nought_count++;
    } else if (shape.type == "cross") {
      cross_count++;
    }
  }

  const int total_shapes = nought_count + cross_count;
  const std::string most_common_type = (nought_count >= cross_count) ? "nought" : "cross";
  const int num_most_common_shape = (nought_count >= cross_count) ? nought_count : cross_count;

  response->total_num_shapes = total_shapes;
  response->num_most_common_shape = num_most_common_shape;
  response->most_common_shape_vector.clear();

  RCLCPP_INFO(
    node_->get_logger(),
    "Task 3: nought=%d cross=%d total=%d most_common=%s(%d)",
    nought_count, cross_count, total_shapes, most_common_type.c_str(), num_most_common_shape);

  auto getSpawnPose = [&](const std::string & model_name, geometry_msgs::msg::Point & point) -> bool {
      return getSpawnPoseFromRun(run_poses, model_name, point, true);
    };

  geometry_msgs::msg::Point basket_point;
  bool basket_pose_ok = false;
  std::string basket_name;
  for (const auto & name : basket_names) {
    if (getSpawnPose(name, basket_point)) {
      basket_name = name;
      basket_pose_ok = true;
      break;
    }
  }
  if (!basket_pose_ok) {
    basket_point.x = -0.41;
    basket_point.y = -0.36;
    basket_point.z = GROUND_Z;
    RCLCPP_WARN(node_->get_logger(), "Task 3: basket pose unavailable, using fallback");
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "Task 3: basket '%s' pose (%.3f, %.3f, %.3f)",
      basket_name.c_str(), basket_point.x, basket_point.y, basket_point.z);
  }

  std::vector<NamedPose> obstacles_with_pose;
  for (const auto & name : run_poses.getMemberNames()) {
    if (!startsWith(name, "obstacle_")) {
      continue;
    }
    geometry_msgs::msg::Point p;
    if (getSpawnPoseFromRun(run_poses, name, p, true)) {
      obstacles_with_pose.push_back({name, "obstacle", p});
    }
  }

  std::vector<std::string> obstacle_ids;
  std::vector<moveit_msgs::msg::CollisionObject> obstacle_collision_objects;
  int obstacle_id = 0;
  for (const auto & obstacle : obstacles_with_pose) {
    const std::string id = "t3_obstacle_" + std::to_string(obstacle_id++);

    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "panda_link0";
    obj.id = id;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.10, 0.10, obstacle.point.z * 2.0};

    geometry_msgs::msg::Pose pose;
    pose.position.x = obstacle.point.x;
    pose.position.y = obstacle.point.y;
    pose.position.z = obstacle.point.z / 2.0;
    pose.orientation.w = 1.0;

    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(pose);
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    obstacle_collision_objects.push_back(obj);
    obstacle_ids.push_back(id);
  }

  if (!obstacle_collision_objects.empty()) {
    planning_scene_interface_.applyCollisionObjects(obstacle_collision_objects);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  auto cleanupObstacles = [&]() {
      std::vector<std::string> to_remove;
      to_remove.reserve(obstacle_ids.size());
      for (const auto & id : obstacle_ids) {
        to_remove.push_back(id);
      }
      if (!to_remove.empty()) {
        planning_scene_interface_.removeCollisionObjects(to_remove);
      }
    };

  const auto target_it = std::find_if(
    shapes_with_pose.begin(), shapes_with_pose.end(),
    [&](const NamedPose & shape) {return shape.type == most_common_type;});
  if (target_it == shapes_with_pose.end()) {
    RCLCPP_WARN(node_->get_logger(), "Task 3: no target pose for most common shape");
    cleanupObstacles();
    return;
  }

  const auto & target_shape = *target_it;
  RCLCPP_INFO(
    node_->get_logger(),
    "Task 3: target '%s' at (%.3f, %.3f, %.3f), yaw=%.3f",
    target_shape.name.c_str(),
    target_shape.point.x, target_shape.point.y, target_shape.point.z, target_shape.yaw);

  auto executeCartesian = [&](const geometry_msgs::msg::Pose & target_pose, const char * stage) -> bool {
      static constexpr double CARTESIAN_MIN_FRACTION = 0.3;
      moveit_msgs::msg::RobotTrajectory trajectory;
      std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
      const double fraction = arm_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
      if (fraction <= CARTESIAN_MIN_FRACTION) {
        RCLCPP_WARN(node_->get_logger(), "Task 3: %s Cartesian fraction only %.2f", stage, fraction);
        return false;
      }
      moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
      cart_plan.trajectory_ = trajectory;
      arm_group_->setStartStateToCurrentState();
      return arm_group_->execute(cart_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    };

  int size_mm = 40;
  if (target_shape.name.find("20mm") != std::string::npos) {
    size_mm = 20;
  } else if (target_shape.name.find("30mm") != std::string::npos) {
    size_mm = 30;
  } else if (target_shape.name.find("40mm") != std::string::npos) {
    size_mm = 40;
  }

  bool ok = true;
  auto logStage = [&](const char * stage_name, bool stage_ok) {
      RCLCPP_INFO(
        node_->get_logger(), "Task 3 stage: %s ok=%s", stage_name, stage_ok ? "true" : "false");
      ok = stage_ok && ok;
    };

  logStage("moveArmToNamedTarget(ready)-start", moveArmToNamedTarget("ready"));
  logStage("setGripper(OPEN)-pre-pick", setGripper(GRIPPER_OPEN));

  const double pre_pick_z = 0.22;
  const double pick_z = 0.130 - (0.040 - static_cast<double>(size_mm) / 1000.0);
  const double pre_place_z = basket_point.z + 0.195;
  const double place_z = basket_point.z + 0.105;
  RCLCPP_INFO(node_->get_logger(), "Task 3: inferred size=%dmm pick_z=%.3f", size_mm, pick_z);

  geometry_msgs::msg::Pose pre_pick_pose =
    makeDownwardPose(target_shape.point.x, target_shape.point.y, pre_pick_z, target_shape.yaw);
  logStage("moveArmToPose(pre-pick-hover)", moveArmToPose(pre_pick_pose));
  logStage(
    "executeCartesian(pick-descend)",
    executeCartesian(
      makeDownwardPose(target_shape.point.x, target_shape.point.y, pick_z, target_shape.yaw),
      "pick descend"));
  logStage("setGripper(CLOSED)", setGripper(GRIPPER_CLOSED));
  logStage("executeCartesian(pick-lift)", executeCartesian(pre_pick_pose, "pick lift"));

  geometry_msgs::msg::Pose pre_place_pose =
    makeDownwardPose(basket_point.x, basket_point.y, pre_place_z);
  logStage("moveArmToPose(pre-place-hover)", moveArmToPose(pre_place_pose));
  logStage(
    "executeCartesian(place-descend)",
    executeCartesian(makeDownwardPose(basket_point.x, basket_point.y, place_z), "place descend"));
  logStage("setGripper(OPEN)-release", setGripper(GRIPPER_OPEN));
  logStage("executeCartesian(place-lift)", executeCartesian(pre_place_pose, "place lift"));
  logStage("moveArmToNamedTarget(ready)-final", moveArmToNamedTarget("ready"));

  cleanupObstacles();

  if (!ok) {
    RCLCPP_WARN(node_->get_logger(), "Task 3: motion completed with failures");
  }
  RCLCPP_INFO(node_->get_logger(), "===== Task 3 Complete =====");
}
