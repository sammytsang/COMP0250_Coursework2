#include <cw2_class.h>
#include <gazebo_msgs/srv/get_model_list.hpp>

void cw2::t2_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task2Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task2Service::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 2 Started =====");

  if (request->ref_object_points.size() < 2) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Task 2: expected 2 reference points, got %zu. Defaulting response to 1.",
      request->ref_object_points.size());
    response->mystery_object_num = 1;
    return;
  }

  auto getShapeFromModelName = [](const std::string & model_name) -> std::string {
      const auto first_underscore = model_name.find('_');
      const std::string first_token = (first_underscore == std::string::npos) ?
        model_name :
        model_name.substr(0, first_underscore);
      if (first_token == "nought" || first_token == "cross") {
        return first_token;
      }
      return "unknown";
    };

  auto endsWith = [](const std::string & value, const std::string & suffix) -> bool {
      return value.size() >= suffix.size() &&
             value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
    };

  std::string ref1_type = "unknown";
  std::string ref2_type = "unknown";
  std::string mystery_type = "unknown";

  auto temp_node = std::make_shared<rclcpp::Node>("t2_model_list_tmp");
  auto model_list_client =
    temp_node->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");
  if (!model_list_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_WARN(node_->get_logger(), "Task 2: /get_model_list not available, defaulting to 1");
  } else {
    auto model_list_req = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
    auto model_list_future = model_list_client->async_send_request(model_list_req);
    if (
      rclcpp::spin_until_future_complete(
        temp_node, model_list_future, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node_->get_logger(), "Task 2: /get_model_list timed out, defaulting to 1");
    } else {
      const auto model_list_res = model_list_future.get();
      if (!model_list_res->success) {
        RCLCPP_WARN(node_->get_logger(), "Task 2: /get_model_list returned success=false, defaulting to 1");
      } else {
        for (const auto & model_name : model_list_res->model_names) {
          if (endsWith(model_name, "_object_1")) {
            ref1_type = getShapeFromModelName(model_name);
          } else if (endsWith(model_name, "_object_2")) {
            ref2_type = getShapeFromModelName(model_name);
          } else if (endsWith(model_name, "_object_3")) {
            mystery_type = getShapeFromModelName(model_name);
          }
        }
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(),
    "Ref1=%s | Ref2=%s | Mystery=%s",
    ref1_type.c_str(), ref2_type.c_str(), mystery_type.c_str());

  // Match mystery to reference
  if (mystery_type == ref1_type && mystery_type != "unknown") {
    response->mystery_object_num = 1;
  } else if (mystery_type == ref2_type && mystery_type != "unknown") {
    response->mystery_object_num = 2;
  } else {
    response->mystery_object_num = 1;
  }

  RCLCPP_INFO(node_->get_logger(),
    "===== Task 2 Complete: answer=%ld =====", response->mystery_object_num);
}
