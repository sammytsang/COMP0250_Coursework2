#include <cw2_class.h>
#include <cmath>

void cw2::t1_callback(
  const std::shared_ptr<cw2_world_spawner::srv::Task1Service::Request> request,
  std::shared_ptr<cw2_world_spawner::srv::Task1Service::Response> /*response*/)
{
  RCLCPP_INFO(node_->get_logger(), "===== Task 1 Started =====");
  RCLCPP_INFO(node_->get_logger(), "Shape type: %s", request->shape_type.c_str());

  // Stabilise after any previous task: cancel lingering goals, let joint
  // state publisher settle, then re-read current state for planning.
  arm_group_->stop();
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  arm_group_->setStartStateToCurrentState();

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

  // 3. Observe shape with camera.
  //    Cross: PCA yaw detection — cross has a real principal axis to align gripper with.
  //    Nought: ring is rotationally symmetric — PCA gives noise, so keep grasp_yaw=0.
  //            Camera still confirms object is visible/in position.
  //    Move to "ready" first (known safe state), then above the object.
  double grasp_yaw = 0.0;
  {
    constexpr double OBS_HEIGHT = 0.5;
    moveArmToNamedTarget("ready");
    geometry_msgs::msg::Pose obs_pose =
      makeDownwardPose(obj_pt.x, obj_pt.y, obj_pt.z + OBS_HEIGHT, 0.0);
    bool moved = moveArmToPose(obs_pose);
    if (!moved) {
      RCLCPP_WARN(node_->get_logger(),
        "Task 1: failed to reach observation pose — using grasp_yaw=0");
    } else {
      PointCPtr cloud = waitForFreshCloud(5000);
      if (cloud && !cloud->empty()) {
        PointCPtr cloud_base = transformCloudToBaseFrame(cloud);
        PointCPtr cropped    = cropBoxAroundPoint(cloud_base,
                                                   obj_pt.x, obj_pt.y, obj_pt.z,
                                                   0.15);
        PointCPtr no_ground  = removeGroundPlane(cropped, 0.012f);
        if (request->shape_type == "cross") {
          // Cross: PCA on full cloud detects dominant arm direction
          if (no_ground && no_ground->size() >= 10) {
            float raw_yaw = detectShapeYaw(no_ground);
            double yaw = static_cast<double>(raw_yaw);
            while (yaw <  0.0)       yaw += M_PI / 2.0;
            while (yaw >= M_PI / 2.0) yaw -= M_PI / 2.0;
            grasp_yaw = yaw;
            RCLCPP_INFO(node_->get_logger(),
              "Task 1: cross raw_yaw=%.1f deg → grasp_yaw=%.1f deg",
              raw_yaw * 180.0f / static_cast<float>(M_PI),
              grasp_yaw * 180.0 / M_PI);
          } else {
            RCLCPP_WARN(node_->get_logger(),
              "Task 1: too few cloud points for cross — using grasp_yaw=0");
          }
        } else {
          // Nought: square ring (outer=100mm, inner=60mm, wall midpoint=80mm).
          // PCA is fundamentally broken here — a square frame has equal variance
          // in all directions (degenerate covariance). Edge/shell PCA also fails.
          //
          // Correct approach: directly scan which angle has ring material at 80mm.
          // For each candidate grasp_yaw θ in [0°,90°), count cloud points within
          // 25mm of (centroid + 80mm*[cosθ, sinθ]) — checking all 4 equivalent faces.
          // The angle with the highest count IS the face normal.
          if (no_ground && no_ground->size() >= 10) {
            Eigen::Vector4f cen;
            pcl::compute3DCentroid(*no_ground, cen);
            float cx = cen[0], cy = cen[1];

            const float ARM_OFF  = 0.080f;
            const float SEARCH_R = 0.025f;  // 25mm capture radius
            const int   N_STEPS  = 90;      // 1° resolution

            int    best_cnt   = -1;
            double best_theta = 0.0;

            for (int i = 0; i < N_STEPS; i++) {
              double theta = i * (M_PI / 2.0) / N_STEPS;
              int cnt = 0;
              for (int k = 0; k < 4; k++) {        // all 4 symmetric faces
                double t  = theta + k * M_PI / 2.0;
                float  tx = cx + ARM_OFF * static_cast<float>(std::cos(t));
                float  ty = cy + ARM_OFF * static_cast<float>(std::sin(t));
                for (const auto & pt : no_ground->points) {
                  float dx = pt.x - tx, dy = pt.y - ty;
                  if (dx*dx + dy*dy < SEARCH_R * SEARCH_R) cnt++;
                }
              }
              if (cnt > best_cnt) { best_cnt = cnt; best_theta = theta; }
            }

            grasp_yaw = best_theta;
            RCLCPP_INFO(node_->get_logger(),
              "Task 1: nought face_normal=%.1f deg (material score=%d)",
              grasp_yaw * 180.0 / M_PI, best_cnt);
          } else {
            RCLCPP_WARN(node_->get_logger(),
              "Task 1: too few cloud points for nought — using grasp_yaw=0");
          }
        }
      } else {
        RCLCPP_WARN(node_->get_logger(),
          "Task 1: no cloud received — using grasp_yaw=0");
      }
    }
  }

  // 4. Pick the object
  // Exact grasp offset geometry:
  //   Nought: ring inner=0.06m, outer=0.10m → midpoint at 0.08m ✅
  //   Cross:  arm has 3 blocks: centre(0–0.02m), inner(0.02–0.06m), outer(0.06–0.10m).
  //           0.060m = boundary between inner and outer blocks.
  //           This is 40mm from the centre junction (stable, single arm) and
  //           40mm from the arm tip — the optimal grip point on the arm. ✅
  geometry_msgs::msg::PointStamped grasp_point = request->object_point;
  const double ARM_OFFSET = (request->shape_type == "nought") ? 0.08 : 0.060;
  grasp_point.point.x += ARM_OFFSET * std::cos(grasp_yaw);
  grasp_point.point.y += ARM_OFFSET * std::sin(grasp_yaw);

  // Gripper jaw direction — accounting for the -45° panda_hand_joint rotation:
  //   panda_hand is mounted on panda_link8 with rpy="0 0 -0.785398" (-π/4 around Z).
  //   Fingers slide along panda_hand local-Y.
  //   In world frame, finger spread direction = angle (pick_yaw - π/4).
  //   Therefore to get fingers along world angle φ: pick_yaw = φ + π/4.
  //
  // grasp_yaw=0 → ARM_OFFSET applied along +X:
  //
  //   Nought (ring wall 40mm thick in X, 0.06–0.10m from centre):
  //     Need radial pinch (along +X, φ=0) so fingers straddle the wall thickness.
  //     → pick_yaw = grasp_yaw + π/4
  //
  //   Cross (arm along X, arm width 40mm along Y):
  //     Need transverse pinch (along +Y, φ=π/2) across the 40mm arm width.
  //     → pick_yaw = grasp_yaw + π/2 + π/4 = grasp_yaw + 3π/4
  double pick_yaw;
  if (request->shape_type == "nought") {
    pick_yaw = grasp_yaw + M_PI / 4.0;
    // Verify: finger world angle = pick_yaw - π/4 = grasp_yaw (radial, along ARM_OFFSET) ✓
    const double finger_world_angle_deg = (pick_yaw - M_PI / 4.0) * 180.0 / M_PI;
    const double edge_normal_deg = grasp_yaw * 180.0 / M_PI;
    RCLCPP_INFO(node_->get_logger(),
      "Nought: grasp=(%.3f,%.3f) offset=%.3fm  "
      "edge_normal=%.1fdeg finger_world=%.1fdeg (diff=%.1fdeg, expect 0) pick_yaw=%.1fdeg",
      grasp_point.point.x, grasp_point.point.y, ARM_OFFSET,
      edge_normal_deg, finger_world_angle_deg, finger_world_angle_deg - edge_normal_deg,
      pick_yaw * 180.0 / M_PI);
  } else {
    pick_yaw = grasp_yaw + 3.0 * M_PI / 4.0;
    // Verify: finger world angle = pick_yaw - π/4 = grasp_yaw + π/2 (across arm) ✓
    const double finger_world_angle_deg = (pick_yaw - M_PI / 4.0) * 180.0 / M_PI;
    const double across_arm_deg = (grasp_yaw + M_PI / 2.0) * 180.0 / M_PI;
    RCLCPP_INFO(node_->get_logger(),
      "Cross:  grasp=(%.3f,%.3f) offset=%.3fm  "
      "across_arm=%.1fdeg finger_world=%.1fdeg (diff=%.1fdeg, expect 0) pick_yaw=%.1fdeg",
      grasp_point.point.x, grasp_point.point.y, ARM_OFFSET,
      across_arm_deg, finger_world_angle_deg, finger_world_angle_deg - across_arm_deg,
      pick_yaw * 180.0 / M_PI);
  }

  bool pick_ok = pickObject(grasp_point, pick_yaw);
  if (!pick_ok) {
    RCLCPP_ERROR(node_->get_logger(), "Task 1: pick failed!");
  } else {
    // 5. Place in basket.
    //
    // For NOUGHT: rotationally symmetric → keep grasp_yaw direction and pick_yaw.
    //
    // For CROSS (T1_ANY_ORIENTATION=True): after picking at pick_yaw=grasp_yaw+3π/4
    //   (gripper rotated to match the cross arm), we want to place the cross PARALLEL
    //   to the basket (arm along world-X = 0°).  During the approach-to-basket motion
    //   MoveIt rotates the EEF from its current orientation to place_yaw=3π/4 (135°),
    //   which means:
    //     finger world angle = 3π/4 − π/4 = π/2 (along Y)
    //     cross arm ⊥ fingers → arm at 0° (along X) = parallel to basket ✓
    //   EEF position offset is then along 0° (X), so ARM_OFFSET is purely in X.
    //
    // For CROSS (T1_ANY_ORIENTATION=False): grasp_yaw≈0 → pick_yaw=3π/4, which is
    //   already equal to place_yaw=3π/4, and ARM_OFFSET*cos(0)=ARM_OFFSET in X.
    //   Behaviour is MATHEMATICALLY IDENTICAL to before — no change. ✓
    geometry_msgs::msg::PointStamped place_point = request->goal_point;
    double place_yaw_final;
    if (request->shape_type == "cross") {
      // Always place cross with arm at 0° (parallel to basket X-axis)
      place_yaw_final = 3.0 * M_PI / 4.0;
      place_point.point.x += ARM_OFFSET;  // offset along 0°
    } else {
      // Nought: symmetric, keep same direction as pick
      place_yaw_final = pick_yaw;
      place_point.point.x += ARM_OFFSET * std::cos(grasp_yaw);
      place_point.point.y += ARM_OFFSET * std::sin(grasp_yaw);
    }
    RCLCPP_INFO(node_->get_logger(),
      "Place EEF at (%.3f, %.3f) place_yaw=%.1fdeg → centroid at basket centre (%.3f, %.3f)",
      place_point.point.x, place_point.point.y, place_yaw_final * 180.0 / M_PI,
      goal_pt.x, goal_pt.y);

    // After picking: lift straight up to safe height, then go directly to basket.
    // No "ready" detour — that causes an unnecessary EEF rotation mid-transit.
    rclcpp::sleep_for(std::chrono::milliseconds(500));  // let joint state settle
    arm_group_->setStartStateToCurrentState();
    {
      auto cur_pose = arm_group_->getCurrentPose().pose;
      const double safe_z = std::max(cur_pose.position.z + 0.40, 0.65);
      geometry_msgs::msg::Pose lift_pose =
        makeDownwardPose(cur_pose.position.x, cur_pose.position.y,
                         safe_z, pick_yaw);
      if (!moveArmToPose(lift_pose)) {
        RCLCPP_WARN(node_->get_logger(),
          "Task 1: vertical lift failed — attempting place anyway");
      }
    }

    bool place_ok = placeObject(place_point, place_yaw_final);
    if (!place_ok) {
      RCLCPP_ERROR(node_->get_logger(), "Task 1: place failed!");
    }
  }

  // 6. Cleanup collision objects from MoveIt planning scene
  removeCollisionObject("t1_basket_N");
  removeCollisionObject("t1_basket_S");
  removeCollisionObject("t1_basket_E");
  removeCollisionObject("t1_basket_W");

  // NOTE: Gazebo entity deletion is intentionally NOT done here.
  // Calling /delete_entity from C++ (even with physics paused) causes ODE dError
  // crashes on macOS ARM64 with CycloneDDS because the C++ node cannot
  // establish a working service endpoint to /gazebo/pause_physics via rmw_cyclonedds_cpp.
  // world_spawner's reset_task() (Python) handles ALL entity cleanup: it pauses
  // physics first (Python call succeeds), then despawns all _object_ models,
  // then unpauses. This is triggered automatically when the next task is called.

  const bool returned_home = moveArmToNamedTarget("ready");
  if (!returned_home) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Task 1: non-critical failure returning to 'ready'; returning service response anyway");
  }
  RCLCPP_INFO(node_->get_logger(), "===== Task 1 Complete =====");
}
