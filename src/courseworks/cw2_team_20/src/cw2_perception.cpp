#include <cw2_class.h>

// ---------------------------------------------------------------------------
// waitForFreshCloud
// ---------------------------------------------------------------------------
PointCPtr cw2::waitForFreshCloud(int timeout_ms)
{
  std::uint64_t initial_seq;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    initial_seq = g_cloud_sequence_;
  }

  auto start = std::chrono::steady_clock::now();
  while (true) {
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (g_cloud_sequence_ > initial_seq && !g_cloud_ptr->empty()) {
        return g_cloud_ptr;
      }
    }
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start).count();
    if (elapsed >= timeout_ms) {
      RCLCPP_WARN(node_->get_logger(),
                  "waitForFreshCloud: timeout after %d ms", timeout_ms);
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      return g_cloud_ptr;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
}

// ---------------------------------------------------------------------------
// transformCloudToBaseFrame
// ---------------------------------------------------------------------------
PointCPtr cw2::transformCloudToBaseFrame(PointCPtr cloud)
{
  std::string src_frame;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    src_frame = g_input_pc_frame_id_;
  }

  PointCPtr output(new PointC);
  try {
    pcl_ros::transformPointCloud("panda_link0", *cloud, *output, tf_buffer_);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "transformCloudToBaseFrame: TF error: %s", ex.what());
    return cloud;
  }
  output->header.frame_id = "panda_link0";
  return output;
}

// ---------------------------------------------------------------------------
// cropBoxAroundPoint
// ---------------------------------------------------------------------------
PointCPtr cw2::cropBoxAroundPoint(PointCPtr cloud,
                                   double cx, double cy, double cz,
                                   double half_side)
{
  PointCPtr cropped(new PointC);
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(
    static_cast<float>(cx - half_side),
    static_cast<float>(cy - half_side),
    static_cast<float>(cz - 0.20),
    1.0f));
  crop.setMax(Eigen::Vector4f(
    static_cast<float>(cx + half_side),
    static_cast<float>(cy + half_side),
    static_cast<float>(cz + 0.20),
    1.0f));
  crop.filter(*cropped);
  return cropped;
}

// ---------------------------------------------------------------------------
// removeGroundPlane
// ---------------------------------------------------------------------------
PointCPtr cw2::removeGroundPlane(PointCPtr cloud, float dist_thresh)
{
  pcl::SACSegmentation<PointT> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(dist_thresh);
  seg.setInputCloud(cloud);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    return cloud;
  }

  PointCPtr result(new PointC);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*result);
  return result;
}

// ---------------------------------------------------------------------------
// filterByColourRange
// ---------------------------------------------------------------------------
PointCPtr cw2::filterByColourRange(PointCPtr cloud,
                                    float r_min, float r_max,
                                    float g_min, float g_max,
                                    float b_min, float b_max)
{
  PointCPtr filtered(new PointC);
  for (const auto & pt : cloud->points) {
    float r = static_cast<float>(pt.r) / 255.0f;
    float g = static_cast<float>(pt.g) / 255.0f;
    float b = static_cast<float>(pt.b) / 255.0f;
    if (r >= r_min && r <= r_max &&
        g >= g_min && g <= g_max &&
        b >= b_min && b <= b_max) {
      filtered->points.push_back(pt);
    }
  }
  filtered->width    = static_cast<uint32_t>(filtered->points.size());
  filtered->height   = 1;
  filtered->is_dense = true;
  return filtered;
}

// ---------------------------------------------------------------------------
// voxelDownsample
// ---------------------------------------------------------------------------
PointCPtr cw2::voxelDownsample(PointCPtr cloud, float leaf)
{
  PointCPtr downsampled(new PointC);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*downsampled);
  return downsampled;
}

// ---------------------------------------------------------------------------
// euclideanCluster
// ---------------------------------------------------------------------------
std::vector<PointCPtr> cw2::euclideanCluster(PointCPtr cloud,
                                              float tolerance,
                                              int min_size,
                                              int max_size)
{
  std::vector<PointCPtr> result;

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (const auto & idx : cluster_indices) {
    PointCPtr cluster(new PointC);
    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(idx));
    extract.setInputCloud(cloud);
    extract.setIndices(indices_ptr);
    extract.setNegative(false);
    extract.filter(*cluster);
    result.push_back(cluster);
  }
  return result;
}

// ---------------------------------------------------------------------------
// classifyShape
// ---------------------------------------------------------------------------
std::string cw2::classifyShape(PointCPtr cloud)
{
  if (!cloud || cloud->points.size() < 30) {
    return "unknown";
  }

  // Grid constants for the 2-D top-down projection
  static constexpr int    GRID_SIZE         = 64;
  static constexpr int    GRID_MAX_IDX      = GRID_SIZE - 1;   // 63
  static constexpr int    GRID_BORDER       = 2;               // border pixels excluded from hole search
  static constexpr int    GRID_INTERIOR_MAX = GRID_SIZE - 1 - GRID_BORDER;  // 61
  // Minimum valid shape dimension (metres) — smaller bbox is considered invalid
  static constexpr float  MIN_SHAPE_DIM_M   = 0.01f;

  // 1. Find XY bounding box
  float xmin = std::numeric_limits<float>::max();
  float xmax = -std::numeric_limits<float>::max();
  float ymin = std::numeric_limits<float>::max();
  float ymax = -std::numeric_limits<float>::max();

  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
      continue;
    }
    xmin = std::min(xmin, pt.x);
    xmax = std::max(xmax, pt.x);
    ymin = std::min(ymin, pt.y);
    ymax = std::max(ymax, pt.y);
  }

  float xrange = xmax - xmin;
  float yrange = ymax - ymin;
  if (xrange < MIN_SHAPE_DIM_M || yrange < MIN_SHAPE_DIM_M) {
    return "unknown";
  }

  // 2. Create GRID_SIZE×GRID_SIZE projection
  cv::Mat proj(GRID_SIZE, GRID_SIZE, CV_8U, cv::Scalar(0));
  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
      continue;
    }
    int gx = static_cast<int>((pt.x - xmin) / xrange * static_cast<float>(GRID_MAX_IDX));
    int gy = static_cast<int>((pt.y - ymin) / yrange * static_cast<float>(GRID_MAX_IDX));
    gx = std::max(0, std::min(GRID_MAX_IDX, gx));
    gy = std::max(0, std::min(GRID_MAX_IDX, gy));
    proj.at<uint8_t>(gy, gx) = 255;
  }

  // 3. Morphological operations
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(proj, proj, cv::MORPH_CLOSE, kernel);
  cv::dilate(proj, proj, kernel);

  // 4. Find interior holes
  cv::Mat inverted;
  cv::bitwise_not(proj, inverted);
  cv::floodFill(inverted, cv::Point(0, 0), cv::Scalar(128));

  int hole_pixels = 0;
  for (int row = GRID_BORDER; row <= GRID_INTERIOR_MAX; ++row) {
    for (int col = GRID_BORDER; col <= GRID_INTERIOR_MAX; ++col) {
      if (inverted.at<uint8_t>(row, col) == 0) {
        ++hole_pixels;
      }
    }
  }

  double hole_fraction =
    static_cast<double>(hole_pixels) /
    static_cast<double>(GRID_SIZE * GRID_SIZE);
  RCLCPP_INFO(node_->get_logger(),
              "classifyShape: hole_fraction = %.4f", hole_fraction);

  return (hole_fraction > 0.03) ? "nought" : "cross";
}

// ---------------------------------------------------------------------------
// detectShapeYaw
// ---------------------------------------------------------------------------
float cw2::detectShapeYaw(PointCPtr cloud)
{
  if (cloud->points.size() < 10) {
    return 0.0f;
  }
  pcl::PCA<PointT> pca;
  pca.setInputCloud(cloud);
  Eigen::Matrix3f evecs = pca.getEigenVectors();
  return std::atan2(evecs(1, 0), evecs(0, 0));
}

// ---------------------------------------------------------------------------
// getCloudCentroid
// ---------------------------------------------------------------------------
Eigen::Vector4f cw2::getCloudCentroid(PointCPtr cloud)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  return centroid;
}
