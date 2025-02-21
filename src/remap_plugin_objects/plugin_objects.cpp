// Copyright 2025 PAL Robotics, S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include "remap_plugin_objects/plugin_objects.hpp"

namespace remap
{
namespace plugins
{
PluginObjects::PluginObjects()
: SemanticPlugin() {}

PluginObjects::PluginObjects(
  std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
  std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register)
: SemanticPlugin(semantic_map, regions_register),
  camera_info_initialized_(false) {}

PluginObjects::~PluginObjects()
{
  timer_.reset();
  semantic_map_.reset();
  regions_register_.reset();
}

void PluginObjects::initialize()
{
  new_objects_ = false;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  camera_info_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/depth_registered/camera_info",
    rclcpp::SensorDataQoS(),
    std::bind(&PluginObjects::cameraInfoCallback, this, std::placeholders::_1));
  depth_image_sub_.subscribe(
    node_ptr_,
    "/depth_registered/image_rect",
    rmw_qos_profile_sensor_data);
  segmentation_sub_.subscribe(
    node_ptr_,
    "/world/objects/detections",
    rmw_qos_profile_sensor_data);
  depth_seg_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10),
    depth_image_sub_,
    segmentation_sub_);
  depth_seg_sync_->registerCallback(&PluginObjects::depthSegmentationCallback, this);
}

void PluginObjects::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  // Storing intrinsics and other camera parameters
  if (!camera_info_initialized_) {
    camera_optical_frame_ = camera_info->header.frame_id;
    fx_ = camera_info->k[0];
    fy_ = camera_info->k[4];
    cx_ = camera_info->k[2];
    cy_ = camera_info->k[5];

    camera_info_initialized_ = true;
    RCLCPP_INFO(node_ptr_->get_logger(), "Camera info initialized");
  } else {
    camera_info_sub_.reset();
  }
}

void PluginObjects::depthSegmentationCallback(
  const sensor_msgs::msg::Image::SharedPtr depth_image,
  const segmentation_msgs::msg::SegmentationArray::SharedPtr segmentation_array)
{
  if (!camera_info_initialized_) {
    RCLCPP_WARN(node_ptr_->get_logger(), "Camera info not initialized yet");
    return;
  }

  if (depth_image->encoding != "16UC1") {
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Image encoding not supported");
    return;
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Processing");

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      "map", camera_optical_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      node_ptr_->get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  // here we lock the objects mutex; this way we ensure that
  // the grid processing can happen correctly
  std::lock_guard<std::mutex> lock(objects_mutex_);
  int i;
  for (i = 0; i < static_cast<int>(segmentation_array->detections.detections.size()); i++) {
    const auto & detection = segmentation_array->detections.detections[i];
    const auto & mask = segmentation_array->masks[i];
    // We generate a Rect object equivalent to the bounding box of the segmentation mask
    auto roi_width = mask.width;
    auto roi_height = mask.height;
    auto roi_x = detection.bbox.center.position.x - roi_width / 2;
    auto roi_y = detection.bbox.center.position.y - roi_height / 2;
    cv::Rect mask_box(roi_x, roi_y, roi_width, roi_height);
    cv::Rect box_max(0, 0, depth_image->width, depth_image->height);
    mask_box &= box_max;                             // Ensuring that the mask roi fits the image

    auto cv_mask = cv_bridge::toCvCopy(mask, "mono8")->image;

    cv::Mat eroded_mask;
    cv::Mat erosion_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::erode(cv_mask, eroded_mask, erosion_kernel, cv::Point(-1, -1), 5);

    cv::Mat cv_scaled_mask;
    eroded_mask.convertTo(cv_scaled_mask, CV_8U, 255);
    auto cv_depth_image = cv_bridge::toCvCopy(depth_image, "16UC1")->image;
    auto depth_roi = cv_depth_image(mask_box);
    cv::Mat masked_depth;
    depth_roi.copyTo(masked_depth, cv_scaled_mask);

    std::vector<pcl::PointXYZ> points;
    std::vector<pcl::PointXYZ> rotated_points;

    for (int v = 0; v < masked_depth.rows; ++v) {
      for (int u = 0; u < masked_depth.cols; ++u) {
        uint16_t depth_value = masked_depth.at<uint16_t>(v, u);

        // Skip invalid depth value
        if (depth_value == 0) {continue;}

        float Z = depth_value * 0.001f;

        // Compute 3D coordinates
        float X = (u + roi_x - cx_) * Z / fx_;
        float Y = (v + roi_y - cy_) * Z / fy_;

        // Add the point to the cloud
        // this instruction needs to be updated with the actual transform
        points.push_back(pcl::PointXYZ(X, Y, Z));
      }
    }

    transformPointCloud(points, rotated_points, transform_stamped);

    std::string object_id = detection.id;                          // we assume tracking is in place
    new_objects_points_[object_id] = rotated_points;
    if (static_cast<int>(detection.results.size()) > 0) {
      new_entities_[object_id] = detection.results[0].hypothesis.class_id;
    }
  }
  new_objects_ = true;
  RCLCPP_INFO(node_ptr_->get_logger(), "New objects collected");
}

void PluginObjects::run()
{
  auto start_time = std::chrono::high_resolution_clock::now();
  std::lock_guard<std::mutex> lock(objects_mutex_);
  if (new_objects_) {
    for (const auto & object : objects_points_) {
      semantic_map_->removeRegion(object.first, *regions_register_);
    }
    for (const auto & new_object : new_objects_points_) {
      semantic_map_->insertSemanticPoints(new_object.second, new_object.first, *regions_register_);
    }
    updateEntities();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    std::cout << "Elapsed time: " << elapsed_seconds.count() << " seconds" << std::endl;
    objects_points_ = new_objects_points_;
    new_objects_points_.clear();
    new_objects_ = false;
    RCLCPP_INFO(node_ptr_->get_logger(), "Points inserted");
  }
}

void PluginObjects::updateEntities()
{
  for (const auto & entity : entities_) {  // we only keep those entities that got detected now
    if (new_entities_.find(entity.first) == new_entities_.end()) {
      // We found that this entity did not get re-detected;
      // We remove it from the map and the knowledge base.
      this->removeFact(entity.first + " rdf:type " + entity.second);
    }
  }
  for (const auto & new_entity : new_entities_) {
    if (entities_.find(new_entity.first) == entities_.end()) {
      // This is the first time we see this entity;
      // we update the knowledge base accordingly.
      this->pushFact(new_entity.first + " rdf:type " + new_entity.second);
    }
  }
  entities_ = new_entities_;
  new_entities_.clear();
}

void PluginObjects::transformPointCloud(
  const std::vector<pcl::PointXYZ> & input_points,
  std::vector<pcl::PointXYZ> & output_points,
  const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  Eigen::Affine3f eigen_transform;
  tf2::Transform tf2_transform;
  tf2::fromMsg(transform_stamped.transform, tf2_transform);

  eigen_transform = Eigen::Affine3f::Identity();
  eigen_transform.translation() << tf2_transform.getOrigin().x(),
    tf2_transform.getOrigin().y(),
    tf2_transform.getOrigin().z();
  eigen_transform.rotate(
    Eigen::Quaternionf(
      tf2_transform.getRotation().w(),
      tf2_transform.getRotation().x(),
      tf2_transform.getRotation().y(),
      tf2_transform.getRotation().z()));

  output_points.resize(input_points.size());
  for (size_t i = 0; i < input_points.size(); ++i) {
    output_points[i] = pcl::transformPoint(input_points[i], eigen_transform);
  }
}
}  // namespace plugins
}  // namespace remap

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(remap::plugins::PluginObjects, remap::plugins::PluginBase)
