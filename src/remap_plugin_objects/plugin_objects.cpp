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
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/has_resource.hpp>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>

#include <pcl/filters/statistical_outlier_removal.h>

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
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_ptr_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};

  descriptor.description = "Distance threshold";
  node_ptr_->declare_parameter("plugin/objects/distance_threshold", 3.0, descriptor);

  distance_threshold_ = node_ptr_->get_parameter("plugin/objects/distance_threshold").as_double();

  // Checking if a detection class/ontology class mapping exists
  // We read the name of the mapping as a parameter (/plugin/objects/ontology_class_map)
  // and, if this exists as a resourse of type remap.ontology, we read the file

  descriptor.description = "Detection class/ontology class map file";
  node_ptr_->declare_parameter("plugin/objects/ontology_class_map", "coco_oro_mapping", descriptor);

  auto class_map_name = node_ptr_->get_parameter("plugin/objects/ontology_class_map").as_string();
  class_map_name += ".yaml";

  std::string resource_type = "remap.ontologies";
  auto class_map_path_fs = std::filesystem::path();

  std::map<std::string, std::string> resources = ament_index_cpp::get_resources(resource_type);
  if (resources.size() > 0) {
    for (const auto & resource : ament_index_cpp::get_resources(resource_type)) {
      std::string resource_name = resource.first;
      std::string resource_path = resource.second;
      std::string resource_content;
      ament_index_cpp::get_resource(resource_type, resource_name, resource_content);
      std::istringstream resource_content_stream(resource_content);
      std::string map_relative_path;
      char path_delimiter = ';';
      while (std::getline(resource_content_stream, map_relative_path, path_delimiter)) {
        if (map_relative_path.find(class_map_name) != std::string::npos) {
          class_map_path_fs = std::filesystem::path(resource_path) / std::string("share") /
            resource_name / map_relative_path;
        }
      }
      if (!class_map_path_fs.empty()) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Found class map: %s", class_map_path_fs.string().c_str());
        break;
      }
    }

    if (class_map_path_fs.empty()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(), "Class map %s not found in the resource index.",
        class_map_name.c_str());
    } else {
      YAML::Node config = YAML::LoadFile(class_map_path_fs.string());

      for (const auto & node : config) {
        std::string od_class = node.first.as<std::string>();
        if (node.second["ontology_class"]) {
          std::string ontology_class = node.second["ontology_class"].as<std::string>();
          ontology_class_map_[od_class] = ontology_class;
          RCLCPP_INFO(
            node_ptr_->get_logger(), "Mapping detection class '%s' to ontology class '%s'",
            od_class.c_str(), ontology_class.c_str());
        } else {
          RCLCPP_WARN(
            node_ptr_->get_logger(),
            "No ontology class mapping found for detection class '%s'",
            od_class.c_str());
        }
      }
    }
  } else {
    RCLCPP_ERROR(node_ptr_->get_logger(), "No resources of type  %s found.", resource_type.c_str());
  }

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

  bool depth_in_meters = false;

  if (depth_image->encoding != "16UC1") {
    depth_in_meters = true;
  }

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
  std::vector<std::string> new_facts;
  std::vector<std::string> old_facts;
  int i;
  for (i = 0; i < static_cast<int>(segmentation_array->detections.detections.size()); i++) {
    const auto & detection = segmentation_array->detections.detections[i];
    if (detection.results[0].hypothesis.class_id == "person") {
      continue;
    }
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
    // cv_mask.convertTo(cv_scaled_mask, CV_8U, 255);
    auto cv_depth_image = cv_bridge::toCvCopy(depth_image, depth_image->encoding)->image;
    auto depth_roi = cv_depth_image(mask_box);
    cv::Mat masked_depth;
    depth_roi.copyTo(masked_depth, cv_scaled_mask);

    std::vector<pcl::PointXYZ> points;
    std::vector<pcl::PointXYZ> filtered_points;
    std::vector<pcl::PointXYZ> rotated_points;

    for (int v = 0; v < masked_depth.rows; ++v) {
      for (int u = 0; u < masked_depth.cols; ++u) {
        uint16_t depth_value = masked_depth.at<uint16_t>(v, u);

        // Skip invalid depth value
        if (depth_value == 0) {continue;}

        float Z = (depth_in_meters) ? depth_value : (depth_value * 0.001f);

        // Compute 3D coordinates
        float X = (u + roi_x - cx_) * Z / fx_;
        float Y = (v + roi_y - cy_) * Z / fy_;

        // Add the point to the cloud
        // this instruction needs to be updated with the actual transform
        points.push_back(pcl::PointXYZ(X, Y, Z));
      }
    }

    filterPointCloud(points, filtered_points);
    const auto centroid_distance = computeCentroidDistance(filtered_points, detection.id);
    if (centroid_distance > distance_threshold_) {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "Object %s centroid distance is too far: %f", detection.id.c_str(), centroid_distance);
      continue;
    } else {
      RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Object %s centroid distance is: %f", detection.id.c_str(), centroid_distance);
    }

    transformPointCloud(filtered_points, rotated_points, transform_stamped);

    std::string object_id = detection.id;                          // we assume tracking is in place
    // here we check if it's a new object or not
    auto entities_objects_it = entities_objects_.find(object_id);
    if (entities_objects_it == entities_objects_.end()) {
      // This is the first time we see this object;
      // we create a new entity
      entities_objects_[object_id] = remap::entity::Entity(
        object_id, detection.results[0].hypothesis.class_id,
        node_ptr_->get_clock()->now().seconds());
      entities_objects_[object_id].setAnonEntityId(detection.results[0].hypothesis.class_id);
      entities_objects_[object_id].updateRemove_f(
        std::bind(
          &remap::map_handler::SemanticMapHandler::removeRegion,
          std::ref(*semantic_map_), std::placeholders::_1, std::ref(*regions_register_)));
      entities_objects_[object_id].update_f(
        std::bind(
          &remap::map_handler::SemanticMapHandler::insertSemanticPoints,
          std::ref(*semantic_map_), rotated_points, std::placeholders::_1,
          std::ref(*regions_register_)));
      std::string ontology_class;
      if (ontology_class_map_.size() > 0) {
        if (ontology_class_map_.find(detection.results[0].hypothesis.class_id) != ontology_class_map_.end()) {
          ontology_class = ontology_class_map_[detection.results[0].hypothesis.class_id];
        }
      }
      if (ontology_class.empty()) {
        ontology_class = detection.results[0].hypothesis.class_id;
      }
      new_facts.push_back(detection.results[0].hypothesis.class_id + " rdf:type " + ontology_class);

      // isIn computation. Demo only. Needs refinement.
      // This part has to be performed later, when inserting the object
    } else {
      // We update the time of the entity
      entities_objects_it->second.update_f(
        std::bind(
          &remap::map_handler::SemanticMapHandler::insertSemanticPoints,
          std::ref(*semantic_map_), rotated_points, std::placeholders::_1,
          std::ref(*regions_register_)));
      entities_objects_it->second.updateTime(node_ptr_->get_clock()->now().seconds());
    }
  }
  if (new_facts.size() > 0) {
    this->revisePushFacts(new_facts);
  }
}

void PluginObjects::run()
{
  std::lock_guard<std::mutex> lock(objects_mutex_);
  std::vector<std::string> entities_to_remove;
  std::vector<std::string> new_facts;

  for (auto & entity : entities_objects_) {
    entity.second.remove();
    if (!entity.second.checkTime(node_ptr_->get_clock()->now().seconds())) {
      // entities_to_remove.push_back(entity.first + "rdf:type " + entity.second.getEntityType());
      entities_to_remove.push_back(entity.first);
    } else {
      entity.second.map();

      auto object_id = entity.first;
      std::vector<std::string> rooms = {"kitchen", "living_room", "corridor", "working_area"};
      auto presence_entities = regions_register_->getCoexistentEntities(entity.second.getAnonEntityId());
      std::cout << "Found coexistence for object " << object_id << ": ";
      for (const auto & coex : presence_entities) {
        std::cout << coex << " ";
      }
      std::cout << std::endl;
      for (const auto & room : rooms) {
        if (presence_entities.find(room) != presence_entities.end()) {
          new_facts.push_back(entity.second.getAnonEntityId() + " isIn " + room);
        }
      }
    }
  }

  std::vector<std::string> old_facts;
  for (const auto & entity : entities_to_remove) {
    old_facts.push_back(entity + " rdf:type " + entities_objects_[entity].getEntityType());
    entities_objects_.erase(entity);
  }

  if (old_facts.size() > 0) {
    //this->reviseRemoveFacts(old_facts);   // removed for demo purposes
  }

  if (new_facts.size() > 0) {
    this->revisePushFacts(new_facts);
  }
}

void PluginObjects::storeEntitiesRelationships(
  std::map<std::string, std::map<std::string,
  std::string>> relationships_matrix)
{
  relationships_.clear();
  for (const auto & relationship : relationships_matrix) {
    auto subject = relationship.first;
    if (entities_objects_.find(subject) != entities_objects_.end()) {
      std::string object;
      for (const auto & matrix_elem : relationship.second) {
        object = matrix_elem.first;
        // We check whether the subject of the triple is a detected object
        if (entities_objects_.find(object) != entities_objects_.end()) {
          // The subject is actually an object detected by this plugin
          // We now iterate over the "row" of the matrix
          auto predicate = matrix_elem.second;
          if (predicate == "aboveTouching") {
            predicate = "oro:isOn";
          } else {
            continue;
          }
          std::string fact = subject + " " + predicate + " " + object;
          // If the relationships wasn't already stored, we store it
          if (std::find(
              relationships_.begin(), relationships_.end(),
              fact) == relationships_.end())
          {
            relationships_.push_back(fact);
          }
        }
      }
    }
  }

  for (const auto & relationship : relationships_) {
    // We check: if the relationship wans't already there, then we push it to the kb
    if (std::find(
        old_relationships_.begin(), old_relationships_.end(),
        relationship) == old_relationships_.end())
    {
      this->pushFact(relationship);
    }
  }

  for (const auto & old_relationship : old_relationships_) {
    // We check: if the relationship is no more there, then we remove it from the kb
    if (std::find(
        relationships_.begin(), relationships_.end(),
        old_relationship) == relationships_.end())
    {
      // this->removeFact(old_relationship);  // removed for demo purposes
    }
  }

  old_relationships_ = relationships_;
}

void PluginObjects::filterPointCloud(
  const std::vector<pcl::PointXYZ> & input_points,
  std::vector<pcl::PointXYZ> & output_points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> aligned_points(input_points.begin(), input_points.end());

  cloud->points = aligned_points;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_filtered);

  output_points = std::vector<pcl::PointXYZ>(cloud_filtered->points.begin(), cloud_filtered->points.end());
}

float PluginObjects::computeCentroidDistance(
  const std::vector<pcl::PointXYZ>& points,
  const std::string & object_id) {
  if (points.empty()) {
    if (object_id.size() > 0) {
      RCLCPP_WARN(
        node_ptr_->get_logger(),
        "No points found for object %s. Returning default point.", object_id.c_str());
    }
    return distance_threshold_ + 0.5f;
  }

  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

  for (const auto& point : points) {
      sum_x += point.x;
      sum_y += point.y;
      sum_z += point.z;
  }

  float n = static_cast<float>(points.size());
  pcl::PointXYZ centroid(sum_x / n, sum_y / n, sum_z / n);
  return std::sqrt(centroid.x * centroid.x +
                   centroid.y * centroid.y +
                   centroid.z * centroid.z);
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
