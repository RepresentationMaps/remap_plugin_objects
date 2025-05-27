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

#ifndef REMAP_PLUGIN_OBJECTS__PLUGIN_OBJECTS_HPP_
#define REMAP_PLUGIN_OBJECTS__PLUGIN_OBJECTS_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <remap_entity/entity.hpp>
#include <remap_plugin_base/plugin_base.hpp>
#include <remap_plugin_base/semantic_plugin.hpp>
#include <remap_regions_register/regions_register.hpp>

#include <segmentation_msgs/msg/segmentation_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
    segmentation_msgs::msg::SegmentationArray> SyncPolicy;

namespace remap
{
namespace plugins
{
class PluginObjects : public SemanticPlugin
{
private:
  void filterPointCloud(
    const std::vector<pcl::PointXYZ> & input_points,
    std::vector<pcl::PointXYZ> & output_points);
  float computeCentroidDistance(
    const std::vector<pcl::PointXYZ>& points,
    const std::string & object_id);
  void transformPointCloud(
    const std::vector<pcl::PointXYZ> & input_points,
    std::vector<pcl::PointXYZ> & output_points,
    const geometry_msgs::msg::TransformStamped & transform_stamped);

  std::vector<std::string> regions_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_old_;
  message_filters::Subscriber<segmentation_msgs::msg::SegmentationArray> segmentation_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> depth_seg_sync_;

  // Object storing depth masks for the objects
  std::map<std::string, std::vector<pcl::PointXYZ>> objects_points_;
  std::map<std::string, std::vector<pcl::PointXYZ>> new_objects_points_;

  float distance_threshold_;

  bool new_objects_;
  std::mutex objects_mutex_;
  std::vector<std::string> relationships_;
  std::vector<std::string> old_relationships_;

  std::map<std::string, remap::entity::Entity> entities_objects_;

  // tf2 objects
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // camera info
  bool camera_info_initialized_;
  std::string camera_optical_frame_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  std::string image_encoding_;

  void depthSegmentationCallback(
    const sensor_msgs::msg::Image::SharedPtr depth_image,
    const segmentation_msgs::msg::SegmentationArray::SharedPtr segmentation_array);

  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

public:
  PluginObjects();
  PluginObjects(
    std::shared_ptr<map_handler::SemanticMapHandler> & semantic_map,
    std::shared_ptr<remap::regions_register::RegionsRegister> & regions_register);
  ~PluginObjects();
  void run() override;
  void initialize() override;
  void storeEntitiesRelationships(
    std::map<std::string,
    std::map<std::string, std::string>> relationships_matrix) override;
};
}  // namespace plugins
}  // namespace remap
#endif  // REMAP_PLUGIN_OBJECTS__PLUGIN_OBJECTS_HPP_
