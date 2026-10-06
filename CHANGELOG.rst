^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package remap_plugin_objects
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.3 (2026-10-06)
------------------
* compat lyrical and cleanup CMakeLists
  - use ament_auto, drop ament_target_dependencies
  - explicitly find and link OpenCV and PCL
  - declare remap_regions_register, sensor_msgs, tf2_geometry_msgs deps
  - use .hpp headers for tf2/message_filters/cv_bridge (drops humble)
  - pass rclcpp::QoS to message_filters subscribe on Kilted+
  Co-Authored-By: Claude Opus 5.5 (1M context) <noreply@anthropic.com>
* Contributors: Séverin Lemaignan

0.2.2 (2026-02-18)
------------------
* jazzy compat
  cv_bridge.h -> cv_bridge.hpp on jazzy
* removed pcl_ros dependency
* Contributors: Lorenzo Ferrini, Séverin Lemaignan

0.2.1 (2025-10-13)
------------------
* Fix ament_auto warning about headers install destination
* Contributors: Noel Jimenez

0.2.0 (2025-04-07)
------------------
* moved remap_entity to this package
* general clean up (including pleasing almighty linters)
* wip
* making linters happy
* managing relationships
* supporting 32 bits encoding
* Contributors: Lorenzo Ferrini, lorenzoferrini

0.1.0 (2025-02-24)
------------------
* making linters happy
* first commit
* Initial commit
* Contributors: Lorenzo Ferrini, lorenzoferrini
