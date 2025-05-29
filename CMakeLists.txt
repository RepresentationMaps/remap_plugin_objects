cmake_minimum_required(VERSION 3.8)
project(remap_plugin_objects)

add_compile_options(-fPIC)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${FIND_OPENVDB_PATH}")

find_package(yaml-cpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Targets
add_library(remap_plugin_objects SHARED
src/remap_plugin_objects/plugin_objects.cpp
src/remap_entity/entity.cpp)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

pluginlib_export_plugin_description_file(remap_plugin_base plugins.xml)

target_include_directories(remap_plugin_objects PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  remap_plugin_objects
  ament_index_cpp
  cv_bridge
  geometry_msgs
  pcl_conversions
  pcl_ros
  pluginlib
  rclcpp
  remap_map_handler
  remap_plugin_base
  remap_regions_register
  segmentation_msgs
  tf2
  tf2_ros
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  remap_plugin_objects
)

ament_export_targets(
  export_${PROJECT_NAME}
)

install(
  TARGETS remap_plugin_objects
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

set(TARGET_DIRECTORY ${PROJECT_SOURCE_DIR}/config)

file(GLOB FOLDERS ${TARGET_DIRECTORY}/*)

foreach(FOLDER_PATH ${FOLDERS})
    if(IS_DIRECTORY ${FOLDER_PATH})
        get_filename_component(FOLDER_NAME ${FOLDER_PATH} NAME)

        file(GLOB FILES "${FOLDER_PATH}/*")

        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.${FOLDER_NAME} "")

        foreach(FILE_PATH ${FILES})
            get_filename_component(FILE_NAME ${FILE_PATH} NAME)
            file(SIZE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.${FOLDER_NAME} FILE_SIZE)
            if(FILE_SIZE GREATER 0)
              file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.${FOLDER_NAME} ";")
            endif()  
            file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.${FOLDER_NAME} "config/${FOLDER_NAME}/${FILE_NAME}")
            install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.${FOLDER_NAME} DESTINATION share/ament_index/resource_index/remap.${FOLDER_NAME}/ RENAME ${PROJECT_NAME})
        endforeach()
    endif()
endforeach()

ament_auto_package()
