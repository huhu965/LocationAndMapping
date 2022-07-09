cmake_minimum_required(VERSION 2.8.3)
project(loam_frame)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions
  eigen_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(Boost REQUIRED COMPONENTS system timer filesystem)

set(G2O_TARGET_LIBRARIES "")
set(ALL_TARGET_LIBRARIES "")
set(YAML_TARGET_LIBRARIES "")
set(GLOG_TARGET_LIBRARIES "")
include(cmake/glog.cmake)
include(cmake/g2o.cmake)
include(cmake/global_defination.cmake)
include(cmake/YAML.cmake)
catkin_package()

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED QUIET)
find_package(Ceres REQUIRED QUIET)
find_package(g2o REQUIRED QUIET)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS PCL EIGEN3
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  /usr/include/boost
)
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/)
list(APPEND ALL_TARGET_LIBRARIES libGeographiccc)

link_directories(
  include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
  ${Eigen3_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
  ${G2O_LIBRARY_DIRS}
  /usr/lib/x86_64-linux-gnu
)

add_executable(FrontEndNode  
              src/app/front_end_node.cpp
              src/mapping/front_end/front_end_flow.cpp
              src/mapping/front_end/front_end.cpp 
              src/modules/matching/scan_to_scan.cpp 
              src/modules/matching/scan_to_map.cpp 
              src/modules/feature_extract/go_feature_extract.cpp 
              src/modules/feature_extract/normal_feature_extract.cpp
              src/modules/cloud_filter/voxel_filter.cpp 
              src/modules/registration/ICP/aloam_registration.cpp
              src/modules/registration/ICP/fast_registration.cpp 
              src/modules/transformation/point_cloud_transformation.cpp 
              src/modules/transformation/pose_transformation.cpp 
              src/modules/scan_adjust/distortion_adjust.cpp
              src/data_pretreat/imu_preintegrate.cpp
              src/sensor_data/gnss_data.cpp
              src/sensor_data/imu_data.cpp  
              src/publisher/cloud_publisher.cpp
              src/publisher/path_publisher.cpp 
              src/publisher/odometry_publisher.cpp
              src/subscriber/cloud_subscriber.cpp
              src/subscriber/gnss_subscriber.cpp
              src/subscriber/imu_subscriber.cpp
              src/tf_listener/tf_listener.cpp
              src/tools_/file_manager.cpp)

target_link_libraries(FrontEndNode 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES} 
                      ${Eigen3_LIBRARY_DIRS} 
                      ${CERES_LIBRARIES}
                      ${YAML_TARGET_LIBRARIES}
                      ${GLOG_TARGET_LIBRARIES}
                      ${CHOLMOD_LIBRARIES}
                      ${ALL_TARGET_LIBRARIES})

add_executable(BackEndNode
              src/app/back_end_node.cpp
              src/mapping/back_end/back_end_flow.cpp
              src/mapping/back_end/back_end.cpp 
              src/sensor_data/gnss_data.cpp
              src/publisher/cloud_publisher.cpp
              src/publisher/path_publisher.cpp
              src/subscriber/cloud_subscriber.cpp
              src/subscriber/odometry_subscriber.cpp
              src/modules/optimization/g2o/g2o_graph_optimization.cpp
              src/modules/map/map_process.cpp
              src/modules/map/map_variable.cpp
              src/modules/map/submap_process.cpp
              src/modules/transformation/point_cloud_transformation.cpp
              src/tools_/file_manager.cpp
              )

target_link_libraries(BackEndNode 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES} 
                      ${Eigen3_LIBRARY_DIRS} 
                      ${CERES_LIBRARIES}
                      ${YAML_TARGET_LIBRARIES}
                      ${GLOG_TARGET_LIBRARIES}
                      ${G2O_TARGET_LIBRARIES}
                      ${CHOLMOD_LIBRARIES}
                      ${ALL_TARGET_LIBRARIES})

add_executable(SavePathNode
              src/app/save_path_node.cpp
              src/sensor_data/gnss_data.cpp
              src/publisher/path_publisher.cpp
              src/subscriber/gnss_subscriber.cpp
              src/subscriber/path_subscriber.cpp
              src/tf_listener/tf_listener.cpp
              src/tools_/file_manager.cpp
              )
                
target_link_libraries(SavePathNode 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES} 
                      ${OpenCV_LIBRARIES} 
                      ${Eigen3_LIBRARY_DIRS} 
                      ${CERES_LIBRARIES}
                      ${YAML_TARGET_LIBRARIES}
                      ${GLOG_TARGET_LIBRARIES}
                      ${G2O_TARGET_LIBRARIES}
                      ${CHOLMOD_LIBRARIES}
                      ${ALL_TARGET_LIBRARIES})

# add_executable(test_node
#               src/app/test_cloud.cpp
#               src/mapping/front_end/front_end_flow.cpp
#               src/mapping/front_end/front_end.cpp 
#               src/modules/matching/scan_to_scan.cpp 
#               src/modules/matching/scan_to_map.cpp 
#               src/modules/feature_extract/go_feature_extract.cpp 
#               src/modules/feature_extract/normal_feature_extract.cpp
#               src/modules/cloud_filter/voxel_filter.cpp 
#               src/modules/registration/ICP/aloam_registration.cpp
#               src/modules/registration/ICP/fast_registration.cpp 
#               src/modules/transformation/point_cloud_transformation.cpp 
#               src/modules/transformation/pose_transformation.cpp 
#               src/modules/scan_adjust/distortion_adjust.cpp
#               src/sensor_data/gnss_data.cpp 
#               src/publisher/cloud_publisher.cpp
#               src/publisher/path_publisher.cpp 
#               src/subscriber/cloud_subscriber.cpp
#               src/subscriber/gnss_subscriber.cpp
#               src/tf_listener/tf_listener.cpp
#               src/tools_/file_manager.cpp)

# target_link_libraries(test_node 
#                       ${catkin_LIBRARIES}
#                       ${PCL_LIBRARIES} 
#                       ${OpenCV_LIBRARIES} 
#                       ${Eigen3_LIBRARY_DIRS} 
#                       ${CERES_LIBRARIES}
#                       ${YAML_TARGET_LIBRARIES}
#                       ${GLOG_TARGET_LIBRARIES}
#                       ${CHOLMOD_LIBRARIES}
#                       ${ALL_TARGET_LIBRARIES})

# add_executable(MappingNode  
#               src/mapping_node.cpp 
#               src/modules/point_cloud_handle/point_cloud_variable.cpp
#               src/modules/point_cloud_handle/point_cloud_segmentation.cpp
#               src/modules/point_cloud_handle/feature_extract.cpp
#               src/modules/pose_transform/pose_transform_variable.cpp
#               src/modules/registration/scan_to_map_icp.cpp
#               src/publisher/odometry_publisher.cpp
#               src/publisher/cloud_publisher.cpp
#               src/publisher/path_publisher.cpp
#               src/subscriber/cloud_subscriber.cpp
#               src/subscriber/odometry_subscriber.cpp
#               src/subscriber/gnss_subscriber.cpp
#               src/modules/map/map_variable.cpp
#               src/modules/map/map_process.cpp
#               src/modules/map/submap_process.cpp 
#               src/modules/optimization/g2o/g2o_graph_optimization.cpp
#               src/sensor_data/gnss_data.cpp 
#               src/tf_listener/tf_listener.cpp 
#               )

# target_link_libraries(MappingNode 
#                       ${catkin_LIBRARIES} 
#                       ${PCL_LIBRARIES} 
#                       ${OpenCV_LIBRARIES} 
#                       ${Eigen3_LIBRARY_DIRS} 
#                       ${CERES_LIBRARIES}
#                       ${G2O_TARGET_LIBRARIES}
#                       ${CHOLMOD_LIBRARIES}
#                       ${ALL_TARGET_LIBRARIES}
#                       )

# add_executable(featureAssociation src/featureAssociation.cpp)
# add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
# target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(mapOptmization src/mapOptmization.cpp)
# target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

# add_executable(transformFusion src/transformFusion.cpp)
# target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
