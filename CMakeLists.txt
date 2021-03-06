cmake_minimum_required(VERSION 2.8.3)
project(motion_creator)
add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rviz
        geometry_msgs
        event_capture
        tf2
        tf2_ros
        pcl_ros
        tf2_geometry_msgs
        visualization_msgs)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS roscpp rviz geometry_msgs event_capture tf2 tf2_ros pcl_ros tf2_geometry_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

qt5_wrap_cpp(QT_HEADER_FILES
        src/maker/widget.hpp
)

set(QT_SOURCE_FILES
        src/maker/widget.cpp
        src/utils/waypoints.cpp
        src/utils/points_map.cpp
        src/utils/read_yaml.cpp
        src/library/interpolate_exported.cpp
        src/library/planning_utils_exported.cpp
)

add_library(${PROJECT_NAME}
  ${QT_HEADER_FILES}
  ${QT_SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  yaml-cpp
)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )