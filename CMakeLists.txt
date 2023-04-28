cmake_minimum_required(VERSION 3.5)
project(recepcionist_forocoches)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    action_msgs
    rclcpp
    rclcpp_action
    behaviortree_cpp_v3
    ament_index_cpp
    nav2_msgs
    geometry_msgs
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(recepcionist_go_to_waypoint_bt_node SHARED src/recepcionist_forocoches/Go_to_Waypoint.cpp)
add_library(recepcionist_get_waypoint_bt_node SHARED src/recepcionist_forocoches/Get_Waypoint.cpp)

list(APPEND plugin_libs
  recepcionist_go_to_waypoint_bt_node
  recepcionist_get_waypoint_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(rec_forocoches src/recepcionist_main.cpp)
ament_target_dependencies(rec_forocoches ${dependencies})
target_link_libraries(rec_forocoches ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  rec_forocoches
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch behavior_tree_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
