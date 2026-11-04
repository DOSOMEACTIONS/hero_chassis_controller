cmake_minimum_required(VERSION 3.0.2)
project(hero_chassis_controller)

find_package(catkin REQUIRED COMPONENTS
        controller_interface
        hardware_interface
        pluginlib
        control_toolbox
        geometry_msgs
        std_msgs
        nav_msgs
        tf
        roscpp
        roslint
)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/drive_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES hero_chassis_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# 代码风格检查（roslint / cpplint）
roslint_cpp()
