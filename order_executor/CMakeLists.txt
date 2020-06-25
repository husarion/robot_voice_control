cmake_minimum_required(VERSION 2.8.3)
project(order_executor)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
            orders_supervisor
            actionlib
            roscpp
            nav_msgs
            sensor_msgs
            tf2_ros
            tf
            image_transport
            cv_bridge
            )




include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS orders_supervisor
)

 add_library(order_executor src/ActionSrv.cpp
                            src/RosMath.cpp
                            src/Executor.cpp
                            #executors below
                            src/MoveBaseExec.cpp
            )

            add_executable(mbgoal_demo nodes/mbgoal_demo.cpp)

 target_link_libraries(mbgoal_demo order_executor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
