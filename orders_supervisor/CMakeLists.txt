cmake_minimum_required(VERSION 2.8.3)
project(orders_supervisor)


add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            geometry_msgs
            message_generation
            actionlib
            actionlib_msgs
            )


add_message_files(
  FILES
  Orders.msg
  OrderFeedbackSimple.msg
  OrdersFeedback.msg
)

add_service_files(
  FILES
  CancelOrder.srv
  ShiftOrder.srv
)

add_action_files(
  DIRECTORY action
  FILES Order.action
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
  actionlib_msgs
)
  
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )
  
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs roscpp
  LIBRARIES ${PROJECT_NAME} 
  )

add_library(orders_supervisor src/OrdersSupervisor.cpp
                              src/OrdersContainer.cpp)
add_executable(main nodes/main.cpp)

target_link_libraries(main orders_supervisor ${catkin_LIBRARIES})

add_dependencies(orders_supervisor orders_supervisor_generate_messages_cpp)