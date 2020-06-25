cmake_minimum_required(VERSION 2.8.3)
project(txt_to_orders)

find_package(catkin REQUIRED 
  rospy
  std_msgs
  geometry_msgs
  message_generation
  orders_supervisor
)


add_message_files(
  FILES
  Dictionary.msg
)

add_service_files(
  FILES
  TxtToOrdersSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
  orders_supervisor
)
catkin_package(
    CATKIN_DEPENDS orders_supervisor
)
