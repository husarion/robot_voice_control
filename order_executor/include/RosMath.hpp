#ifndef ROSMATH_H
#define ROSMATH_H

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_datatypes.h>

const float PI = 3.1415;
float yaw_from_quat(const geometry_msgs::Quaternion& q);
float angular_dist(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2);
float linear_dist(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);

#endif //ROSMATH_H