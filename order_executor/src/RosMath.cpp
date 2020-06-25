#include <RosMath.hpp>

inline int sgn(float x){
    return ( (x > 0) - (x < 0) );
}

float yaw_from_quat(const geometry_msgs::Quaternion& q){
    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(quat);
    double _r, _p, yaw;
    m.getRPY(_r, _p, yaw);
    return yaw;
}

//smaller angular distance between two points - range (-pi, pi)
float angular_dist(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2){
    float diff = yaw_from_quat(q2) - yaw_from_quat(q1);
    float diff_c = diff - sgn(diff)*2*PI;
    
    if (abs(diff) < abs(diff_c))
        return diff;
    else
        return diff_c;
}

float linear_dist(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2){
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}
