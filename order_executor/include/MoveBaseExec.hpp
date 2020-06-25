#ifndef MOVE_BASE_EXEC_H
#define MOVE_BASE_EXEC_H

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>

#include <RosMath.hpp>
#include <Executor.hpp>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveBaseExec : public Executor{
public:
    MoveBaseExec(MoveBaseClient& mbcl, ros::Rate& rate, ros::Publisher& cmd_vel_pub, std::string odom_frame);
    void execute(const orders_supervisor::OrderGoalConstPtr& goal);
    void update_odom(const nav_msgs::OdometryConstPtr& odom);
private:
    void _transform_to_odom(geometry_msgs::PoseStamped& target);
    void _fix_empty_quat(geometry_msgs::PoseStamped& target); 

    tf2_ros::Buffer _tfbuffer;
    ros::Rate* _rate;
    tf2_ros::TransformListener _tfl;

    std::string _odom_frame;
    MoveBaseClient* _mbcl;
    ros::Publisher* _cmd_vel_pub;
    nav_msgs::OdometryConstPtr _odom;

};
#endif //MOVE_BASE_EXEC_H
