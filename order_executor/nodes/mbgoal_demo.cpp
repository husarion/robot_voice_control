#include <ActionSrv.hpp>
#include <MoveBaseExec.hpp>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "order_executor");
    ros::NodeHandle nh;
    ros::Rate r(10);

    //declare executors and their components
    
    //MoveBaseExec
    std::string odom_frame = "odom";
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    MoveBaseClient mb_client("/move_base", true);

    ROS_INFO("waiting for move_base server to become available");
    mb_client.waitForServer();
    ROS_INFO("connected with move base action server");

    MoveBaseExec move_base_exec(mb_client, r, cmd_vel_pub, odom_frame);

    ros::Subscriber odom = nh.subscribe("/odom", 1, &MoveBaseExec::update_odom, &move_base_exec);

    //declare ActionSrv class
    ActionSrv order_executor(nh);
    
    //bind all executors to proper labels
    order_executor.bind_executor("mbgoal", move_base_exec);

    //start action server
    ROS_INFO("starting order execution action server");
    order_executor.start();
    ROS_INFO("server started");

    ros::spin();
}
