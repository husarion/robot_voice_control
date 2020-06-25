#include "OrdersSupervisor.hpp"


int main(int argc, char** argv){
    ros::init(argc, argv, "orders_supervisor");
    ros::NodeHandle nh;
    ros::Rate orders_rate = ros::Rate(2);
    ros::Rate feedback_rate = ros::Rate(4);

    ros::Publisher feedback_pub = nh.advertise<OrdersFeedback>("/orders_feedback", 1);
    
    ActionClient ac("/order");
    Supervisor supervisor(feedback_pub, ac, feedback_rate, orders_rate);
    ros::Subscriber orders_sub = nh.subscribe("/orders", 1, &Supervisor::add_orders, &supervisor);
    ros::ServiceServer cancel_order_srv = nh.advertiseService("/orders/cancel_one", &Supervisor::cancel_order, &supervisor);
    ros::ServiceServer cancel_all_srv = nh.advertiseService("/orders/cancel_all", &Supervisor::cancel_all, &supervisor);

    ROS_INFO("Connecting to order executor's action server");
    ac.waitForServer();
    ROS_INFO("Connected to server");
    
    std::thread feedback_thread = supervisor.feedback_thread();
    std::thread orders_thread = supervisor.orders_thread();
    ROS_INFO("Started listening to orders");

    ros::spin();
}   
