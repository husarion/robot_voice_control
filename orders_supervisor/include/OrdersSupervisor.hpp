#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>
#include <orders_supervisor/Orders.h>
#include <orders_supervisor/CancelOrder.h>
#include <std_srvs/Trigger.h>
#include <OrdersContainer.hpp>
#include <thread>

// #include <orders_supervisor/CancelOrder.h>
// #include <orders_supervisor/ShiftOrder.h>


typedef actionlib::SimpleActionClient<OrderAction> ActionClient;

class Supervisor{
public:
    Supervisor(ros::Publisher& feedback_pub, ActionClient& action_client, ros::Rate& fb_rate, ros::Rate& orders_rate);

    void add_orders(const OrdersConstPtr& orders);
    bool cancel_order(CancelOrderRequest& req, CancelOrderResponse& resp);
    bool cancel_all(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

    std::thread feedback_thread();
    std::thread orders_thread();

private:
    void _start_pub_feedback();
    void _start_send_orders();
    void _update_next_orders_fb();

    ActionClient* _action_client;
    ros::Rate* _fb_rate;
    ros::Rate* _orders_rate;
    ros::Publisher* _feedback_pub;
    OrdersContainer _orders;
    OrderGoal _current_order;
};