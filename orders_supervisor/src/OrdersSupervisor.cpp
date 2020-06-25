
#include "OrdersSupervisor.hpp"
#include <algorithm>

Supervisor::Supervisor(ros::Publisher& feedback_pub, ActionClient& action_client, ros::Rate& fb_rate, ros::Rate& orders_rate) {
    _feedback_pub = &feedback_pub;
    _action_client = &action_client;
    _fb_rate = &fb_rate;
    _orders_rate = &orders_rate;
}

void Supervisor::add_orders(const OrdersConstPtr& orders){
    std::for_each(orders->data.begin(), orders->data.end(),
                    [=](OrderGoal order) { _orders.append(order); }
                );    
}


bool Supervisor::cancel_order(CancelOrderRequest& req, CancelOrderResponse& resp){
    if (_current_order.id == req.id){
        _action_client->cancelGoal();
    }
    else{
        _orders.remove(req.id);
    }
    
    resp.msg = "ok";
    resp.success = true;
    return true;
}


bool Supervisor::cancel_all(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp){
    _action_client->cancelAllGoals();
    _orders.clear();
    return true;
}

std::thread Supervisor::feedback_thread(){
    return std::thread( [=] { _start_pub_feedback(); }); 
}
std::thread Supervisor::orders_thread(){
    return std::thread( [=] { _start_send_orders(); }); 
}

void Supervisor::_start_send_orders(){
    while (ros::ok()){
        _orders_rate->sleep();
        
        if (_orders.empty()){
            continue;
        }
        
        //this flag has to be set even though action client waits for result, 
        // because it does not wait till the end of callbacks execution.
        bool done = false; 
        
        _current_order = _orders.next();
        _action_client->sendGoal(
            _current_order,
            [=, &done](const actionlib::SimpleClientGoalState& state, const OrderResultConstPtr& result) 
                {  _orders.setCurrentDone(state, result); 
                    done = true;}, //order done callback 
            [=](){}, //active cb   
            [=, &done](const OrderFeedbackConstPtr& feedback)
                {  if (!done) _orders.updateCurrentFB(feedback); } //feedback callback
        );

        _action_client->waitForResult();
        while (!done) {_orders_rate->sleep();} //wait for done callback to finish
    }
}


void Supervisor::_start_pub_feedback(){
    while (ros::ok()){
        _feedback_pub->publish(_orders.feedbackMsg());
        _fb_rate->sleep();
    }
}
