#include <ActionSrv.hpp>

ActionSrv::ActionSrv(ros::NodeHandle nh):
            _order_srv(nh, "/order", boost::bind(&ActionSrv::execute_order, this, _1), false){
}

void ActionSrv::bind_executor(std::string label, Executor& executor){
    _executors[label] = &executor;
}

void ActionSrv::start(){
    for (const auto& pair : _executors){
        pair.second->start(_order_srv);
    }
    _order_srv.start();
}

void ActionSrv::execute_order(const orders_supervisor::OrderGoalConstPtr& goal){
    for (const auto& pair : _executors){
        if (pair.first == goal->label){
            pair.second->execute(goal);
            return;
        }   
    }
    _order_srv.setAborted(_failure, _unrecognized_label_info(goal));
}

std::string ActionSrv::_unrecognized_label_info(const orders_supervisor::OrderGoalConstPtr& goal){
    return ("could not recognize label: {" + 
            goal->label + "} of order with description: {" 
            + goal->description +"}"); 
}


