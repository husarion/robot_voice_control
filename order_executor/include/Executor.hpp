#ifndef EXECUTORT_H
#define EXECUTORT_H

#include <ros/ros.h>
#include <orders_supervisor/OrderAction.h>
#include <actionlib/server/simple_action_server.h>
#include <map>

typedef actionlib::SimpleActionServer<orders_supervisor::OrderAction> SimpleActionServer;
typedef std::map<std::string, float> fdict;
typedef std::map<std::string, std::string> strdict;

class Executor{
public:
    virtual void execute(const orders_supervisor::OrderGoalConstPtr& goal) = 0;
    void start(SimpleActionServer& as_);
protected:
    strdict get_strdict(const orders_supervisor::OrderGoalConstPtr& goal);
    fdict get_fdict(const orders_supervisor::OrderGoalConstPtr& goal);
    
    orders_supervisor::OrderFeedback feedback;
    orders_supervisor::OrderResult result;
    SimpleActionServer* as;
};

#endif //EXECUTORT_H