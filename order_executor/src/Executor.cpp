#include <Executor.hpp>

void Executor::start(SimpleActionServer& as_){
    as = &as_;
}

strdict Executor::get_strdict(const orders_supervisor::OrderGoalConstPtr& goal){
    strdict dict;
    for(int i=0; i < goal->skeys.size(); i++)
        dict[(goal->skeys)[i]] = (goal->svals)[i];
    return dict;
} 

fdict Executor::get_fdict(const orders_supervisor::OrderGoalConstPtr& goal){
    fdict dict;
    for(int i=0; i < goal->fkeys.size(); i++)
        dict[(goal->fkeys)[i]] = (goal->fvals)[i];
    return dict;
} 