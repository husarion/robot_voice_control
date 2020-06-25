#include <Executor.hpp>

class ActionSrv{
public:
    ActionSrv(ros::NodeHandle nh);  
    void execute_order(const orders_supervisor::OrderGoalConstPtr& goal);
    void start();
    void bind_executor(std::string label, Executor& executor);

private:
    std::string _unrecognized_label_info(const orders_supervisor::OrderGoalConstPtr& goal);
    SimpleActionServer _order_srv;
    orders_supervisor::OrderResult _failure;

    std::map<std::string, Executor*> _executors;
};
