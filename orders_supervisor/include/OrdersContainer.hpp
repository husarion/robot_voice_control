
#include <orders_supervisor/OrdersFeedback.h>
#include <orders_supervisor/OrderAction.h>
#include <orders_supervisor/OrderResult.h>
#include <orders_supervisor/OrderFeedbackSimple.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <deque>

using namespace orders_supervisor;

class OrdersContainer{
public:
    OrdersContainer();
    void setCurrentDone(const actionlib::SimpleClientGoalState& state, const OrderResultConstPtr& result);
    
    void updateCurrentFB(const OrderFeedbackConstPtr& fb);

    bool empty();
    
    void clear();
    
    void append(OrderGoal& order);
    
    void remove(int id);
    
    OrderGoal next();
    
    const OrdersFeedback& feedbackMsg();

private:
    OrdersFeedback _feedbacks; //feedbacks sent back to user about previous, current and next orders. Check OrdersFeedback msg
 
    std::deque<OrderGoal> _next_orders; //next orders in form that they'll be published 
 
    int _current_id = 0; //id used for identifying each order by user (e.g. it makes possible to track/delete particular order)

    OrderGoal _pop_next();
    
    template <typename T> void _erase_id(T& iterable, int id);
};