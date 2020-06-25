#include <OrdersContainer.hpp>

OrdersContainer::OrdersContainer() {};

void OrdersContainer::setCurrentDone(const actionlib::SimpleClientGoalState& state, const OrderResultConstPtr& result){
    OrderFeedbackSimple previous_order_fb;
    previous_order_fb.description = _feedbacks.current_order.description;
    previous_order_fb.id = _feedbacks.current_order.id; 
    previous_order_fb.result = state.getText();
    previous_order_fb.success = (state == state.SUCCEEDED);
    
    _feedbacks.previous_order = previous_order_fb;
    _feedbacks.current_order = OrderFeedback(); //reset current order   
}

bool OrdersContainer::empty() { 
    return _next_orders.empty(); 
}

void OrdersContainer::clear(){
    _next_orders.clear();
    _feedbacks.next_orders.clear();
}

void OrdersContainer::updateCurrentFB(const OrderFeedbackConstPtr& fb){
    //two components that allow tracking
    int order_id = _feedbacks.current_order.id;
    std::string order_description = _feedbacks.current_order.description;
    if (order_id != 0){ 
        _feedbacks.current_order = *fb;
        _feedbacks.current_order.id = order_id;
        _feedbacks.current_order.description = order_description;
    }
};

OrderGoal OrdersContainer::_pop_next(){
    OrderGoal nxt = _next_orders.front();
    _next_orders.pop_front();
    _feedbacks.next_orders.erase(_feedbacks.next_orders.begin());
    return nxt;
}

void OrdersContainer::remove(int id){
    _erase_id(_next_orders, id);
    _erase_id(_feedbacks.next_orders, id);
}

OrderGoal OrdersContainer::next(){
    OrderGoal nxt = _pop_next();   
    //set new order feedback to mark goal as current before receiving it by Executor's server
    OrderFeedback current_order_fb;
    current_order_fb.id = nxt.id;
    current_order_fb.description = nxt.description;
    current_order_fb.progress = "waiting for acceptance";
    
    _feedbacks.current_order = current_order_fb;

    return  nxt;
}

void OrdersContainer::append(OrderGoal& order){
    OrderFeedbackSimple fb;
    order.id = (++_current_id > 0) ? _current_id : 1; //avoid id < 0
    fb.id = _current_id;
    fb.description = order.description;
    
    _next_orders.push_back(order);
    _feedbacks.next_orders.push_back(fb);
}

const OrdersFeedback& OrdersContainer::feedbackMsg(){
    return _feedbacks;
}

template <typename T> void OrdersContainer::_erase_id(T& iterable, int id){
    if (iterable.empty()) return;
    iterable.erase(std::remove_if(iterable.begin(), iterable.end(), [id](decltype(iterable[0]) order){
        return (order.id == id);
    }));
}