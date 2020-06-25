#include <MoveBaseExec.hpp>

geometry_msgs::Quaternion EMPTY_QUATERNION;

MoveBaseExec::MoveBaseExec(MoveBaseClient& mbcl, ros::Rate& rate, 
                            ros::Publisher& cmd_vel_pub, std::string odom_frame): _tfl(_tfbuffer)
{
    _mbcl = &mbcl;
    _rate = &rate;
    _cmd_vel_pub = &cmd_vel_pub;
    _odom_frame = odom_frame;
}

void MoveBaseExec::update_odom(const nav_msgs::OdometryConstPtr& odom){
    _odom = odom;
} 

void MoveBaseExec::execute(const orders_supervisor::OrderGoalConstPtr& goal){
    move_base_msgs::MoveBaseGoal mb_goal;
    bool mb_done = false; //changed to true when move base goal is done
    std::string mb_state_txt = ""; //text returned from move base after perfoming an action
    bool mb_success = false;  //indicates whether move base aciton succeeded

    feedback.description = goal->description;
    feedback.id = goal->id;
    feedback.progress = "accepted";
    as->publishFeedback(feedback); //publish preliminary feedback

    mb_goal.target_pose = goal->target_pose;
    _fix_empty_quat(mb_goal.target_pose);

    try{
        _transform_to_odom(mb_goal.target_pose);
    }
    catch (tf2::TransformException &ex) {
        result.success = false;
        as->setAborted(result, ex.what());
        return;
    }

    bool reached_dest = false;

    auto feedbackCb = [](const move_base_msgs::MoveBaseFeedbackConstPtr& _){};
    auto resultCb = [&mb_done, &mb_state_txt, &mb_success](const actionlib::SimpleClientGoalState& state, 
                                    const move_base_msgs::MoveBaseResultConstPtr& result){ 
        mb_done = true;
        mb_success = (state == state.SUCCEEDED);
        mb_state_txt = state.getText();
    };
    auto activeCb = [](){};
    
    _mbcl->sendGoal(mb_goal, resultCb, activeCb, feedbackCb);
    feedback.progress = "executing";
    while (!mb_done) 
    {
        if (as->isPreemptRequested() || !ros::ok()){
            result.success = false;
            _mbcl->cancelAllGoals();
            as->setPreempted(result, "preempted");
            return;
        }

        feedback.distance_linear = linear_dist(_odom->pose.pose.position, mb_goal.target_pose.pose.position);
        feedback.distance_angular = angular_dist(_odom->pose.pose.orientation, mb_goal.target_pose.pose.orientation);
        as->publishFeedback(feedback);

        _rate->sleep();
    }

    result.success = mb_success;
    if (result.success)
        as->setSucceeded(result, mb_state_txt);
    else
        as->setAborted(result, mb_state_txt);
}

void MoveBaseExec::_transform_to_odom(geometry_msgs::PoseStamped& target_pose){
    std::string frame_id = target_pose.header.frame_id;
    if (frame_id != _odom_frame){
        tf2::doTransform(target_pose, target_pose, 
            _tfbuffer.lookupTransform(_odom_frame, frame_id, ros::Time(), ros::Duration(1))
        );
    }   
}

void MoveBaseExec::_fix_empty_quat(geometry_msgs::PoseStamped& target_pose){
    if (target_pose.pose.orientation == EMPTY_QUATERNION)
        target_pose.pose.orientation.w = 1;
}