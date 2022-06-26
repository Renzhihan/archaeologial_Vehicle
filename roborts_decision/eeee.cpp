#include "behavior_tree/behavior_tree.h"

#include "action_maker.h" 
#include "action_node/frozen_action.h"
#include "action_node/descend_bucket_action.h"
#include "action_node/go_entrance_action.h"
#include "action_node/go_loading_action.h"
#include "action_node/go_unloading_action.h"
#include "action_node/lift_bucket_action.h"
#include "action_node/Masked_interrupt.h"
#include "action_node/go_electricity.h"
#include "executor/chassis_executor.h"
#include "blackboard/blackboard.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "laalal");
    ros::Time::init();

    //define date interact  ptr
    std::string config_file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
    auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(config_file_path);
    auto action_maker_ = std::make_shared<roborts_decision::ActionMaker>(blackboard_ptr_);
    std::shared_ptr<roborts_decision::ChassisExecutor> chassis_executor_ptr_ = blackboard_ptr_->GetChassisExecutor();
    while (roborts_decision::BehaviorState::SUCCESS!=chassis_executor_ptr_->Update())
    {
        action_maker_->GoGoalPose();
    }
 while(1){
        action_maker_->fangdieluo();
    }
}












