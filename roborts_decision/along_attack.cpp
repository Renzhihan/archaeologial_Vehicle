#include "behavior_tree/behavior_tree.h"
#include "action_maker.h"
//include  Action Header File

#include "action_node/frozen_action.h"
#include "action_node/descend_bucket_action.h"
#include "action_node/go_entrance_action.h"
#include "action_node/go_loading_action.h"
#include "action_node/go_unloading_action.h"
#include "action_node/lift_bucket_action.h"
#include "action_node/Masked_interrupt.h"
#include "action_node/go_electricity.h"
#include "roborts_msgs/state.h"
//include  mate interact Header File

int main(int argc, char **argv){
    ros::init(argc, argv, "along_attack_node");
    ros::Time::init();

    //define date interact  ptr
    std::string config_file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
    auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(config_file_path);
    auto action_maker_ = std::make_shared<roborts_decision::ActionMaker>(blackboard_ptr_);


// define action  node ptr
    auto frozen_action_=std::make_shared<roborts_decision::FrozenAction>(blackboard_ptr_,action_maker_);      
    auto descend_bucket_action_=std::make_shared<roborts_decision::FrozenAction>(blackboard_ptr_,action_maker_);      
    auto go_entrance_action_=std::make_shared<roborts_decision::FrozenAction>(blackboard_ptr_,action_maker_);      
    auto go_loading_action_=std::make_shared<roborts_decision::FrozenAction>(blackboard_ptr_,action_maker_);      
    auto go_unloading_action_=std::make_shared<roborts_decision::FrozenAction>(blackboard_ptr_,action_maker_);      
    auto lift_bucket_action_=std::make_shared<roborts_decision::FrozenAction>(blackboard_ptr_,action_maker_);      
    auto masked_interrupt_action_=std::make_shared<roborts_decision::MaskedInterrupt>(blackboard_ptr_,action_maker_); 
    auto go_electricity_action_=std::make_shared<roborts_decision::GoElectricity>(blackboard_ptr_,action_maker_); 

    auto electricity_selector_=std::make_shared<roborts_decision::SelectorNode>("electricity_selector",blackboard_ptr_);


    auto  electricity_loss_condition_ = std::make_shared<roborts_decision::PreconditionNode>("electricity_loss_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsLossElectricity() && blackboard_ptr_->GetElectricityEnableInterrupt()){ 
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

    auto task_selector_=std::make_shared<roborts_decision::SelectorNode>("task_selector",blackboard_ptr_);   
    electricity_loss_condition_->SetChild(task_selector_);



    auto  have_task_condition_ = std::make_shared<roborts_decision::PreconditionNode>("have_task_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsHaveTask()){ 
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);
        have_task_condition_->SetChild(masked_interrupt_action_);


    auto  not_have_task_condition_ = std::make_shared<roborts_decision::PreconditionNode>("not_have_task_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsHaveTask()){ 
                return false;
            } else {
                return true;
                }
        },
        roborts_decision::AbortType::BOTH);
        have_task_condition_->SetChild(go_electricity_action_);//这里要改  充电的条件的要改    不然电量一上去他就不充了

    task_selector_->AddChildren(not_have_task_condition_);
    task_selector_->AddChildren(have_task_condition_);


    auto call_selector_=std::make_shared<roborts_decision::SelectorNode>("call_selector",blackboard_ptr_);

    auto  is_not_called_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_not_called_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsHaveCall()){ 
                return false;
            } else {
                return true;
                }
        },
        roborts_decision::AbortType::BOTH);
        is_not_called_condition_->SetChild(frozen_action_);

    auto  is_called_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_called_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsHaveCall()){ 
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

        auto is_called_selector_=std::make_shared<roborts_decision::SelectorNode>("is_called_selector",blackboard_ptr_);
        is_called_condition_->SetChild(is_called_selector_);

    call_selector_->AddChildren(is_not_called_condition_);
    call_selector_->AddChildren(is_called_condition_);


    electricity_selector_->AddChildren(electricity_loss_condition_);
    electricity_selector_->AddChildren(call_selector_);


    auto  is_not_go_entrance_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_not_go_entrance_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsGoEntrance()){ 
                return false;
            } else {
                return true;
                }
        },
        roborts_decision::AbortType::BOTH);
    is_not_go_entrance_condition_->SetChild(go_entrance_action_);


    auto  is_not_go_load_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_not_go_load_condition_", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsGoLoad()){ 
                return false;
            } else {
                return true;
                }
        },
        roborts_decision::AbortType::BOTH);
    is_not_go_load_condition_->SetChild(go_loading_action_);

    auto  is_not_load_soil_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_not_load_soil_condition_", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsLoadSoil()){ 
                return false;
            } else {
                return true;
                }
        },
        roborts_decision::AbortType::BOTH);
    is_not_load_soil_condition_->SetChild(descend_bucket_action_);

    auto  is_not_lift_bucket_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_not_lift_bucket_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->FinishLoadingIsLift()){ 
                return false;
            } else {
                blackboard_ptr_->SetLoadingLift(1);
                return true;
                }
        },
        roborts_decision::AbortType::BOTH);
    is_not_load_soil_condition_->SetChild(lift_bucket_action_);

    auto  is_not_go_unload_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_not_go_unload_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsGoUnload()){ 
                return false;
            } else {
                return true;
                }
        },
        roborts_decision::AbortType::BOTH);
    is_not_go_unload_condition_->SetChild(go_unloading_action_);

    auto  is_not_unload_soil_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_not_unload_soil_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsUnloadSoil()){  //这里卸土完成条件没做处理
                return false;
            } else {
                return true;
                }
        },
        roborts_decision::AbortType::BOTH);
    is_not_unload_soil_condition_->SetChild(descend_bucket_action_);


    auto  is_not_lift_bucket2_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_not_lift_bucket2_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsLift()){   //斗是否升起条件没做处理
                return false;
            } else {
                return true;
                }
        },
        roborts_decision::AbortType::BOTH);
    is_not_unload_soil_condition_->SetChild(lift_bucket_action_);


    auto  is_lift_bucket2_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_lift_bucket2_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->IsLift()){ 
                blackboard_ptr_->SetHaveTask(0);
                blackboard_ptr_->SetElectricityEnableInterrupt(1);
                blackboard_ptr_->SetGoEntrance(0);
                blackboard_ptr_->SetHaveCall(0);
                blackboard_ptr_->SetGoUnload(0);
                blackboard_ptr_->SetGoLoad(0);
                blackboard_ptr_->SetLoadSoil(0);
                blackboard_ptr_->SetUnloadSoil(0);
                blackboard_ptr_->SetLoadingLift(0);
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);
    is_lift_bucket2_condition_->SetChild(frozen_action_); //可以冻结  也可以回到出发点



    is_called_selector_->AddChildren(is_not_go_entrance_condition_);
    is_called_selector_->AddChildren(is_not_go_load_condition_);
    is_called_selector_->AddChildren(is_not_load_soil_condition_);
    is_called_selector_->AddChildren(is_not_lift_bucket_condition_);
    is_called_selector_->AddChildren(is_not_go_unload_condition_);
    is_called_selector_->AddChildren(is_not_unload_soil_condition_);
    is_called_selector_->AddChildren(is_not_lift_bucket2_condition_);
    is_called_selector_->AddChildren(is_lift_bucket2_condition_);







//....................................................................

  ros::Rate rate(30);                
    roborts_decision::BehaviorTree root_(electricity_selector_, 500);
    while(ros::ok()){
        root_.Run();
        ros::NodeHandle n;
        ros::Publisher pub = n.advertise<roborts_msgs::state>("state", 20);
        roborts_msgs::state msg;
        msg.state =  blackboard_ptr_->GetRunningState();
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}


