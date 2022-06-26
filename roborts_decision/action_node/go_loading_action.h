#ifndef GO_LOADING_ACTION_H
#define GO_LOADING_ACTION_H
#include <unistd.h>      
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../behavior_tree/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../behavior_tree/behavior_node.h"
#include "../action_maker.h"
#include "../executor/chassis_executor.h"

namespace roborts_decision{
    class GoLoadingAction : public ActionNode {
    public:
        GoLoadingAction(const Blackboard::Ptr &blackboard_ptr, ActionMaker::ActionMakerPtr &action_maker_ptr) :
            ActionNode::ActionNode("go_loading_action", blackboard_ptr), action_maker_ptr_(action_maker_ptr) {
                chassis_executor_ptr_ = blackboard_ptr_->GetChassisExecutor();
        }

        virtual ~GoLoadingAction() = default;

    private:
        std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
        ros::Time start_time_;
        ActionMaker::ActionMakerPtr action_maker_ptr_;

        virtual void OnInitialize() {              
         start_time_=ros::Time::now();
         blackboard_ptr_->SetRunningState(2);
        ROS_INFO("%s %s",name_.c_str(),__FUNCTION__);
        }

        virtual BehaviorState Update() {
            ros::Time now_time = ros::Time::now();
            ROS_INFO("go_loading_action  time:%f",(now_time-start_time_).toSec());
           action_maker_ptr_->fangdieluo();
            ROS_INFO("go_loading_action");
            return chassis_executor_ptr_->Update();
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state){
            case BehaviorState::IDLE:
                ROS_INFO("%s %s IDLE!",name_.c_str(),__FUNCTION__);
                break;
            case BehaviorState::SUCCESS:
                ROS_INFO("%s %s SUCCESS!",name_.c_str(),__FUNCTION__);
                blackboard_ptr_->SetGoLoad(1);
                break;
            case BehaviorState::FAILURE:
                ROS_INFO("%s %s FAILURE!",name_.c_str(),__FUNCTION__);
                break;
            default:
                ROS_INFO("%s %s ERROR!",name_.c_str(),__FUNCTION__);
                return;
            }
        }

    }; 
} //namespace roborts_decision

#endif
//ROBORTS_DECISION_BACKBOOTAREA_H