#include <actionlib/client/simple_action_client.h>
#include "chassis_executor.h"
#include "yhs_can_msgs/ctrl_cmd.h"

namespace roborts_decision{

ChassisExecutor::ChassisExecutor():execution_mode_(ExcutionMode::IDLE_MODE), execution_state_(BehaviorState::IDLE),
                                   global_planner_client_("move_base", true)
{
  ros::NodeHandle nh;
  ctrl_cmd_pub_ = nh.advertise<yhs_can_msgs::ctrl_cmd>("ctrl_cmd", 1);
  ros::NodeHandle hw_nh;
  hw_sub = hw_nh.subscribe<yhs_can_msgs::hw>("hw_data",30, &ChassisExecutor::HwCallBack, this);

  global_planner_client_.waitForServer();
  ROS_INFO("Global planer server start!");
}

void ChassisExecutor::HwCallBack(const yhs_can_msgs::hw::ConstPtr & hw){
  // double x1 = hw->v1;
  // double x2 = hw->v2;
  // if(x1<2.1){
  //   count1++;
  //   if(count1>10){v1_ = x1;count1 = 0;}
  // }
  // else{ v1_ = hw->v1;}

  // if(x2<2.1){
  //   count2++;
  //   if(count2>10){v2_ = x2;count2 = 0;}
  // }
  // else{ v2_ = hw->v2;}
    a[a_count%10] = hw->v1;
  a_count++;
  b[b_count%10] = hw->v2;
  b_count++;
  for(int t = 0;t<10;t++){
      if(a[t]<2.1 && a[t]!=0){
          a_threshold++;
      }
  }
for(int t = 0;t<10;t++){
      if(b[t]<2.1 && b[t]!=0){
          b_threshold++;
      }
  }
  if(a_threshold>=5){
      a_threshold = 0;
      a_flag = 1; 
  }
  else {
      a_flag =  0;
      a_threshold = 0;}
    if(b_threshold>=5){
      b_threshold = 0;
      b_flag = 1;
  }
  else {
      b_flag =  0;
      b_threshold = 0;}
}

void ChassisExecutor::Execute(const geometry_msgs::PoseStamped &goal){
    if( execution_mode_ == ExcutionMode::SPEED_MODE){
    Cancel();
  }
  execution_mode_ = ExcutionMode::GOAL_MODE;
  global_planner_goal_.target_pose = goal;
  global_planner_client_.sendGoal(global_planner_goal_,
                                  GlobalActionClient::SimpleDoneCallback(),
                                  GlobalActionClient::SimpleActiveCallback(),
                                  boost::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, _1));
}

void ChassisExecutor::Execute( yhs_can_msgs::ctrl_cmd &twist){
  if( execution_mode_ == ExcutionMode::GOAL_MODE){
    Cancel();
  }
  // if(v1_<2.1){
  //   twist.ctrl_cmd_steering = -20;
  // }
  // if(v2_<2.1){
  //   twist.ctrl_cmd_steering = 20;
  // }
    if(a_flag == 1){
      Cancel();
      sleep(3);
    twist.ctrl_cmd_steering = -20;
  }
  if(b_flag == 1){
     Cancel();
      sleep(3);
    twist.ctrl_cmd_steering = 20;
  }
  execution_mode_ = ExcutionMode::SPEED_MODE;
  for(int x=0;x<30;x++){
  ctrl_cmd_pub_.publish(twist);
  }
}

BehaviorState ChassisExecutor::Update(){
  actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::LOST;
  switch (execution_mode_){
    case ExcutionMode::IDLE_MODE:
      execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::GOAL_MODE:
      state = global_planner_client_.getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE){
        ROS_INFO("%s : ACTIVE", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;

      } else if (state == actionlib::SimpleClientGoalState::PENDING) {
        ROS_INFO("%s : PENDING", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;

      } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("%s : SUCCEEDED", __FUNCTION__);
        execution_state_ = BehaviorState::SUCCESS;

      } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("%s : ABORTED", __FUNCTION__);
        execution_state_ = BehaviorState::FAILURE;

      } else {
        ROS_ERROR("Error: %s", state.toString().c_str());
        execution_state_ = BehaviorState::FAILURE;
      }
      break;

    case ExcutionMode::SPEED_MODE:
      execution_state_ = BehaviorState::RUNNING;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;

};

void ChassisExecutor::Cancel(){
  switch (execution_mode_){
    case ExcutionMode::IDLE_MODE:
      ROS_WARN("Nothing to be canceled.");
      break;

    case ExcutionMode::GOAL_MODE:
      global_planner_client_.cancelGoal();
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_MODE:
      zero_twist_.ctrl_cmd_gear = 3; 
      zero_twist_.ctrl_cmd_steering = 0;
      zero_twist_.ctrl_cmd_velocity = 0;
      ctrl_cmd_pub_.publish(zero_twist_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    default:
      ROS_ERROR("Wrong Execution Mode");
  }

}

void ChassisExecutor::GlobalPlannerFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& global_planner_feedback){

}

}