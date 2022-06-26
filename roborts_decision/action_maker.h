#ifndef ACTION_MAKER_H
#define ACTION_MAKER_H

#include<ros/ros.h>
#include<tf/tf.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_listener.h>

#include<actionlib/client/simple_action_client.h>
#include "blackboard/blackboard.h"
#include "executor/chassis_executor.h"

#include "behavior_tree/behavior_node.h"
#include "behavior_tree/behavior_state.h"

#include "io/io.h"
#include "proto/decision.pb.h"

#include "roborts_msgs/Bucket.h"

namespace roborts_decision{
  class ActionMaker
  {
protected : 
  Blackboard::Ptr blackboard_ptr_;
    std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;


  public:
    typedef std::shared_ptr<ActionMaker> ActionMakerPtr;
  ActionMaker(const Blackboard::Ptr &blackboard_ptr):
  blackboard_ptr_(blackboard_ptr){
      chassis_executor_ptr_ = blackboard_ptr_->GetChassisExecutor();
  }
    ~ActionMaker() = default;


void LiftBucket(){
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roborts_msgs::Bucket>("bucket", 20);
  ros::Rate loop_rate(10);
  for(int i=0;i<10;i++){
    roborts_msgs::Bucket msg;
    msg.is_lift = 1;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void DescendBucket(){
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roborts_msgs::Bucket>("bucket", 20);
  ros::Rate loop_rate(10);
  for(int i=0;i<10;i++){
    roborts_msgs::Bucket msg;
    msg.is_lift = 0;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void GoGoalPose() {
      geometry_msgs::PoseStamped buff_goal;
      buff_goal.header.frame_id = "map";
      buff_goal.pose.position.x =3.6;
      buff_goal.pose.position.y =16;
      buff_goal.pose.position.z =0;     
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,1.57);  
      buff_goal.pose.orientation.x = quaternion.x();
      buff_goal.pose.orientation.y = quaternion.y();
      buff_goal.pose.orientation.z = quaternion.z();
      buff_goal.pose.orientation.w = quaternion.w();
      chassis_executor_ptr_->Execute(buff_goal);

    }

void fangdieluo() {
       yhs_can_msgs::ctrl_cmd goal;
       goal.ctrl_cmd_gear = 4;
       goal.ctrl_cmd_velocity = 0.4;
       goal.ctrl_cmd_steering = 0;
       chassis_executor_ptr_->Execute(goal);
    }



void GoEntrancePose() {
      geometry_msgs::PoseStamped entrance_goal;
      entrance_goal = blackboard_ptr_->GetEntrancePose();
      chassis_executor_ptr_->Execute(entrance_goal);
    }

void MaskedInterrupt() {
  blackboard_ptr_->SetElectricityEnableInterrupt(0);
    }

void SearchGoal(){
      geometry_msgs::PoseStamped path;
      path.header.frame_id = "map";
      path.pose.orientation.x = 0;
      path.pose.orientation.y = 0;
      path.pose.orientation.z = 0;
      path.pose.orientation.w = 1;
      path.pose.position.x = 0;
      path.pose.position.y = 0;
      path.pose.position.z = 0;
      int count = blackboard_ptr_->GetSearchCount();
      int size =  blackboard_ptr_->GetSearchPointSize();
        path = blackboard_ptr_->GetSearchPose(count);
        geometry_msgs::PoseStamped current_pose =blackboard_ptr_->GetRobotMapPose();
        auto dx = current_pose.pose.position.x - path.pose.position.x;        
        auto dy = current_pose.pose.position.y - path.pose.position.y;        
        double s_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        if(s_distance<0.8){
           blackboard_ptr_->SetSearchCount((count+1)%size);
        }
      count = blackboard_ptr_->GetSearchCount();
      path = blackboard_ptr_->GetSearchPose(count);
      path.pose.orientation.x = current_pose.pose.orientation.x;
      path.pose.orientation.y = current_pose.pose.orientation.y;
      path.pose.orientation.z = current_pose.pose.orientation.z;
      path.pose.orientation.w = current_pose.pose.orientation.w;
      try{
        chassis_executor_ptr_->Execute(path);
      }
      catch(std::exception& e){
        ROS_WARN("search execute error %s: ", e.what());
      }
    }

void CancelGoal() {

}    



 //   void CancelChassis(){
  //    chassis_executor_ptr_->Cancel();
 //   }

  };

}
#endif 