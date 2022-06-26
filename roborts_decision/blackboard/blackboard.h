#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include "io/io.h"
#include "../proto/decision.pb.h"
//#include "costmap/costmap_interface.h"
#include "../executor/chassis_executor.h"

#include "roborts_msgs/Order.h"
#include "roborts_msgs/Bucket.h"
#include "yhs_can_msgs/bms_flag_Infor_fb.h"


namespace roborts_decision{

class Blackboard {
 public:
  typedef std::shared_ptr<Blackboard> Ptr;
  //typedef roborts_costmap::CostmapInterface CostMap;
  //typedef roborts_costmap::Costmap2D CostMap2D;
  explicit Blackboard(const std::string &proto_file_path):
  proto_file_path_(proto_file_path),
  have_call_(0),
  call_number_(0),
  bms_flag_Infor_soc_(100),
  electricity_enable_interrupt_(1),
  is_have_task_(0),
  is_go_entrance_(0),
  is_go_load_(0),
  is_load_soil_(0),
  search_points_size_(0),
  is_lift_(1),
  is_go_unload_(0),
  running_state(0),
  finish_loading_is_lift_(0),
  search_count_(0),
  is_unload_soil_(0)
  {

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

    // std::string map_path = ros::package::getPath("roborts_costmap") + \
    //   "/config/costmap_parameter_config_for_decision.prototxt";
    //costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
        //                                     map_path);
    //charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

    //costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();
chassis_executor_ptr_=std::make_shared<ChassisExecutor>();
    roborts_decision::DecisionConfig decision_config;
    roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

//是否被呼叫的订阅者
    ros::NodeHandle order_nh;
    order_sub = order_nh.subscribe<roborts_msgs::Order>("order",30, &Blackboard::OrderCallBack, this);

    ros::NodeHandle electricity_nh;
    electricity_fb_sub_ = electricity_nh.subscribe<yhs_can_msgs::bms_flag_Infor_fb>("bms_flag_Infor_fb",30, &Blackboard::ElectricityCallBack, this);

    ros::NodeHandle bucket_nh;
    order_sub = bucket_nh.subscribe<roborts_msgs::Bucket>("bucket",30, &Blackboard::BucketCallBack, this);    


    LoadParam(proto_file_path);
  }

  ~Blackboard() = default;


  /*---------------------------------- functions ------------------------------------------*/

    int GetSearchCount(){
        return search_count_;
      }

    void SetSearchCount(unsigned int count){
        ROS_INFO("set search count: %d",search_count_);
        search_count_ = count;
      }

    int GetSearchPointSize(){
        return search_points_size_;
      }

    geometry_msgs::PoseStamped GetSearchPose(int s_count){
        ROS_INFO("get x:%f,y:%f",search_point[s_count].pose.position.x,search_point[s_count].pose.position.y);
        return search_point[s_count];
      }

std::shared_ptr<ChassisExecutor> GetChassisExecutor(){
        return chassis_executor_ptr_;
      }

 void LoadParam(const std::string &proto_file_path) {
        if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
            ROS_ERROR("Load param failed !");
            return ;
        }
          entrance_pose_.resize(decision_config.buff_point().size());
          for (int i = 0; i!= decision_config.buff_point().size(); i++) {
            entrance_pose_[i].header.frame_id = "map";
            entrance_pose_[i].pose.position.x = decision_config.buff_point(i).x();
            entrance_pose_[i].pose.position.z = decision_config.buff_point(i).z();
            entrance_pose_[i].pose.position.y = decision_config.buff_point(i).y();
            tf::Quaternion master_quaternion = tf::createQuaternionFromRPY(decision_config.buff_point(i).roll(),
                                                                          decision_config.buff_point(i).pitch(),
                                                                          decision_config.buff_point(i).yaw());
            entrance_pose_[i].pose.orientation.x = master_quaternion.x();
            entrance_pose_[i].pose.orientation.y = master_quaternion.y();
            entrance_pose_[i].pose.orientation.z = master_quaternion.z();
            entrance_pose_[i].pose.orientation.w = master_quaternion.w();
          }

          search_points_size_ = decision_config.search_region_1().size();
        search_point.resize(search_points_size_);
        for (int i = 0; i != search_points_size_; i++) {
            search_point[i].header.frame_id = "map";
            search_point[i].pose.position.x = decision_config.search_region_1(i).x();
            search_point[i].pose.position.y = decision_config.search_region_1(i).y();
            search_point[i].pose.position.z = decision_config.search_region_1(i).z();

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.search_region_1(i).roll(),
                                                                    decision_config.search_region_1(i).pitch(),
                                                                    decision_config.search_region_1(i).yaw());
            search_point[i].pose.orientation.x = quaternion.x();
            search_point[i].pose.orientation.y = quaternion.y();
            search_point[i].pose.orientation.z = quaternion.z();
            search_point[i].pose.orientation.w = quaternion.w();
           //ROS_INFO("get search x:%f,search y:%f",search_point[i].pose.position.x,search_point[i].pose.position.y); 
        }

 }


geometry_msgs::PoseStamped GetEntrancePose(){ //FIXME
  return entrance_pose_[call_number_];
}




  double GetAngle(const geometry_msgs::PoseStamped &pose1,
                  const geometry_msgs::PoseStamped &pose2) {
    const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
    const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
    tf::Quaternion rot1, rot2;
    tf::quaternionMsgToTF(quaternion1, rot1);
    tf::quaternionMsgToTF(quaternion2, rot2);
    return rot1.angleShortestPath(rot2);
  }

  const geometry_msgs::PoseStamped GetRobotMapPose() {
    UpdateRobotPose();
    return robot_map_pose_;
  }

  // const std::shared_ptr<CostMap> GetCostMap(){
  //   return costmap_ptr_;
  // }

  // const CostMap2D* GetCostMap2D() {
  //   return costmap_2d_;
  // }

  // const unsigned char* GetCharMap() {
  //   return charmap_;
  // }

void OrderCallBack(const roborts_msgs::Order::ConstPtr & order_){
  if(have_call_==0){
  have_call_ = order_->have_call;
  call_number_ = order_->number;
  }
}


bool IsHaveCall(){
  return have_call_;
}

void SetHaveCall(bool flag){
  have_call_ = flag;
}

void ElectricityCallBack(const yhs_can_msgs::bms_flag_Infor_fb::ConstPtr & electricity_){
  bms_flag_Infor_soc_ = electricity_->bms_flag_Infor_soc;
}


int GetElectricity(){
  return bms_flag_Infor_soc_;
}


bool IsLossElectricity(){
  if(bms_flag_Infor_soc_<=20)  return true;
  else return false;
}

bool GetElectricityEnableInterrupt(){
  return electricity_enable_interrupt_;
}

void SetElectricityEnableInterrupt(bool flag){
  electricity_enable_interrupt_ = flag;
}

bool IsHaveTask(){
  return is_have_task_;
}

void SetHaveTask(bool flag){
  is_have_task_ = flag;
}

bool IsGoEntrance(){
  return is_go_entrance_;
}

void SetGoEntrance(bool flag){
  is_go_entrance_ = flag;
}

bool IsGoLoad(){
  return is_go_load_;
}

void SetGoLoad(bool flag){
  is_go_load_ = flag;
}

bool IsGoUnload(){
  return is_go_unload_;
}

void SetGoUnload(bool flag){
  is_go_unload_ = flag;
}

bool IsLoadSoil(){
  return is_load_soil_;
}


void SetUnloadSoil(bool flag){
  is_unload_soil_ = flag;
}

bool IsUnloadSoil(){
  return is_unload_soil_;
}


void SetLoadSoil(bool flag){
  is_load_soil_ = flag;
}

void BucketCallBack(const roborts_msgs::Bucket::ConstPtr & bucket){
  is_lift_ = bucket->is_lift;
}

bool IsLift(){
  return is_lift_;
}

void SetRunningState(int state){
  running_state = state;
}

int GetRunningState(){
  return running_state;
}

bool FinishLoadingIsLift(){
  return finish_loading_is_lift_;
}

void SetLoadingLift(bool flag){
  finish_loading_is_lift_ = flag;
}


 private:
  void UpdateRobotPose() {
    tf::Stamped<tf::Pose> robot_tf_pose;
    robot_tf_pose.setIdentity();

    robot_tf_pose.frame_id_ = "base_footprint";
    robot_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped robot_pose;
      tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
      tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up robot pose: %s", ex.what());
    }
  }


  /*---------------------------------- Variable definition ------------------------------------------*/

  std::shared_ptr<tf::TransformListener> tf_ptr_;
  // std::shared_ptr<CostMap> costmap_ptr_;
  // CostMap2D* costmap_2d_;
  // unsigned char* charmap_;
  const std::string &proto_file_path_;
  geometry_msgs::PoseStamped robot_map_pose_;
  std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
  roborts_decision::DecisionConfig decision_config;
  std::vector<geometry_msgs::PoseStamped> entrance_pose_;  
  ros::Subscriber order_sub;
  ros::Subscriber electricity_fb_sub_;
  
unsigned int search_count_;
unsigned int search_points_size_;
std::vector<geometry_msgs::PoseStamped> search_point;
bool have_call_;
int call_number_;
int bms_flag_Infor_soc_;
bool electricity_enable_interrupt_;
bool is_have_task_;
bool is_go_entrance_;
bool is_go_load_;
bool is_load_soil_;
bool is_lift_;
bool is_go_unload_;
bool is_unload_soil_;
/*
0:闲置
1:前往坑入口
2:入坑装土
3:出坑卸土
4:充电中
*/
int running_state;
/*
装土结束后是否升斗
*/
bool finish_loading_is_lift_;
};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
