#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_tree.h"
#include "action_node/go_loading_action.h"
#include "action_node/go_unloading_action.h"
#include "action_node/go_electricity.h"

void Command();
char command = '0';

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_node");
      ros::Time::init();
std::string file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
auto black_ptr = std::make_shared<roborts_decision::Blackboard>(file_path);
auto g_factory = std::make_shared<roborts_decision::ActionMaker>(black_ptr);


 auto go_goal=std::make_shared<roborts_decision::GoLoadingAction>(black_ptr,g_factory);    
auto fangdieluo=std::make_shared<roborts_decision::GoUnloadingAction>(black_ptr,g_factory);    
auto search=std::make_shared<roborts_decision::GoElectricity>(black_ptr,g_factory);    

auto game_status_selector=std::make_shared<roborts_decision::SelectorNode>("game_status_selector",black_ptr);


roborts_decision::BehaviorTree root(game_status_selector, 500);
 //command=argv[1][0];
  auto command_thread= std::thread(Command);
  ros::Rate rate(10);
  while(ros::ok()){
    ros::spinOnce();
    switch (command) {
      case '1':
  {  
    game_status_selector->AddChildren(go_goal);
        root.Run();
        break;
  }
      case '2':
      {
game_status_selector->AddChildren(fangdieluo);
         root.Run();
        break;
      }
      case '3':
      {
game_status_selector->AddChildren(search);
       root.Run();
        break;
      }
     case '4':
      {
geometry_msgs::PoseStamped pose;
 pose = black_ptr->GetRobotMapPose();
  std::cout <<"x:"<< pose.pose.position.x << std::endl;
  std::cout <<"y:"<< pose.pose.position.y<< std::endl;
  std::cout <<"x:"<< pose.pose.orientation.x << std::endl;
  std::cout <<"y:"<< pose.pose.orientation.y<< std::endl;
   std::cout <<"z:"<< pose.pose.orientation.z << std::endl;
  std::cout <<"w:"<< pose.pose.orientation.w<< std::endl;
        break;
      }
      case '5':
      {

           root.Run();
        break;
       }
        case '6':

           root.Run();
        break;
         case '7':

        break;
            case '8':

           root.Run();
        break;
         case '9':

           root.Run();
        break;   
          case 'a':

           root.Run(); 
        break;    
       case 'b':

           root.Run(); 
        break;  
            case 'c':

           root.Run(); 
        break;  
        /*    case 27:
                if (command_thread.joinable()){
                    command_thread.join();
                }
                return 0;*/
            default:
                break;
    }
    rate.sleep();
  }

  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: gogoal" << std::endl
              << "2: fangdieluo" << std::endl
              << "3: search" << std::endl
              << "4: get pose" << std::endl
              << "5: rolldefend" << std::endl
              << "6: swingdefend" << std::endl
              << "7: .." << std::endl
              << "8: follow" << std::endl
              << "9: chase" << std::endl 
              << "a: escape" << std::endl 
              << "b: turndefend" << std::endl        
              << "c: staticpose" << std::endl        
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != '7' 
        &&command != '8' &&command != '9' &&command != 'a'  &&command != 'b'  && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}
