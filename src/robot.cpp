#include "robot.h"
#include <chrono>
#include <thread>
 
namespace RobotBTNodes
{

BT::NodeStatus SayHi::tick()
{
   std::cout << "[SayHi]: Hello human" << std::endl;
   std::this_thread::sleep_for(std::chrono::milliseconds(5000));
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckBattery()
{
   std::cout << "[Battery]: Fully Charged" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveToPoint::tick()
{
   std::cout << "[MoveToPoint]: Moving to point" << std::endl;
   std::this_thread::sleep_for(std::chrono::milliseconds(5000));
   return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckDoor::tick()
{
    std::cout << "[CheckDoor]: Checking door is opened or closed" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus OpenDoor::tick()
{
   std::cout << "[OpenDoor]: Opening door" << std::endl;
   return BT::NodeStatus::RUNNING;
}
 
}  // end namespace