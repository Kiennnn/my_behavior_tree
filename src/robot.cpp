#include "robot.h"
#include "ros/ros.h"
#include "ros/console.h"
 
namespace RobotBTNodes
{

BT::NodeStatus SayHi::tick()
{
   std::cout << "[SayHi]: Starting up..." << std::endl;
   ros::Duration(3.0).sleep();
   std::cout << "[SayHi]: Hello human!" << std::endl;
   // ros::Subscriber sub = nh_.subscribe("/chatter", 1000, &SayHi::chatterCallback, this);
   // clbk = false;
   // while(!clbk){
   //    ros::spinOnce();
   // }
   return BT::NodeStatus::SUCCESS;
}

void SayHi::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
   std::cout << ("[SayHi]: ROS topic testing: I heard: [%s]", msg->data.c_str());
   clbk = true;
}

// Check battery
BT::NodeStatus CheckBattery()
{
   ros::Duration(1.0).sleep();
   std::cout << "[CheckBattery]: Fully Charged" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

// Call move base action
MoveToPoint::MoveToPoint(const std::string& name, const BT::NodeConfiguration& config) :
    BT::StatefulActionNode(name, config)
{
   ROS_WARN("[MoveToPoint]: Constructor");
}

BT::NodeStatus MoveToPoint::onStart()
{
   std::cout << "[MoveToPoint]: Ready to move" << std::endl;
   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToPoint::onRunning()
{
   std::cout << "[MoveToPoint]: Going to destination..." << std::endl;  
   ros::Duration(5.0).sleep();
   return BT::NodeStatus::SUCCESS;
}

void MoveToPoint::onHalted() {};

BT::PortsList MoveToPoint::providedPorts() {
    return { BT::InputPort<std::string>("loc") };
}

// Check if robot has arrived
BT::NodeStatus ArrivedPoint::tick()
{
    std::cout << "[ArrivedPoint]: Arrived to point" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// Check if package was taken
BT::NodeStatus ReceivePackage::tick()
{
   taken = false;
   std::cout << "[ReceivePackage]: Take your package at tray number 2" << std::endl;
   // ros::Duration(5.0).sleep();
   while (!taken)
   {
      ros::spinOnce();
   }
   std::cout << "[ReceivePackage]: Package was taken" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

bool ReceivePackage::package_clbk(my_behavior_tree::TakePackage::Request &req,
                      my_behavior_tree::TakePackage::Response &res)
{
   res.success = true;
   taken = true;
   return true;
}

// Go back to started position
BT::NodeStatus GoBack::tick()
{
   std::cout << "[GoBack]: Going back to reception desk..." << std::endl;
   ros::Duration(5.0).sleep();;
   std::cout << "[GoBack]: Arrived home" << std::endl;
   return BT::NodeStatus::SUCCESS;
}
 
}  // end namespace