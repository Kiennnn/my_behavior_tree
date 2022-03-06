#include "robot.h"
#include "ros/ros.h"
#include "ros/console.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 
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
    BT::StatefulActionNode(name, config),
    client_("move_base", true)
{
   client_.waitForServer();
   ROS_WARN("[MoveToPoint]: Constructor");
}

BT::NodeStatus MoveToPoint::onStart()
{
   std::cout << "[" << this->name() << "] Sending goal..." << std::endl;
   goal_.target_pose.header.frame_id = "map";
   goal_.target_pose.pose.position.x = 2;
   goal_.target_pose.pose.position.y = 0;
   tf2::Quaternion q;
   q.setRPY(0, 0, 0);
   q.normalize();
   goal_.target_pose.pose.orientation = tf2::toMsg(q);
   client_.sendGoal(goal_);
   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToPoint::onRunning()
{
   actionlib::SimpleClientGoalState state = client_.getState();
   if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      std::cout << "[" << this->name() << "] Goal reached" << std::endl;
      return BT::NodeStatus::SUCCESS;
   } else if (state == actionlib::SimpleClientGoalState::ACTIVE) {
      return BT::NodeStatus::RUNNING;
   } else {
      std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
      return BT::NodeStatus::FAILURE;
   }
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
GoBack::GoBack(const std::string& name, const BT::NodeConfiguration& config) :
    BT::StatefulActionNode(name, config),
    client_("move_base", true)
{
   client_.waitForServer();
   ROS_WARN("[MoveToPoint]: Constructor");
}

BT::NodeStatus GoBack::onStart()
{
   std::cout << "[" << this->name() << "] Sending goal..." << std::endl;
   goal_.target_pose.header.frame_id = "map";
   goal_.target_pose.pose.position.x = 4;
   goal_.target_pose.pose.position.y = 0;
   tf2::Quaternion q;
   q.setRPY(0, 0, 0);
   q.normalize();
   goal_.target_pose.pose.orientation = tf2::toMsg(q);
   client_.sendGoal(goal_);
   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoBack::onRunning()
{
   actionlib::SimpleClientGoalState state = client_.getState();
   if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      std::cout << "[" << this->name() << "] Goal reached" << std::endl;
      return BT::NodeStatus::SUCCESS;
   } else if (state == actionlib::SimpleClientGoalState::ACTIVE) {
      return BT::NodeStatus::RUNNING;
   } else {
      std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
      return BT::NodeStatus::FAILURE;
   }
}

void GoBack::onHalted() {};

BT::PortsList GoBack::providedPorts() {
    return { BT::InputPort<std::string>("loc") };
}
 
}  // end namespace