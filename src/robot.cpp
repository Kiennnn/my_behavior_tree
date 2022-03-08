#include "robot.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"

bool pkg_taken = false;

// Template specialization to converts a string to geometry_msgs/Pose
template <> inline geometry_msgs::Pose BT::convertFromString(BT::StringView str)
{
   // We expect real numbers separated by semicolons
   auto parts = splitString(str, ';');
   if (parts.size() != 3)
   {
      throw BT::RuntimeError("invalid input)");
   }
   else
   {
      geometry_msgs::Pose output;
      output.position.x = convertFromString<double>(parts[0]);
      output.position.y = convertFromString<double>(parts[1]);
      double yaw = convertFromString<double>(parts[2]);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      q.normalize();
      output.orientation = tf2::toMsg(q);
      return output;
   }
}

namespace RobotBTNodes
{

// Starting up node
SayHi::SayHi(const std::string& name) : BT::SyncActionNode(name, {})
{
   srv = nh_.advertiseService("take_package", &SayHi::package_clbk, this);
   ROS_WARN("[ReceivePackage]: /take_package service is ready");
}

BT::NodeStatus SayHi::tick()
{
   std::cout << "[SayHi]: Starting up..." << std::endl;
   ros::Duration(3.0).sleep();
   std::cout << "[SayHi]: Hello human!" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

bool SayHi::package_clbk(my_behavior_tree::TakePackage::Request &req,
                      my_behavior_tree::TakePackage::Response &res)
{
   if(req.take)
   {
      res.success = true;
      pkg_taken = true;
      return true;
   }
   else
   {
      return false;
   }
}

// Check battery
BT::NodeStatus CheckBattery()
{
   std::cout << "[CheckBattery]: Checking battery..." << std::endl;
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
   ROS_WARN("[MoveToPoint]: Move Base server ready");
}

BT::NodeStatus MoveToPoint::onStart()
{
   auto res = getInput<geometry_msgs::Pose>("pose");
   if(!res)
   {
      throw BT::RuntimeError("error reading port [target]:", res.error());
   }
   std::cout << "[" << this->name() << "] Sending goal..." << std::endl;
   geometry_msgs::Pose pose = res.value();
   goal_.target_pose.header.frame_id = "map";
   goal_.target_pose.pose.position.x = pose.position.x;
   goal_.target_pose.pose.position.y = pose.position.y;
   goal_.target_pose.pose.orientation = pose.orientation;
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
    return { BT::InputPort<geometry_msgs::Pose>("pose") };
}

// Check if package was taken
ReceivePackage::ReceivePackage(const std::string& name, const BT::NodeConfiguration& config) :
    BT::ConditionNode(name, config)
{
}

BT::NodeStatus ReceivePackage::tick()
{
   BT::Optional<std::string> tray = getInput<std::string>("tray");
   pkg_taken = false;
   std::cout << "[ReceivePackage]: Take your package at tray number " << tray.value() << std::endl;
   while (!pkg_taken)
   {
      ros::spinOnce();
   }
   std::cout << "[ReceivePackage]: Package is taken" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

BT::PortsList ReceivePackage::providedPorts() {
    return { BT::InputPort<std::string>("tray") };
}

}  // end namespace