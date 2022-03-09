#include "robot.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"
#include "my_behavior_tree/DeliveryPose.h"

bool pkg_taken = false;
bool choose_path = false;
int number_of_poses;
int arr_index = 0;
my_behavior_tree::DeliveryPose *deli_pose;
int timeout = 10;  // time to wait (s)

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

// Startup node
Initialize::Initialize(const std::string& name) : BT::SyncActionNode(name, {})
{
   take_package_srv = nh_.advertiseService("take_package", &Initialize::take_package_clbk, this);
   ROS_WARN("[Initialize]: /take_package service is ready");
   delivery_point_srv = nh_.advertiseService("set_delivery_point", &Initialize::delivery_point_clbk, this);
   ROS_WARN("[Initialize]: /set_delivery_point service is ready");
}

BT::NodeStatus Initialize::tick()
{
   std::cout << "[Initialize]: Starting up..." << std::endl;
   ros::Duration(3.0).sleep();
   std::cout << "[Initialize]: Hello human!" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

bool Initialize::take_package_clbk(my_behavior_tree::TakePackage::Request &req,
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

bool Initialize::delivery_point_clbk(my_behavior_tree::SetDeliveryPoint::Request &req,
                      my_behavior_tree::SetDeliveryPoint::Response &res)
{
   ROS_WARN("Path created");
   number_of_poses = req.number_of_poses;
   deli_pose  = new my_behavior_tree::DeliveryPose[number_of_poses];
   // copy requested pose array
   for(int i=0; i<number_of_poses; i++)
   {
      *(deli_pose+i) = req.pose[i];
   }
   res.success = true;
   choose_path = true;
   return true;
}

// Check battery
BT::NodeStatus CheckBattery()
{
   std::cout << "[CheckBattery]: Checking battery..." << std::endl;
   ros::Duration(1.0).sleep();
   std::cout << "[CheckBattery]: Fully Charged" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

// Wait for choosing path process
ChoosePath::ChoosePath(const std::string& name) : BT::ConditionNode(name, {})
{
}

BT::NodeStatus ChoosePath::tick()
{
   std::cout << "[ChoosePath]: Choose your delivery path" << std::endl;
   while (!choose_path)
   {
      ros::spinOnce();
   }
   std::cout << "[ChoosePath]: Path is completed, delivering..." << std::endl;
   return BT::NodeStatus::SUCCESS;
}

// Get number of delivery point for iteration
GetNumOfPoses::GetNumOfPoses(const std::string& name, const BT::NodeConfiguration& config):
        BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetNumOfPoses::tick()
{
   setOutput<int>("num_cycles", number_of_poses);
   return BT::NodeStatus::SUCCESS;
}

BT::PortsList GetNumOfPoses::providedPorts()
{
   return { BT::OutputPort<int>("num_cycles") };
}

// Get delivery point
GetDeliveryPoint::GetDeliveryPoint(const std::string& name, const BT::NodeConfiguration& config):
        BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetDeliveryPoint::tick()
{
   geometry_msgs::Pose delivery_pose;
   delivery_pose.position.x = (deli_pose + arr_index)->x;
   delivery_pose.position.y = (deli_pose + arr_index)->y;
   tf2::Quaternion q;
   q.setRPY(0, 0, (deli_pose + arr_index)->yaw);
   q.normalize();
   delivery_pose.orientation = tf2::toMsg(q);
   setOutput<geometry_msgs::Pose>("pose", delivery_pose);
   return BT::NodeStatus::SUCCESS;
}

BT::PortsList GetDeliveryPoint::providedPorts()
{
   return { BT::OutputPort<geometry_msgs::Pose>("pose") };
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

// Get tray
GetTray::GetTray(const std::string& name, const BT::NodeConfiguration& config):
        BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetTray::tick()
{
   std::string tray_ = (deli_pose + arr_index)->tray;
   setOutput<std::string>("tray", tray_);
   return BT::NodeStatus::SUCCESS;
}

BT::PortsList GetTray::providedPorts()
{
   return { BT::OutputPort<std::string>("tray") };
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
   int count = 0;
   while (!pkg_taken && count<timeout)
   {
      ros::Duration(1).sleep();
      count++;
      ros::spinOnce();
   }
   if(count < timeout)
   {
      std::cout << "[ReceivePackage]: Package is taken" << std::endl;
   }
   else
   {
      std::cout << "Timeout, moving on..." << std::endl;
   }

   // End of loop
   if(arr_index == number_of_poses-1)
   {
      arr_index = 0;
      delete[] deli_pose;
      choose_path = false;
   }
   else
   {
      arr_index++;
   }
   return BT::NodeStatus::SUCCESS;
}

BT::PortsList ReceivePackage::providedPorts() {
    return { BT::InputPort<std::string>("tray") };
}

}  // end namespace