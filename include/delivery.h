#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "my_behavior_tree/TakePackage.h"
#include "my_behavior_tree/SetDeliveryPoint.h"
#include "my_behavior_tree/ChooseMode.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

namespace Control
{

class ChooseMode : public BT::SyncActionNode
{
public:
    ChooseMode(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class Initialize : public BT::SyncActionNode
{
public:
    ros::NodeHandle nh_;
    ros::ServiceServer take_package_srv, delivery_point_srv, choose_mode_srv;
    Initialize(const std::string& name);
    BT::NodeStatus tick() override;
    bool take_package_clbk(my_behavior_tree::TakePackage::Request &req,
                      my_behavior_tree::TakePackage::Response &res);
    bool delivery_point_clbk(my_behavior_tree::SetDeliveryPoint::Request &req,
                      my_behavior_tree::SetDeliveryPoint::Response &res);
    bool choose_mode_clbk(my_behavior_tree::ChooseMode::Request &req,
                      my_behavior_tree::ChooseMode::Response &res);
};

BT::NodeStatus CheckBattery();  // another way to define node

class ChoosePath : public BT::ConditionNode
{
public:
    ChoosePath(const std::string& name);
    BT::NodeStatus tick() override;
};

class GetNumOfPoses : public BT::SyncActionNode
{
public:
    GetNumOfPoses(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class GetDeliveryPoint : public BT::SyncActionNode
{
public:
    GetDeliveryPoint(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class GetTray : public BT::SyncActionNode
{
public:
    GetTray(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class MoveToPoint : public BT::StatefulActionNode
{
public:
    MoveBaseClient client_;
    move_base_msgs::MoveBaseGoal goal_;

    MoveToPoint(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
};

class ReceivePackage : public BT::ConditionNode
{
public:
    ReceivePackage(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

}  // end namespace