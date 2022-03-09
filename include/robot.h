#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "my_behavior_tree/TakePackage.h"
#include "my_behavior_tree/SetDeliveryPoint.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace RobotBTNodes
{

class Initialize : public BT::SyncActionNode
{
public:
    ros::NodeHandle nh_;
    ros::ServiceServer take_package_srv, delivery_point_srv;
    Initialize(const std::string& name);
    BT::NodeStatus tick() override;
    bool take_package_clbk(my_behavior_tree::TakePackage::Request &req,
                      my_behavior_tree::TakePackage::Response &res);
    bool delivery_point_clbk(my_behavior_tree::SetDeliveryPoint::Request &req,
                      my_behavior_tree::SetDeliveryPoint::Response &res);
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