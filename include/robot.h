#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "my_behavior_tree/TakePackage.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
namespace RobotBTNodes
{

class SayHi : public BT::SyncActionNode
{
    public:
    ros::NodeHandle nh_;
    bool clbk;
    SayHi(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }
    BT::NodeStatus tick() override;
    void chatterCallback(const std_msgs::String::ConstPtr& msg);
};

BT::NodeStatus CheckBattery();  // another way to define node

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

class ArrivedPoint : public BT::ConditionNode
{
    public:
    ArrivedPoint(const std::string& name) : BT::ConditionNode(name, {})
    {
    }
    BT::NodeStatus tick() override;
};

class ReceivePackage : public BT::ConditionNode
{
    public:
    ros::NodeHandle nh_;
    ros::ServiceServer srv;
    bool taken;
    ReceivePackage(const std::string& name) : BT::ConditionNode(name, {})
    {
        srv = nh_.advertiseService("take_package", &ReceivePackage::package_clbk, this);
        ROS_WARN("[ReceivePackage]: /take_package service is ready");
    }
    BT::NodeStatus tick() override;
    bool package_clbk(my_behavior_tree::TakePackage::Request &req,
                      my_behavior_tree::TakePackage::Response &res);
};

class GoBack : public BT::StatefulActionNode
{
    public:
    MoveBaseClient client_;
    move_base_msgs::MoveBaseGoal goal_;

    GoBack(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
};

}  // end namespace