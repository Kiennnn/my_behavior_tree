#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
 
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
    ReceivePackage(const std::string& name) : BT::ConditionNode(name, {})
    {
    }
    BT::NodeStatus tick() override;
};

class GoBack : public BT::ConditionNode
{
    public:
    GoBack(const std::string& name) : BT::ConditionNode(name, {})
    {
    }
    BT::NodeStatus tick() override;
};

}  // end namespace