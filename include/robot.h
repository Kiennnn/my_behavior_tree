#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
 
namespace RobotBTNodes
{

class SayHi : public BT::SyncActionNode
{
    public:
    SayHi(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }
    BT::NodeStatus tick() override;
};

BT::NodeStatus CheckBattery();

class MoveToPoint : public BT::SyncActionNode
{
    public:
    MoveToPoint(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }
    BT::NodeStatus tick() override;
};

class CheckDoor : public BT::ConditionNode
{
    public:
    CheckDoor(const std::string& name) : BT::ConditionNode(name, {})
    {
    }
    BT::NodeStatus tick() override;
};

class OpenDoor : public BT::ConditionNode
{
    public:
    OpenDoor(const std::string& name) : BT::ConditionNode(name, {})
    {
    }
    BT::NodeStatus tick() override;
};

}  // end namespace