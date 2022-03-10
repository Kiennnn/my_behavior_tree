#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"

namespace Control
{

class GetChargeStation : public BT::SyncActionNode
{
public:
    GetChargeStation(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

}  // end namespace