#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"

namespace Reception
{

class TimeKeeping : public BT::SyncActionNode
{
public:
    TimeKeeping(const std::string& name);
    BT::NodeStatus tick() override;
};

}  // end namespace