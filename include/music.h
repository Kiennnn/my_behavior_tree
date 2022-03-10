#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"

namespace Reception
{

class Music : public BT::SyncActionNode
{
public:
    Music(const std::string& name);
    BT::NodeStatus tick() override;
};

}  // end namespace