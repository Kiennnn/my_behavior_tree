#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include "my_behavior_tree/GoToPose.h"

namespace Reception
{

class GetGoalPose : public BT::SyncActionNode
{
public:
    ros::NodeHandle nh;
    ros::ServiceServer go_to_goal_srv;
    geometry_msgs::Pose pose;
    GetGoalPose(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    bool go_to_goal_clbk(my_behavior_tree::GoToPose::Request &req,
                      my_behavior_tree::GoToPose::Response &res);
    static BT::PortsList providedPorts();
};

}  // end namespace