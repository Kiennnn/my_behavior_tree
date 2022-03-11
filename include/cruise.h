#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include "my_behavior_tree/CruiseMode.h"
#include "my_behavior_tree/PauseRequest.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace Control
{

class InitCruise : public BT::SyncActionNode
{
public:
    ros::NodeHandle nh_;
    ros::ServiceServer pause_srv, cruise_path_srv;
    InitCruise(const std::string& name);
    BT::NodeStatus tick() override;
    bool cruise_path_clbk(my_behavior_tree::CruiseMode::Request &req,
                      my_behavior_tree::CruiseMode::Response &res);
    bool pause_clbk(my_behavior_tree::PauseRequest::Request &req,
                      my_behavior_tree::PauseRequest::Response &res);
};

class CruisePath : public BT::ConditionNode
{
public:
    CruisePath(const std::string& name);
    BT::NodeStatus tick() override;
};

class GetNumLoops : public BT::SyncActionNode
{
public:
    GetNumLoops(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class GetNumCruisePoses : public BT::SyncActionNode
{
public:
    GetNumCruisePoses(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class GetCruisePose : public BT::SyncActionNode
{
public:
    GetCruisePose(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

class CruiseMove : public BT::StatefulActionNode
{
public:
    MoveBaseClient client_;
    move_base_msgs::MoveBaseGoal goal_;

    CruiseMove(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
};

}  // end namespace