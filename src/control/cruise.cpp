#include "cruise.h"
#include "delivery.h"
#include "my_behavior_tree/OfficePose.h"

bool choose_cruise_path = false;
bool pause_cruise = false;
my_behavior_tree::OfficePose *path;
int cruise_path_index = 0;
int loop;
int num_of_cruise_pose;
int cruise_timeout = 10;  //seconds

namespace Control
{

// Initialize
InitCruise::InitCruise(const std::string& name) : BT::SyncActionNode(name,{})
{
    cruise_path_srv = nh_.advertiseService("set_cruise_path", &InitCruise::cruise_path_clbk, this);
    ROS_WARN("[Initialize]: /set_cruise_path service is ready");
    pause_srv = nh_.advertiseService("cruise_pause", &InitCruise::pause_clbk, this);
    ROS_WARN("[Initialize]: /cruise_pause service is ready");
}

BT::NodeStatus InitCruise::tick()
{
   std::cout << "Entering Cruise mode" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

bool InitCruise::cruise_path_clbk(my_behavior_tree::CruiseMode::Request &req,
                      my_behavior_tree::CruiseMode::Response &res)
{
    loop = req.loop;
    num_of_cruise_pose = req.number_of_poses;
    cruise_timeout = req.timeout;
    path = new my_behavior_tree::OfficePose[num_of_cruise_pose];
    for(int i=0; i<num_of_cruise_pose; i++)
    {
        *(path+i) = req.path[i];
    }
    res.success = true;
    choose_cruise_path = true;
    return true;
}

bool InitCruise::pause_clbk(my_behavior_tree::PauseRequest::Request &req,
                      my_behavior_tree::PauseRequest::Response &res)
{
    if(req.pause)
    {
        pause_cruise = true;
        res.success = true;
        return true;
    }
    else
    {
        return false;
    }
}

// Choose cruise path
CruisePath::CruisePath (const std::string& name) : BT::ConditionNode(name, {})
{
}

BT::NodeStatus CruisePath::tick()
{
    std::cout << "Choose your cruise path" << std::endl;
    while (!choose_cruise_path)
    {
        ros::spinOnce();
    }
    std::cout << "Path is completed, moving..." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// Get number of loops
GetNumLoops::GetNumLoops(const std::string& name, const BT::NodeConfiguration& config) :
                    BT::SyncActionNode(name, config)
{ 
}

BT::NodeStatus GetNumLoops::tick()
{
    setOutput<int>("loops", loop);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList GetNumLoops::providedPorts()
{
    return { BT::OutputPort<int>("loops") };
}

// Get number of poses
GetNumCruisePoses::GetNumCruisePoses(const std::string& name, const BT::NodeConfiguration& config) :
                    BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetNumCruisePoses::tick()
{
    setOutput<int>("num_of_poses", num_of_cruise_pose);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList GetNumCruisePoses::providedPorts()
{
    return { BT::OutputPort<int>("num_of_poses") };
}

// Get cruise pose one by one
GetCruisePose::GetCruisePose(const std::string& name, const BT::NodeConfiguration& config) :
                    BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetCruisePose::tick()
{
    geometry_msgs::Pose cruise_pose;
    cruise_pose.position.x = (path + cruise_path_index)->x;
    cruise_pose.position.y = (path + cruise_path_index)->y;
    tf2::Quaternion q;
    q.setRPY(0, 0, (path + cruise_path_index)->yaw);
    q.normalize();
    cruise_pose.orientation = tf2::toMsg(q);
    setOutput<geometry_msgs::Pose>("pose", cruise_pose);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList GetCruisePose::providedPorts()
{
    return { BT::OutputPort<geometry_msgs::Pose>("pose") };
}

// Call move base action
CruiseMove::CruiseMove(const std::string& name, const BT::NodeConfiguration& config) :
    BT::StatefulActionNode(name, config),
    client_("move_base", true)
{
   client_.waitForServer();
   ROS_WARN("[CruiseMove]: Move Base server ready");
}

BT::NodeStatus CruiseMove::onStart()
{
   auto res = getInput<geometry_msgs::Pose>("pose");
   if(!res)
   {
      throw BT::RuntimeError("error reading port [target]:", res.error());
   }
   std::cout << "[" << this->name() << "] Sending goal..." << std::endl;
   geometry_msgs::Pose pose = res.value();
   goal_.target_pose.header.frame_id = "map";
   goal_.target_pose.pose.position.x = pose.position.x;
   goal_.target_pose.pose.position.y = pose.position.y;
   goal_.target_pose.pose.orientation = pose.orientation;
   client_.sendGoal(goal_);
   return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CruiseMove::onRunning()
{
   actionlib::SimpleClientGoalState state = client_.getState();
   if (state == actionlib::SimpleClientGoalState::SUCCEEDED){
      std::cout << "[" << this->name() << "] Goal reached" << std::endl;

        // End of path
        if(cruise_path_index == num_of_cruise_pose-1)
        {
            cruise_path_index = 0;
            loop--;
        }
        else
        {
            cruise_path_index++;
        }

        // End of loops
        if(loop==0)
        {
            delete[] path;
            choose_cruise_path = false;
        }

      return BT::NodeStatus::SUCCESS;
   } else if (state == actionlib::SimpleClientGoalState::ACTIVE) {
      return BT::NodeStatus::RUNNING;
   } else {
      std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
      return BT::NodeStatus::FAILURE;
   }
}

void CruiseMove::onHalted() {};

BT::PortsList CruiseMove::providedPorts() {
    return { BT::InputPort<geometry_msgs::Pose>("pose") };
}

} // end namespace