#include "delivery.h"
#include "direction.h"

bool get_goal = false;

namespace Reception
{

GetGoalPose::GetGoalPose(const std::string& name, const BT::NodeConfiguration& config):
        BT::SyncActionNode(name, config)
{
    go_to_goal_srv = nh.advertiseService("/direction/go_to_goal_pose", &GetGoalPose::go_to_goal_clbk, this);
    ROS_WARN("/direction/go_to_goal_pose service is ready");
}

BT::NodeStatus GetGoalPose::tick()
{
    get_goal = false;
    std::cout << "Where do you want to go to?" << std::endl;
    while(!get_goal)
    {
        ros::spinOnce();
    }
    
    setOutput<geometry_msgs::Pose>("goal_pose", pose);
    std::cout << "Location confirmed, follow me" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

bool GetGoalPose::go_to_goal_clbk(my_behavior_tree::GoToPose::Request &req,
                      my_behavior_tree::GoToPose::Response &res)
{
    pose.position.x = req.goal_pose.x;
    pose.position.y = req.goal_pose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, req.goal_pose.yaw);
    q.normalize();
    pose.orientation = tf2::toMsg(q);

    res.success = true;
    get_goal = true;
    return true;
}

BT::PortsList GetGoalPose::providedPorts()
{
   return { BT::OutputPort<geometry_msgs::Pose>("goal_pose") };
}

}  // end namespace