#include "charge.h"
#include "delivery.h"

namespace Control
{

GetChargeStation::GetChargeStation(const std::string& name, const BT::NodeConfiguration& config):
        BT::SyncActionNode(name, config)
{
}

BT::NodeStatus GetChargeStation::tick()
{
   geometry_msgs::Pose delivery_pose;
   delivery_pose.position.x = 2;
   delivery_pose.position.y = 0;
   tf2::Quaternion q;
   q.setRPY(0, 0, 0);
   q.normalize();
   delivery_pose.orientation = tf2::toMsg(q);
   setOutput<geometry_msgs::Pose>("station", delivery_pose);
   std::cout << "Going back to station for charging" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

BT::PortsList GetChargeStation::providedPorts()
{
   return { BT::OutputPort<geometry_msgs::Pose>("station") };
}

}  // end namespace