#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "robot.h"

static const char* xml_text = R"(
    <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <SayHi          name = "say_hi"/>
                <CheckBattery   name = "check_battery"/>
                <MoveToPoint    name = "move_to_point"/>
                <Fallback>
                    <CheckDoor  name = "check_door"/>
                    <OpenDoor   name = "open_door"/>
                </Fallback>
            </Sequence>
        </BehaviorTree>
    </root>
)";

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle nh;

    // Setup BT
    BT::Tree tree;
    BT::BehaviorTreeFactory factory;

    // Register nodes
    factory.registerSimpleCondition("CheckBattery", std::bind(RobotBTNodes::CheckBattery));
    factory.registerNodeType<RobotBTNodes::SayHi>("SayHi");
    factory.registerNodeType<RobotBTNodes::MoveToPoint>("MoveToPoint");
    factory.registerNodeType<RobotBTNodes::CheckDoor>("CheckDoor");
    factory.registerNodeType<RobotBTNodes::OpenDoor>("OpenDoor");

    // Create tree
    tree = factory.createTreeFromText(xml_text);
    

    // Visualize in Groot
    BT::PublisherZMQ publisher_zmq(tree);

    // Wait till finish executing
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING)
    {
        status = tree.tickRoot();
        ros::Duration(0.5).sleep();
    }
    return 0;
}