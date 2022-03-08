#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "robot.h"
#include "std_msgs/String.h"

static const char* xml_text = R"(
    <root main_tree_to_execute="MainTree" >
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <SayHi name="say_hi"/>
                <CheckBattery name="check_battery"/>
                <Repeat num_cycles="2">
                    <Sequence name="delivery_sequence">
                        <MoveToPoint name="move_to_point"    pose="1;0;1.57"/>
                        <Fallback>
                            <ReceivePackage name="receive_package"    tray="2"/>
                        </Fallback>
                    </Sequence>
                </Repeat>
                <MoveToPoint name="move_to_point"    pose="0;0;0"/>
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
    factory.registerNodeType<RobotBTNodes::SayHi>("SayHi");
    factory.registerSimpleCondition("CheckBattery", std::bind(RobotBTNodes::CheckBattery));
    factory.registerNodeType<RobotBTNodes::MoveToPoint>("MoveToPoint");
    factory.registerNodeType<RobotBTNodes::ReceivePackage>("ReceivePackage");

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