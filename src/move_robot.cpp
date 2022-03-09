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
                <Initialize         name="initialize"/>
                <CheckBattery       name="check_battery"/>
                <ChoosePath         name="choose_path"/>
                <GetNumOfPoses      name="get_number_of_poses"  num_cycles="{loops}"/>
                <Repeat num_cycles="{loops}">
                    <Sequence name="delivery_sequence">
                        <GetDeliveryPoint   name="get_delivery_point"   pose="{pose_value}"/>
                        <MoveToPoint        name="move_to_point"        pose="{pose_value}"/>
                        <GetTray            name="get_tray"             tray="{tray_number}"/>
                        <Fallback>
                            <ReceivePackage     name="receive_package"      tray="{tray_number}"/>
                        </Fallback>
                    </Sequence>
                </Repeat>
                <MoveToPoint    name="move_to_point"    pose="0;0;0"/>
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
    factory.registerNodeType<RobotBTNodes::Initialize>("Initialize");
    factory.registerSimpleCondition("CheckBattery", std::bind(RobotBTNodes::CheckBattery));
    factory.registerNodeType<RobotBTNodes::ChoosePath>("ChoosePath");
    factory.registerNodeType<RobotBTNodes::GetNumOfPoses>("GetNumOfPoses");
    factory.registerNodeType<RobotBTNodes::GetDeliveryPoint>("GetDeliveryPoint");
    factory.registerNodeType<RobotBTNodes::MoveToPoint>("MoveToPoint");
    factory.registerNodeType<RobotBTNodes::GetTray>("GetTray");
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