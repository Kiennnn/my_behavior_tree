#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "delivery.h"
#include "music.h"
#include "time_keeping.h"
#include "std_msgs/String.h"

static const char* xml_text = R"(
    <root main_tree_to_execute="MainTree" >

        <BehaviorTree ID="Delivery">
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
                        <ReceivePackage     name="receive_package"      tray="{tray_number}"/>
                    </Sequence>
                </Repeat>
                <MoveToPoint    name="move_to_point"    pose="0;0;0"/>
            </Sequence>
        </BehaviorTree>


        <BehaviorTree ID="Control">
            <Fallback name="control_fallback">
                <SubTree ID="Delivery"/>
            </Fallback>
        </BehaviorTree>

        <BehaviorTree ID="Reception">
            <Fallback name="reception_fallback">
                <TimeKeeping    name="time_keeping"/>
                <Music          name="music"/>
            </Fallback>
        </BehaviorTree>

        <BehaviorTree ID="MainTree">
            <Fallback name="root_fallback">
                <SubTree ID="Control"/>
                <SubTree ID="Reception"/>
            </Fallback>
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
    factory.registerNodeType<Control::Initialize>("Initialize");
    factory.registerSimpleCondition("CheckBattery", std::bind(Control::CheckBattery));
    factory.registerNodeType<Control::ChoosePath>("ChoosePath");
    factory.registerNodeType<Control::GetNumOfPoses>("GetNumOfPoses");
    factory.registerNodeType<Control::GetDeliveryPoint>("GetDeliveryPoint");
    factory.registerNodeType<Control::MoveToPoint>("MoveToPoint");
    factory.registerNodeType<Control::GetTray>("GetTray");
    factory.registerNodeType<Control::ReceivePackage>("ReceivePackage");
    factory.registerNodeType<Reception::TimeKeeping>("TimeKeeping");
    factory.registerNodeType<Reception::Music>("Music");

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