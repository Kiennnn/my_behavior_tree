#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "delivery.h"
#include "music.h"
#include "time_keeping.h"
#include "direction.h"
#include "charge.h"
#include "cruise.h"
#include "std_msgs/String.h"

static const char* xml_text = R"(
    <root main_tree_to_execute="MainTree" >

        <BehaviorTree ID="Delivery">
            <Sequence name="delivery_sequence">
                <Initialize         name="initialize"/>
                <CheckBattery       name="check_battery"/>
                <ChoosePath         name="choose_path"/>
                <GetNumOfPoses      name="get_number_of_poses"  num_cycles="{loops}"/>
                <Repeat num_cycles="{loops}">
                    <Sequence name="moving_sequence">
                        <GetDeliveryPoint   name="get_delivery_point"   pose="{pose_value}"/>
                        <MoveToPoint        name="move_to_point"        pose="{pose_value}"/>
                        <GetTray            name="get_tray"             tray="{tray_number}"/>
                        <ReceivePackage     name="receive_package"      tray="{tray_number}"/>
                    </Sequence>
                </Repeat>
                <MoveToPoint    name="move_to_point"    pose="0;0;0"/>
            </Sequence>
        </BehaviorTree>

        <BehaviorTree ID="Cruise">
            <Sequence name="cruise_sequence">
                <InitCruise     name="init_cruise"/>
                <CruisePath     name="cruise_path"/>
                <GetNumLoops    name="get_num_loops"    loops="{cruise_loop}"/>
                <Repeat num_cycles="{cruise_loop}">
                    <Sequence   name="loop_sequence">
                        <GetNumCruisePoses  name="get_num_cruise_poses"     num_of_poses="{num_cruise_poses}"/>
                        <Repeat num_cycles="{num_cruise_poses}">
                            <Sequence name="cruise_mode_sequence">
                                <GetCruisePose  name="get_cruise_pose"  pose="{cruise_pose}"/>
                                <CruiseMove     name="cruise_move"      pose="{cruise_pose}"/>
                            </Sequence>
                        </Repeat>
                    </Sequence>
                </Repeat>
                <MoveToPoint    name="move_to_point"    pose="0;0;0"/>
            </Sequence>
        </BehaviorTree>

        <BehaviorTree ID="Charge">
            <Sequence name="charge_sequence">
                <GetChargeStation   name="get_charge_station"   station="{charging_station}"/>
                <MoveToPoint        name="move_to_point"        pose="{charging_station}"/>
            </Sequence>
        </BehaviorTree>

        <BehaviorTree ID="Direction">
            <Sequence name="direction_sequence">
                <GetGoalPose        name="get_goal_pose"    goal_pose="{goal_pose}"/>
                <MoveToPoint        name="move_to_point"    pose="{goal_pose}"/>
            </Sequence>
        </BehaviorTree>


        <BehaviorTree ID="Control">
            <Fallback name="control_fallback">
                <SubTree ID="Cruise"/>
                <SubTree ID="Delivery"/>
                <SubTree ID="Charge"/>
            </Fallback>
        </BehaviorTree>

        <BehaviorTree ID="Reception">
            <Fallback name="reception_fallback">
                <TimeKeeping    name="time_keeping"/>
                <Music          name="music"/>
                <SubTree        ID="Direction"/>
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

    factory.registerNodeType<Control::InitCruise>("InitCruise");
    factory.registerNodeType<Control::CruisePath>("CruisePath");
    factory.registerNodeType<Control::GetNumLoops>("GetNumLoops");
    factory.registerNodeType<Control::GetNumCruisePoses>("GetNumCruisePoses");
    factory.registerNodeType<Control::GetCruisePose>("GetCruisePose");
    factory.registerNodeType<Control::CruiseMove>("CruiseMove");

    factory.registerNodeType<Control::GetChargeStation>("GetChargeStation");

    factory.registerNodeType<Reception::TimeKeeping>("TimeKeeping");
    factory.registerNodeType<Reception::Music>("Music");
    factory.registerNodeType<Reception::GetGoalPose>("GetGoalPose");

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