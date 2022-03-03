#include "behaviortree_cpp_v3/bt_factory.h"
#include "dummy.h"
#include "ros/ros.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "time.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

using namespace BT;
using namespace std;

/** Behavior Tree are used to create a logic to decide what
 * to "do" and when. For this reason, our main building blocks are
 * Actions and Conditions.
 *
 * In this tutorial, we will learn how to create custom ActionNodes.
 * It is important to remember that NodeTree are just a way to
 * invoke callbacks (called tick() ). These callbacks are implemented by the user.
 */

// clang-format off
static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <CheckBattery   name="battery_ok"/>
            <OpenGripper    name="open_gripper"/>
            <ApproachObject name="approach_object"/>
            <CloseGripper   name="close_gripper"/>
        </Sequence>
     </BehaviorTree>

 </root>
 )";

// clang-format on

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "my_1st_bt");
    ros::NodeHandle nh;
    Tree tree;
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    /* There are two ways to register nodes:
    *    - statically, i.e. registering all the nodes one by one.
    *    - dynamically, loading the TreeNodes from a shared library (plugin).
    * */

    // Note: the name used to register should be the same used in the XML.
    // Note that the same operations could be done using DummyNodes::RegisterNodes(factory)

    using namespace DummyNodes;

    // The recommended way to create a Node is through inheritance.
    // Even if it requires more boilerplate, it allows you to use more functionalities
    // like ports (we will discuss this in future tutorials).
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    //You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));

    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
    tree = factory.createTreeFromText(xml_text);
    BT::StdCoutLogger logger_cout(tree);

    PublisherZMQ publisher_zmq(tree);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING)
    {
        status = tree.tickRoot();
        ros::Duration(0.5).sleep();
    }

    return 0;
}

/* Expected output:
*
       [ Battery: OK ]
       GripperInterface::open
       ApproachObject: approach_object
       GripperInterface::close
*/