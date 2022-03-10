#include "time_keeping.h"

namespace Reception
{

// TimeKeeping
TimeKeeping::TimeKeeping(const std::string& name): BT::SyncActionNode(name, {})
{
   std::cout << "TimeKeeping mode" << std::endl;
}

BT::NodeStatus TimeKeeping::tick()
{
   std::cout << "Chao dong chi Tran Trung Kien" << std::endl;
   return BT::NodeStatus::SUCCESS;
}

}  // end namespace