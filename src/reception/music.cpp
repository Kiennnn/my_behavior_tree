#include "music.h"

namespace Reception
{

// Play music
Music::Music(const std::string& name): BT::SyncActionNode(name, {})
{
   std::cout << "Music mode" << std::endl;
}

BT::NodeStatus Music::tick()
{
   std::cout << "Doan quan Viet Nam di..." << std::endl;
   return BT::NodeStatus::SUCCESS;
}

}  // end namespace