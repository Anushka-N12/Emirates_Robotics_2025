#include "behaviortree_cpp_v3/bt_factory.h"
#include <iostream>

using namespace BT;

// Define the XML tree structure
static const char *xml_text = R"(
<root main_tree_to_execute = "MainTree" >

    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Sequence name="waypoints_and_spin">
                <NavigateToWaypoint name="waypoint_1" goal="{waypoint_1}"/>
                <PerformSpin name="spin_at_waypoint_1"/>
                <NavigateToWaypoint name="waypoint_2" goal="{waypoint_2}"/>
                <PerformSpin name="spin_at_waypoint_2"/>
                <NavigateToWaypoint name="waypoint_3" goal="{waypoint_3}"/>
                <PerformSpin name="spin_at_waypoint_3"/>
            </Sequence>
            <FinalAction name="final_action"/>
        </Sequence>
    </BehaviorTree>
</root>
)";

// Main function
int main() {
  BehaviorTreeFactory factory;

  // Register your custom nodes here
  factory.registerNodeType<NavigateToWaypoint>("NavigateToWaypoint");
  factory.registerNodeType<PerformSpin>("PerformSpin");
  factory.registerNodeType<FinalAction>("FinalAction");

  // Create the tree from the XML
  auto tree = factory.createTreeFromText(xml_text);

  // Tick the root node (execute the tree)
  while (true) {
    NodeStatus status = tree.tickRoot();
    if (status == NodeStatus::SUCCESS || status == NodeStatus::FAILURE) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
