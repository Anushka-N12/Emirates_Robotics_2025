#include "behaviortree_cpp_v3/action_node.h"
#include <chrono>
#include <iostream>
#include <thread>

// Define the FinalAction node
class FinalAction : public BT::SyncActionNode {
public:
  FinalAction(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  // Overriding the tick method to perform the action
  BT::NodeStatus tick() override {
    std::cout << "Performing final action: double spin" << std::endl;

    // Simulate a double spin action
    performDoubleSpin();
    return BT::NodeStatus::SUCCESS;
  }

  // Define the node configuration
  static BT::PortsList providedPorts() { return {}; }

private:
  void performDoubleSpin() {
    std::cout << "Spinning 360 degrees (first spin)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3)); // Simulate first spin
    std::cout << "Spinning 360 degrees (second spin)..." << std::endl;
    std::this_thread::sleep_for(
        std::chrono::seconds(3)); // Simulate second spin
    std::cout << "Final spin action completed!" << std::endl;
  }
};
