#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    // we need one thread with the instructions to make the robot move, 
    // and one thread to keep the node alive and spinning

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto spinner = std::thread([&executor]() { executor.spin(); });

    // 'arm' is the name of the MoveIt planning group for the robot's arm
    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    // named goal: the goal is specified in terms of a named pose


    // 1. set starting state
    // make sure moveit starts from the current state of the robot
    arm.setStartStateToCurrentState();

    // 2. set goal state
    arm.setNamedTarget("pose_1");

    // 3. plan
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success1 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // 4. execute
    if (success1) {
        RCLCPP_INFO(node->get_logger(), "Plan 1 successful, executing...");
        arm.execute(plan1);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Plan 1 failed!");
    }


    // alternate between home and pose1

    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success2) {
        RCLCPP_INFO(node->get_logger(), "Plan 2 successful, executing...");
        arm.execute(plan2);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Plan 2 failed!");
    }



    rclcpp::shutdown();
    spinner.join();
    return 0;
}