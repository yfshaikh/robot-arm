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

    // pose goal: the goal is specified in terms of a pose

    // create a quaternion from roll, pitch, yaw
    // rpy is easy for humans to understand, but quaternions are easier for computers to work with
    tf2::Quaternion q;
    q.setRPY(3.14, 0, 0);
    q = q.normalize();


    // a pose is specified in terms of a position and an orientation
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    // 1. set starting state
    arm.setStartStateToCurrentState();

    // 2. set goal state
    arm.setPoseTarget(target_pose);

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


    // cartesian path: the goal is specified in terms of a series of waypoints

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z -= 0.2;  // down
    waypoints.push_back(pose1);

    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.y += 0.2;  
    waypoints.push_back(pose2);

    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.y -= 0.2;  
    pose3.position.z += 0.2;
    waypoints.push_back(pose3);

    moveit_msgs::msg::RobotTrajectory trajectory;

    // 0.01 - only allow 1cm of error in cartesian translation
    // fraction - how much of the path was followed successfully
    double fraction = arm.computeCartesianPath(waypoints, 0.01, trajectory);

    if (fraction == 1.0) {
        RCLCPP_INFO(node->get_logger(), "Cartesian path successful, executing...");
        arm.execute(trajectory);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Cartesian path failed!");
    }



    rclcpp::shutdown();
    spinner.join();
    return 0;
}