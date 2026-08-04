#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using bool = example_interfaces::msg::Bool;

class Commander {
    public:
        Commander(std::shared_ptr<rclcpp::Node> node) 
        {
            node_ = node;
            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
            // bind callback to 'this' instance of Commander class
            open_gripper_sub_ = node_->create_subscription<bool>("open_gripper", 10, std::bind(&Commander::openGripperCallback, this, std::placeholders::_1));

            arm_->setMaxVelocityScalingFactor(1.0);
            arm_->setMaxAccelerationScalingFactor(1.0);
        }

        // replicate the functionality of named_goal.cpp
        void goToNamedTarget(const std::string &target_name) {
            arm_->setStartStateToCurrentState();
            arm_->setNamedTarget(target_name);
            planAndExecute(arm_);
        }

        void goToJointTarget(const std::vector<double> &joint_values) {
            arm_->setStartStateToCurrentState();
            arm_->setJointValueTarget(joint_values);
            planAndExecute(arm_);
        }

        void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path=false) {
            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            q = q.normalize();

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "base_link";
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x = q.getX();
            target_pose.pose.orientation.y = q.getY();
            target_pose.pose.orientation.z = q.getZ();
            target_pose.pose.orientation.w = q.getW();

            if (!cartesian_path) {
                arm_->setStartStateToCurrentState();
                arm_->setPoseTarget(target_pose);
                planAndExecute(arm_);
            } else {
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                moveit_msgs::msg::RobotTrajectory trajectory;

                double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);

                if (fraction == 1) {
                    arm_->execute(trajectory);
                }
            }
        }

        void openGripper() {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("open");
            planAndExecute(gripper_);
        }

        void closeGripper() {
            gripper_->setStartStateToCurrentState();
            gripper_->setNamedTarget("close");
            planAndExecute(gripper_);
        }
        
    private:
        void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface) {
            MoveGroupeInterface::Plan plan;
            bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success) {
                RCLCPP_INFO(node_->get_logger(), "Plan successful, executing...");
                interface->execute(plan);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Plan failed!");
            }
        }

        void openGripperCallback(const Bool &msg) {
            if (msg.data) {
                openGripper();
            } else {
                closeGripper();
            }
        }
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        std::shared_ptr<MoveGroupInterface> gripper_;

        rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
};

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);





    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}