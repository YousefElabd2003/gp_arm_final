#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

#include <thread>
#include <vector>
#include <memory>
#include <cmath>

class ArmMotionTestNode : public rclcpp::Node
{
public:
    ArmMotionTestNode()
        : Node("arm_motion_test_node"),
          target_received_(false)
    {
        // Subscribe to fake lever pose topic
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/lever_pose_arm",
            10,
            std::bind(&ArmMotionTestNode::poseCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Arm Motion Test Node Started");
        RCLCPP_INFO(this->get_logger(), "Waiting for target pose on /lever_pose_arm...");
    }

    bool hasTarget() const { return target_received_; }
    geometry_msgs::msg::Pose getTarget() const { return target_pose_; }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (target_received_) return;
        
        target_pose_ = msg->pose;
        target_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Target pose received!");
        RCLCPP_INFO(this->get_logger(), "  Position: x=%.3f, y=%.3f, z=%.3f",
                    target_pose_.position.x,
                    target_pose_.position.y,
                    target_pose_.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                    target_pose_.orientation.x,
                    target_pose_.orientation.y,
                    target_pose_.orientation.z,
                    target_pose_.orientation.w);
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    geometry_msgs::msg::Pose target_pose_;
    bool target_received_;
};

// Function to add orientation constraints
void addOrientationConstraint(moveit::planning_interface::MoveGroupInterface &move_group, 
                              const geometry_msgs::msg::Pose &target_pose,
                              const std::string &frame_id = "base_link")
{
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = move_group.getEndEffectorLink();
    ocm.header.frame_id = frame_id;
    ocm.orientation = target_pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.05;
    ocm.absolute_y_axis_tolerance = 0.05;
    ocm.absolute_z_axis_tolerance = 0.05;
    ocm.weight = 1.0;
    
    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    
    move_group.setPathConstraints(constraints);
    RCLCPP_INFO(rclcpp::get_logger("arm_motion_test_node"), "Orientation constraints applied");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ArmMotionTestNode>();
    
    // Use multithreaded executor to handle both spin and MoveIt
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // Spin in a separate thread to process callbacks
    std::thread spin_thread([&executor]() {
        executor.spin();
    });
    
    // Wait for target pose
    while (rclcpp::ok() && !node->hasTarget()) {
        RCLCPP_INFO(node->get_logger(), "Waiting for target pose from /lever_pose_arm...");
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "Node shutdown before receiving target");
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Target received! Starting trajectory execution...");
    
    // Get the target pose from the subscriber
    geometry_msgs::msg::Pose lever_pose = node->getTarget();
    
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "arm_group");
    
    // =========================================================
    // CONFIGURE MOTION PARAMETERS
    // =========================================================
    
    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.01);
    
    RCLCPP_INFO(node->get_logger(), "MoveGroupInterface initialized");
    
    // Wait for robot state
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    auto current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(node->get_logger(), "Current pose: x=%.3f y=%.3f z=%.3f",
                current_pose.pose.position.x,
                current_pose.pose.position.y,
                current_pose.pose.position.z);
    
    // =========================================================
    // STEP 1: MOVE TO HOME
    // =========================================================
    
    RCLCPP_INFO(node->get_logger(), "Moving to HOME position...");
    move_group.clearPathConstraints();
    move_group.setNamedTarget("home");
    
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (move_group.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(home_plan);
        rclcpp::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(node->get_logger(), "Home position reached");
    } else {
        RCLCPP_WARN(node->get_logger(), "Failed to plan to home, continuing from current position...");
    }
    
    // =========================================================
    // CREATE APPROACH POSE FROM LEVER POSE
    // =========================================================
    
    geometry_msgs::msg::Pose approach_pose;
    approach_pose.position.x = lever_pose.position.x;
    approach_pose.position.y = lever_pose.position.y + 0.20;  // 20cm offset in Y
    approach_pose.position.z = lever_pose.position.z;
    approach_pose.orientation = lever_pose.orientation;
    
    RCLCPP_INFO(node->get_logger(), "Approach pose: x=%.3f y=%.3f z=%.3f",
                approach_pose.position.x, approach_pose.position.y, approach_pose.position.z);
    
    // =========================================================
    // STEP 2: MOVE TO APPROACH POSE
    // =========================================================
    
    RCLCPP_INFO(node->get_logger(), "Moving to APPROACH pose...");
    move_group.clearPathConstraints();
    addOrientationConstraint(move_group, approach_pose);
    move_group.setPoseTarget(approach_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    if (move_group.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(approach_plan);
        rclcpp::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(node->get_logger(), "Approach pose reached");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan to approach pose");
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    
    // =========================================================
    // STEP 3: CARTESIAN DESCENT (VERTICAL)
    // =========================================================
    
    RCLCPP_INFO(node->get_logger(), "Starting vertical descent to lever...");
    move_group.clearPathConstraints();
    
    geometry_msgs::msg::Pose descent_start_pose = move_group.getCurrentPose().pose;
    double descent_distance = descent_start_pose.position.z - lever_pose.position.z;
    
    RCLCPP_INFO(node->get_logger(), "Descent distance: %.3f meters", descent_distance);
    
    std::vector<geometry_msgs::msg::Pose> descent_waypoints;
    int num_waypoints = 50;
    
    for (int i = 1; i <= num_waypoints; i++) {
        geometry_msgs::msg::Pose waypoint = descent_start_pose;
        double t = (double)i / num_waypoints;
        double eased_t = t * t * (3.0 - 2.0 * t);
        waypoint.position.z = descent_start_pose.position.z - (descent_distance * eased_t);
        waypoint.orientation = descent_start_pose.orientation;
        descent_waypoints.push_back(waypoint);
    }
    
    moveit_msgs::msg::RobotTrajectory descent_trajectory;
    double fraction = move_group.computeCartesianPath(
        descent_waypoints, 0.0005, 0.0, descent_trajectory, true);
    
    if (fraction > 0.95) {
        move_group.execute(descent_trajectory);
        RCLCPP_INFO(node->get_logger(), "Vertical descent complete");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Descent planning failed (fraction: %.2f)", fraction);
        rclcpp::shutdown();
        spin_thread.join();
        return 1;
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // =========================================================
    // STEP 4: PULL MOTION (X DIRECTION)
    // =========================================================
    
    RCLCPP_INFO(node->get_logger(), "Executing pull motion in X direction...");
    move_group.clearPathConstraints();
    
    geometry_msgs::msg::Pose pull_start_pose = move_group.getCurrentPose().pose;
    std::vector<geometry_msgs::msg::Pose> pull_waypoints;
    int num_pull_waypoints = 20;
    double pull_distance = 0.05;  // Pull 5cm
    
    for (int i = 1; i <= num_pull_waypoints; i++) {
        geometry_msgs::msg::Pose waypoint = pull_start_pose;
        double t = (double)i / num_pull_waypoints;
        double eased_t = t * t * (3.0 - 2.0 * t);
        waypoint.position.x = pull_start_pose.position.x - (pull_distance * eased_t);
        waypoint.orientation = pull_start_pose.orientation;
        pull_waypoints.push_back(waypoint);
    }
    
    moveit_msgs::msg::RobotTrajectory pull_trajectory;
    fraction = move_group.computeCartesianPath(pull_waypoints, 0.001, 0.0, pull_trajectory, true);
    
    if (fraction > 0.9) {
        move_group.execute(pull_trajectory);
        RCLCPP_INFO(node->get_logger(), "Pull motion complete");
    } else {
        RCLCPP_WARN(node->get_logger(), "Pull motion fraction low (%.2f), skipping", fraction);
    }
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // =========================================================
    // STEP 5: RETRACT (VERTICAL)
    // =========================================================
    
    RCLCPP_INFO(node->get_logger(), "Retracting vertically...");
    move_group.clearPathConstraints();
    
    std::vector<geometry_msgs::msg::Pose> retract_waypoints;
    geometry_msgs::msg::Pose retract_start_pose = move_group.getCurrentPose().pose;
    
    for (int i = 1; i <= num_waypoints; i++) {
        geometry_msgs::msg::Pose waypoint = retract_start_pose;
        double t = (double)i / num_waypoints;
        double eased_t = t * t * (3.0 - 2.0 * t);
        waypoint.position.z = retract_start_pose.position.z + (descent_distance * eased_t);
        waypoint.orientation = retract_start_pose.orientation;
        retract_waypoints.push_back(waypoint);
    }
    
    moveit_msgs::msg::RobotTrajectory retract_trajectory;
    fraction = move_group.computeCartesianPath(retract_waypoints, 0.0005, 0.0, retract_trajectory, true);
    
    if (fraction > 0.95) {
        move_group.execute(retract_trajectory);
        RCLCPP_INFO(node->get_logger(), "Vertical retract complete");
    } else {
        RCLCPP_WARN(node->get_logger(), "Retract planning fraction low (%.2f)", fraction);
    }
    
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // =========================================================
    // STEP 6: RETURN HOME
    // =========================================================
    
    RCLCPP_INFO(node->get_logger(), "Returning home...");
    move_group.clearPathConstraints();
    move_group.setNamedTarget("home");
    
    moveit::planning_interface::MoveGroupInterface::Plan return_plan;
    if (move_group.plan(return_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(return_plan);
        RCLCPP_INFO(node->get_logger(), "Returned home successfully");
    } else {
        RCLCPP_WARN(node->get_logger(), "Failed to plan return home");
    }
    
    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), "=== TRAJECTORY COMPLETE ===");
    RCLCPP_INFO(node->get_logger(), "========================================");
    
    rclcpp::shutdown();
    spin_thread.join();
    
    return 0;
}