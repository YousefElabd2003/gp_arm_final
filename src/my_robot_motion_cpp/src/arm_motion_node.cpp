#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>

#include <thread>
#include <vector>
#include <memory>
#include <cmath>

// Function to add orientation constraints (only used for initial approach)
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
    RCLCPP_INFO(rclcpp::get_logger("arm_motion_node"), "Orientation constraints applied to link: %s", 
                ocm.link_name.c_str());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "arm_motion_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread([&executor]() {
        executor.spin();
    }).detach();

    using moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface move_group(node, "arm_group");

    RCLCPP_INFO(node->get_logger(), "MoveGroupInterface initialized");

    // =========================================================
    // SMOOTH MOTION PARAMETERS - SLOWER VELOCITY
    // =========================================================
    
    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(0.05);      // 5% of max speed - slow and smooth
    move_group.setMaxAccelerationScalingFactor(0.05);  // 5% of max acceleration
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.01);
    
    // Get the end effector link name
    std::string ee_link = move_group.getEndEffectorLink();
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", ee_link.c_str());

    // =========================================================
    // WAIT FOR VALID JOINT STATES
    // =========================================================

    RCLCPP_INFO(node->get_logger(), "Waiting for valid joint states...");

    // Wait for robot state to be ready
    for (int i = 0; i < 10; ++i) {
        try {
            auto current_pose = move_group.getCurrentPose();
            break;
        } catch (...) {
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    rclcpp::sleep_for(std::chrono::seconds(2));

    auto current_pose = move_group.getCurrentPose();

    RCLCPP_INFO(
        node->get_logger(),
        "Current pose: x=%.3f y=%.3f z=%.3f",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z
    );
    
    RCLCPP_INFO(
        node->get_logger(),
        "Current orientation: x=%.3f y=%.3f z=%.3f w=%.3f",
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w
    );

    // =========================================================
    // STEP 1: MOVE TO HOME FIRST
    // =========================================================
    
    RCLCPP_INFO(node->get_logger(), "Moving to HOME position first...");
    
    // Clear any existing constraints
    move_group.clearPathConstraints();
    
    move_group.setNamedTarget("home");
    
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    
    bool home_success = (move_group.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (!home_success)
    {
        RCLCPP_WARN(node->get_logger(), "Failed to plan to home, continuing from current position...");
    }
    else
    {
        auto home_result = move_group.execute(home_plan);
        if (home_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Successfully moved to home");
            rclcpp::sleep_for(std::chrono::seconds(2));
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Failed to execute home motion, continuing from current position...");
        }
    }

    // =========================================================
    // USE THE TUNED POSE FROM RViz
    // =========================================================
    
    geometry_msgs::msg::Pose approach_pose;
    
    // Position from RViz tuning
    approach_pose.position.x = 0.850;
    approach_pose.position.y = 0.014;
    approach_pose.position.z = 0.892;
    
    // Orientation from RViz tuning (kinematically verified)
    approach_pose.orientation.x = -0.011;
    approach_pose.orientation.y = 0.969;
    approach_pose.orientation.z = 0.008;
    approach_pose.orientation.w = -0.248;

    RCLCPP_INFO(
        node->get_logger(),
        "Using TUNED pose from RViz: x=%.3f y=%.3f z=%.3f",
        approach_pose.position.x,
        approach_pose.position.y,
        approach_pose.position.z
    );
    
    RCLCPP_INFO(
        node->get_logger(),
        "Tuned orientation: x=%.3f y=%.3f z=%.3f w=%.3f",
        approach_pose.orientation.x,
        approach_pose.orientation.y,
        approach_pose.orientation.z,
        approach_pose.orientation.w
    );

    // =========================================================
    // STEP 2: APPROACH POSE WITH TUNED ORIENTATION
    // =========================================================

    RCLCPP_INFO(
        node->get_logger(),
        "Moving to APPROACH pose: x=%.3f y=%.3f z=%.3f",
        approach_pose.position.x,
        approach_pose.position.y,
        approach_pose.position.z
    );
    
    // Clear constraints before planning to approach
    move_group.clearPathConstraints();
    
    // Add orientation constraint for the approach
    addOrientationConstraint(move_group, approach_pose);
    
    move_group.setPoseTarget(approach_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    
    bool approach_success = (move_group.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (!approach_success)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed approach planning");
        
        // Fallback: Try without orientation constraint
        RCLCPP_WARN(node->get_logger(), "Retrying without orientation constraint...");
        move_group.clearPathConstraints();
        move_group.setPoseTarget(approach_pose);
        
        approach_success = (move_group.plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!approach_success)
        {
            RCLCPP_ERROR(node->get_logger(), "Approach planning failed again, aborting");
            rclcpp::shutdown();
            return 1;
        }
    }
    
    RCLCPP_INFO(node->get_logger(), "Executing approach motion...");
    
    auto approach_result = move_group.execute(approach_plan);
    
    if (approach_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed executing approach motion");
        rclcpp::shutdown();
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Approach motion completed successfully");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // =========================================================
    // STEP 3: SMOOTH CARTESIAN DESCENT - VERTICAL (SLOW VELOCITY)
    // =========================================================

    RCLCPP_INFO(node->get_logger(), "Starting SLOW VERTICAL descent (Z direction)...");
    
    // CRITICAL: Clear ALL constraints to prevent wrist rotation during descent
    move_group.clearPathConstraints();
    
    // Get the current pose (which already has the correct approach orientation)
    geometry_msgs::msg::Pose descent_start_pose = move_group.getCurrentPose().pose;
    
    RCLCPP_INFO(node->get_logger(), "Descent start pose: x=%.3f y=%.3f z=%.3f", 
                descent_start_pose.position.x, descent_start_pose.position.y, descent_start_pose.position.z);
    
    // Define vertical movement (Z direction only)
    // Adjust end_z based on your door handle height
    double start_z = descent_start_pose.position.z;
    double end_z = 0.750;  // Move down to handle (adjust this value!)
    double descent_distance = start_z - end_z;
    
    RCLCPP_INFO(node->get_logger(), "Vertical descent distance: %.3f meters (from z=%.3f to z=%.3f)", 
                descent_distance, start_z, end_z);
    
    // Generate dense waypoints for smooth motion
    std::vector<geometry_msgs::msg::Pose> descent_waypoints;
    int num_waypoints = 50;  // Dense waypoints for ultra-smooth motion
    
    for (int i = 1; i <= num_waypoints; i++) {
        geometry_msgs::msg::Pose waypoint = descent_start_pose;
        double t = (double)i / num_waypoints;
        // Cubic easing for natural motion (smooth start and stop)
        double eased_t = t * t * (3.0 - 2.0 * t);
        // Move in Z direction only
        waypoint.position.z = start_z - (descent_distance * eased_t);
        
        // CRITICAL: Keep the SAME orientation throughout descent
        waypoint.orientation = descent_start_pose.orientation;
        
        descent_waypoints.push_back(waypoint);
    }
    
    moveit_msgs::msg::RobotTrajectory descent_trajectory;
    
    const double eef_step = 0.0005;  // 0.5mm steps for ultra-smooth motion
    const double jump_threshold = 0.0;  // Prevent joint jumps
    
    double fraction = move_group.computeCartesianPath(
        descent_waypoints,
        eef_step,
        jump_threshold,
        descent_trajectory,
        true  // Enable collision checking for safety
    );
    
    RCLCPP_INFO(
        node->get_logger(),
        "Cartesian descent fraction: %.2f%% with %d waypoints",
        fraction * 100,
        (int)descent_waypoints.size()
    );
    
    if (fraction < 0.95)
    {
        RCLCPP_ERROR(node->get_logger(), "Descent path planning failed (fraction too low)");
        rclcpp::shutdown();
        return 1;
    }
    
    // Reduce velocity and acceleration for slower, smoother execution
    if (!descent_trajectory.joint_trajectory.points.empty()) {
        double max_velocity = 0.03;     // Slow velocity (rad/s)
        double max_acceleration = 0.03; // Slow acceleration (rad/s^2)
        
        for (auto& point : descent_trajectory.joint_trajectory.points) {
            for (auto& vel : point.velocities) {
                if (std::abs(vel) > max_velocity) {
                    vel = (vel > 0) ? max_velocity : -max_velocity;
                }
            }
            for (auto& acc : point.accelerations) {
                if (std::abs(acc) > max_acceleration) {
                    acc = (acc > 0) ? max_acceleration : -max_acceleration;
                }
            }
        }
        
        RCLCPP_INFO(node->get_logger(), "Trajectory velocity limited to %.2f rad/s for slow smooth motion", max_velocity);
    }
    
    auto descent_result = move_group.execute(descent_trajectory);
    
    if (descent_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Descent execution failed");
        rclcpp::shutdown();
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Slow vertical descent complete");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // =========================================================
    // SIMULATED LEVER CONTACT / PULL - NOW IN X DIRECTION
    // =========================================================
    
    RCLCPP_INFO(node->get_logger(), "Lever contacted - executing smooth pull motion in X direction...");
    
    // Clear constraints before pull
    move_group.clearPathConstraints();
    
    // Get current pose for pull
    geometry_msgs::msg::Pose pull_start_pose = move_group.getCurrentPose().pose;
    
    // Create smooth pull waypoints in X direction
    std::vector<geometry_msgs::msg::Pose> pull_waypoints;
    int num_pull_waypoints = 20;  // More waypoints for smoother pull
    double pull_distance = 0.05;  // Pull 5cm in X direction (adjust as needed)
    
    for (int i = 1; i <= num_pull_waypoints; i++) {
        geometry_msgs::msg::Pose waypoint = pull_start_pose;
        double t = (double)i / num_pull_waypoints;
        double eased_t = t * t * (3.0 - 2.0 * t);
        // Pull in X direction (forward/backward relative to robot)
        waypoint.position.x = pull_start_pose.position.x - (pull_distance * eased_t);
        waypoint.orientation = pull_start_pose.orientation;
        pull_waypoints.push_back(waypoint);
    }
    
    moveit_msgs::msg::RobotTrajectory pull_trajectory;
    fraction = move_group.computeCartesianPath(
        pull_waypoints,
        0.001,
        0.0,
        pull_trajectory,
        true
    );
    
    if (fraction > 0.9)
    {
        // Slow down pull motion
        if (!pull_trajectory.joint_trajectory.points.empty()) {
            for (auto& point : pull_trajectory.joint_trajectory.points) {
                for (auto& vel : point.velocities) {
                    if (std::abs(vel) > 0.03) vel = (vel > 0) ? 0.03 : -0.03;
                }
            }
        }
        
        RCLCPP_INFO(node->get_logger(), "Executing slow pull motion in X direction...");
        auto pull_result = move_group.execute(pull_trajectory);
        if (pull_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Pull motion complete");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Pull motion fraction low (%.2f), skipping", fraction);
    }
    
    rclcpp::sleep_for(std::chrono::seconds(1));

    // =========================================================
    // STEP 4: SMOOTH CARTESIAN RETRACT - VERTICAL (SLOW VELOCITY)
    // =========================================================

    RCLCPP_INFO(node->get_logger(), "Starting SLOW VERTICAL retract (Z direction)...");
    
    // Clear constraints before retract
    move_group.clearPathConstraints();
    
    // Get current pose for retract
    geometry_msgs::msg::Pose retract_start_pose = move_group.getCurrentPose().pose;
    
    RCLCPP_INFO(node->get_logger(), "Retract start pose: x=%.3f y=%.3f z=%.3f", 
                retract_start_pose.position.x, retract_start_pose.position.y, retract_start_pose.position.z);
    
    // Generate dense waypoints for smooth retract
    std::vector<geometry_msgs::msg::Pose> retract_waypoints;
    
    for (int i = 1; i <= num_waypoints; i++) {
        geometry_msgs::msg::Pose waypoint = retract_start_pose;
        double t = (double)i / num_waypoints;
        double eased_t = t * t * (3.0 - 2.0 * t);
        // Move back up to original Z height
        waypoint.position.z = end_z + (descent_distance * eased_t);
        waypoint.orientation = retract_start_pose.orientation;
        retract_waypoints.push_back(waypoint);
    }
    
    moveit_msgs::msg::RobotTrajectory retract_trajectory;
    
    fraction = move_group.computeCartesianPath(
        retract_waypoints,
        eef_step,
        jump_threshold,
        retract_trajectory,
        true
    );
    
    RCLCPP_INFO(
        node->get_logger(),
        "Cartesian retract fraction: %.2f%% with %d waypoints",
        fraction * 100,
        (int)retract_waypoints.size()
    );
    
    if (fraction < 0.95)
    {
        RCLCPP_ERROR(node->get_logger(), "Retract path planning failed (fraction too low)");
        rclcpp::shutdown();
        return 1;
    }
    
    // Limit velocity and acceleration for slow smooth retract
    if (!retract_trajectory.joint_trajectory.points.empty()) {
        double max_velocity = 0.03;  // Slow velocity
        double max_acceleration = 0.03;  // Slow acceleration
        
        for (auto& point : retract_trajectory.joint_trajectory.points) {
            for (auto& vel : point.velocities) {
                if (std::abs(vel) > max_velocity) {
                    vel = (vel > 0) ? max_velocity : -max_velocity;
                }
            }
            for (auto& acc : point.accelerations) {
                if (std::abs(acc) > max_acceleration) {
                    acc = (acc > 0) ? max_acceleration : -max_acceleration;
                }
            }
        }
    }
    
    auto retract_result = move_group.execute(retract_trajectory);
    
    if (retract_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Retract execution failed");
        rclcpp::shutdown();
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Slow vertical retract complete");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // =========================================================
    // STEP 5: RETURN HOME
    // =========================================================

    RCLCPP_INFO(node->get_logger(), "Returning home...");
    
    move_group.clearPathConstraints();
    move_group.setNamedTarget("home");
    
    moveit::planning_interface::MoveGroupInterface::Plan return_home_plan;
    
    bool return_success = (move_group.plan(return_home_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (!return_success)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed planning return home");
        rclcpp::shutdown();
        return 1;
    }
    
    auto home_result = move_group.execute(return_home_plan);
    
    if (home_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed executing return home");
        rclcpp::shutdown();
        return 1;
    }
    
    RCLCPP_INFO(node->get_logger(), "Returned home successfully");
    RCLCPP_INFO(node->get_logger(), "=========================================");
    RCLCPP_INFO(node->get_logger(), "=== TRAJECTORY COMPLETE - PULL IN X DIRECTION ===");
    RCLCPP_INFO(node->get_logger(), "=========================================");

    rclcpp::shutdown();

    return 0;
}