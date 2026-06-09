#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>

#include <fstream>
#include <thread>
#include <chrono>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        rclcpp::Node::make_shared(
            "workspace_sampler",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(true));

    auto logger =
        rclcpp::get_logger("workspace_sampler");

    // =====================================================
    // EXECUTOR REQUIRED FOR MOVEIT
    // =====================================================

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);

    std::thread(
        [&executor]()
        {
            executor.spin();
        }).detach();

    // =====================================================
    // MOVE GROUP
    // =====================================================

    using moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface move_group(node, "arm_group");

    move_group.setPlanningTime(2.0);

    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    move_group.startStateMonitor();

    // =====================================================
    // WAIT FOR STATE MONITOR
    // =====================================================

    RCLCPP_INFO(logger, "Waiting for robot state...");

    rclcpp::sleep_for(std::chrono::seconds(5));

    auto current_state =
        move_group.getCurrentState(10.0);

    if (!current_state)
    {
        RCLCPP_ERROR(
            logger,
            "Could not obtain robot state.");

        rclcpp::shutdown();
        return 1;
    }

    const moveit::core::JointModelGroup* joint_model_group =
        current_state->getJointModelGroup("arm_group");

    if (!joint_model_group)
    {
        RCLCPP_ERROR(
            logger,
            "Could not get arm_group.");

        rclcpp::shutdown();
        return 1;
    }

    // =====================================================
    // FIXED ORIENTATION
    // =====================================================

    geometry_msgs::msg::Quaternion fixed_orientation;

    fixed_orientation.x = -0.442;
    fixed_orientation.y = -0.467;
    fixed_orientation.z = 0.552;
    fixed_orientation.w = 0.531;

    // =====================================================
    // OUTPUT FILE
    // =====================================================

    std::ofstream csv_file(
        "reachable_points.csv");

    csv_file
        << "x,y,z,qx,qy,qz,qw\n";
// =====================================================
// SAMPLING REGION
// Centered around the proven successful pose
// x=0.78 y=0.04 z=0.78
// =====================================================

double x_min = 0.68;
double x_max = 0.88;

double y_min = -0.06;
double y_max = 0.14;

double z_min = 0.68;
double z_max = 0.88;

double step = 0.01;

    int valid_count = 0;
    int total_count = 0;

    RCLCPP_INFO(
        logger,
        "Starting planning-valid sampling...");

    // =====================================================
    // MAIN LOOP
    // =====================================================

    for (double x = x_min; x <= x_max; x += step)
    {
        for (double y = y_min; y <= y_max; y += step)
        {
            for (double z = z_min; z <= z_max; z += step)
            {
                total_count++;

                geometry_msgs::msg::Pose target_pose;

                target_pose.position.x = x;
                target_pose.position.y = y;
                target_pose.position.z = z;

                target_pose.orientation =
                    fixed_orientation;

                move_group.setStartStateToCurrentState();

                move_group.setPoseTarget(
                    target_pose);

                moveit::planning_interface::
                    MoveGroupInterface::Plan plan;

                bool success =
                    (
                        move_group.plan(plan)
                        ==
                        moveit::core::
                        MoveItErrorCode::SUCCESS
                    );

                if (!success)
                {
                    continue;
                }

                csv_file
                    << x << ","
                    << y << ","
                    << z << ","
                    << fixed_orientation.x << ","
                    << fixed_orientation.y << ","
                    << fixed_orientation.z << ","
                    << fixed_orientation.w
                    << "\n";

                valid_count++;

                RCLCPP_INFO(
                    logger,
                    "VALID %d : x=%.2f y=%.2f z=%.2f",
                    valid_count,
                    x,
                    y,
                    z);
            }
        }
    }

    csv_file.close();

    RCLCPP_INFO(
        logger,
        "Finished. Valid points: %d / %d",
        valid_count,
        total_count);

    rclcpp::shutdown();

    return 0;
}