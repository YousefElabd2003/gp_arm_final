#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <random>

struct WorkspacePoint
{
    double x;
    double y;
    double z;

    double qx;
    double qy;
    double qz;
    double qw;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        rclcpp::Node::make_shared(
            "noise_test_node",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(true));

    auto logger = node->get_logger();

    using moveit::planning_interface::MoveGroupInterface;

    MoveGroupInterface move_group(node, "arm_group");

    move_group.setPlanningTime(5.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    RCLCPP_INFO(logger, "Loading reachable_points.csv");

    std::ifstream file("reachable_points.csv");

    if (!file.is_open())
    {
        RCLCPP_ERROR(logger, "Could not open reachable_points.csv");
        return 1;
    }

    std::vector<WorkspacePoint> points;

    std::string line;

    std::getline(file, line);

    while (std::getline(file, line))
    {
        std::stringstream ss(line);

        std::string token;

        WorkspacePoint p;

        std::getline(ss, token, ',');
        p.x = std::stod(token);

        std::getline(ss, token, ',');
        p.y = std::stod(token);

        std::getline(ss, token, ',');
        p.z = std::stod(token);

        std::getline(ss, token, ',');
        p.qx = std::stod(token);

        std::getline(ss, token, ',');
        p.qy = std::stod(token);

        std::getline(ss, token, ',');
        p.qz = std::stod(token);

        std::getline(ss, token, ',');
        p.qw = std::stod(token);

        points.push_back(p);
    }

    file.close();

    RCLCPP_INFO(
        logger,
        "Loaded %ld workspace points",
        points.size());

    std::ofstream results("noise_results.csv");

    results << "trial,success\n";

    std::random_device rd;

    std::mt19937 gen(rd());

    std::uniform_int_distribution<>
        point_dist(0, points.size() - 1);

    std::normal_distribution<>
        noise(0.0, 0.005);

    const int NUM_TRIALS = 100;

    int success_count = 0;

    for (int trial = 0; trial < NUM_TRIALS; trial++)
    {
        const auto& p = points[point_dist(gen)];

        geometry_msgs::msg::Pose target_pose;

        target_pose.position.x = p.x + noise(gen);
        target_pose.position.y = p.y + noise(gen);
        target_pose.position.z = p.z + noise(gen);

        target_pose.orientation.x = p.qx;
        target_pose.orientation.y = p.qy;
        target_pose.orientation.z = p.qz;
        target_pose.orientation.w = p.qw;

        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success =
            (move_group.plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            success_count++;
        }

        results
            << trial
            << ","
            << success
            << "\n";

        RCLCPP_INFO(
            logger,
            "Trial %d : %s",
            trial,
            success ? "SUCCESS" : "FAIL");
    }

    results.close();

    double rate =
        100.0 *
        static_cast<double>(success_count) /
        static_cast<double>(NUM_TRIALS);

    RCLCPP_INFO(
        logger,
        "Success rate = %.2f %% (%d/%d)",
        rate,
        success_count,
        NUM_TRIALS);

    rclcpp::shutdown();

    return 0;
}