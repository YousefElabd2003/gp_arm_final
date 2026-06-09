#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <fstream>
#include <memory>
#include <chrono>

class ReachabilityMapNode : public rclcpp::Node
{
public:
    ReachabilityMapNode()
        : Node("reachability_map_node")
    {
        RCLCPP_INFO(this->get_logger(),
                    "Starting reachability map node...");
    }

    void initialize()
    {
        /*
         * Wait for move_group to fully initialize
         */
        rclcpp::sleep_for(std::chrono::seconds(5));

        move_group_ =
            std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(),
                "arm_group");

        move_group_->setPlanningTime(2.0);
        move_group_->setNumPlanningAttempts(3);

        RCLCPP_INFO(this->get_logger(),
                    "MoveGroupInterface initialized");

        generateWorkspaceMap();
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
        move_group_;

    void generateWorkspaceMap()
    {
        std::ofstream csv_file("reachable_points.csv");

        csv_file << "x,y,z\n";

        double x_min = 0.20;
        double x_max = 1.00;

        double y_min = -0.60;
        double y_max = 0.60;

        double z_min = 0.10;
        double z_max = 1.20;

        double step = 0.05;

        int reachable_count = 0;
        int total_count = 0;

        geometry_msgs::msg::Pose target_pose;

        /*
         * Vertical downward orientation
         */
        target_pose.orientation.x = 1.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;

        for (double x = x_min; x <= x_max; x += step)
        {
            for (double y = y_min; y <= y_max; y += step)
            {
                for (double z = z_min; z <= z_max; z += step)
                {
                    total_count++;

                    target_pose.position.x = x;
                    target_pose.position.y = y;
                    target_pose.position.z = z;

                    move_group_->setPoseTarget(target_pose);

                    moveit::planning_interface::MoveGroupInterface::Plan
                        plan;

                    bool success =
                        (move_group_->plan(plan) ==
                         moveit::core::MoveItErrorCode::SUCCESS);

                    if (success)
                    {
                        reachable_count++;

                        csv_file
                            << x << ","
                            << y << ","
                            << z << "\n";

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Reachable: x=%.2f y=%.2f z=%.2f",
                            x,
                            y,
                            z);
                    }

                    move_group_->clearPoseTargets();
                }
            }
        }

        csv_file.close();

        RCLCPP_INFO(this->get_logger(),
                    "Finished workspace sampling");

        RCLCPP_INFO(this->get_logger(),
                    "Reachable points: %d / %d",
                    reachable_count,
                    total_count);

        RCLCPP_INFO(this->get_logger(),
                    "Saved to reachable_points.csv");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ReachabilityMapNode>();

    node->initialize();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}