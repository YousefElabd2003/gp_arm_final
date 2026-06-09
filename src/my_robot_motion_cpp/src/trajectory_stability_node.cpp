#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <fstream>
#include <mutex>
#include <thread>

class TrajectoryStabilityNode : public rclcpp::Node
{
public:

    TrajectoryStabilityNode()
    : Node(
        "trajectory_stability_node",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true))
    {
        csv_.open("trajectory_stability.csv");

        csv_
            << "sample,"
            << "goal_x,"
            << "goal_y,"
            << "goal_z,"
            << "success,"
            << "planning_time,"
            << "j1,"
            << "j2,"
            << "j3,"
            << "j4,"
            << "j5,"
            << "j6\n";

        sub_ =
            create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/noisy_handle_pose",
                10,
                std::bind(
                    &TrajectoryStabilityNode::poseCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Trajectory stability node ready");
    }

    void setMoveGroup(
        std::shared_ptr<
            moveit::planning_interface::MoveGroupInterface> move_group)
    {
        move_group_ = move_group;
    }

private:

    void poseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!move_group_)
            return;

        sample_count_++;

        move_group_->setPoseTarget(msg->pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        auto start =
            std::chrono::steady_clock::now();

        bool success =
            (move_group_->plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        auto end =
            std::chrono::steady_clock::now();

        double planning_time =
            std::chrono::duration<double>(
                end - start).count();

        std::vector<double> joints(6, 0.0);

        if (success)
        {
            const auto &points =
                plan.trajectory_.joint_trajectory.points;

            if (!points.empty())
            {
                auto final =
                    points.back().positions;

                for (size_t i = 0;
                     i < std::min(final.size(), joints.size());
                     ++i)
                {
                    joints[i] = final[i];
                }
            }
        }

        csv_
            << sample_count_ << ","
            << msg->pose.position.x << ","
            << msg->pose.position.y << ","
            << msg->pose.position.z << ","
            << success << ","
            << planning_time << ","
            << joints[0] << ","
            << joints[1] << ","
            << joints[2] << ","
            << joints[3] << ","
            << joints[4] << ","
            << joints[5]
            << "\n";

        csv_.flush();

        RCLCPP_INFO(
            get_logger(),
            "Sample %ld | success=%d | time=%.3f",
            sample_count_,
            success,
            planning_time);
    }

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr sub_;

    std::shared_ptr<
        moveit::planning_interface::MoveGroupInterface> move_group_;

    std::ofstream csv_;

    std::mutex mutex_;

    long sample_count_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<TrajectoryStabilityNode>();

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);

    std::thread spin_thread(
        [&executor]()
        {
            executor.spin();
        });

    rclcpp::sleep_for(
        std::chrono::seconds(5));

    auto move_group =
        std::make_shared<
            moveit::planning_interface::MoveGroupInterface>(
                node,
                "arm_group");

    move_group->setPlanningTime(10.0);

    move_group->setMaxVelocityScalingFactor(0.2);

    move_group->setMaxAccelerationScalingFactor(0.2);

    node->setMoveGroup(move_group);

    RCLCPP_INFO(
        node->get_logger(),
        "MoveGroup initialized");

    spin_thread.join();

    rclcpp::shutdown();

    return 0;
}