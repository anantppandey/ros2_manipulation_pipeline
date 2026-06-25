#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.h>

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<rclcpp::Node>("mtc_node");

    moveit::task_constructor::Task task;

    RCLCPP_INFO(node->get_logger(),"MTC Loaded Successfully!");

    rclcpp::shutdown();
}