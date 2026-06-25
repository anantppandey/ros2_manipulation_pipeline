#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.hpp>
#include <moveit/task_constructor/stages/current_state.hpp>
#include <moveit/task_constructor/stages/move_to.hpp>

#include <moveit/task_constructor/solvers/pipeline_planner.hpp>

using namespace moveit::task_constructor;

class PickPlaceNode : public rclcpp::Node
{
public:

    PickPlaceNode()
    : Node("mtc_pick_place")
    {
        createTask();
    }

private:

    Task task_;

    void createTask()
    {
        task_.stages()->setName("Pick Task");

        auto pipeline =
            std::make_shared<solvers::PipelinePlanner>(shared_from_this());

        auto current =
            std::make_unique<stages::CurrentState>("current");

        task_.add(std::move(current));

        auto move =
            std::make_unique<stages::MoveTo>(
                "move home",
                pipeline);

        move->setGroup("YOUR_ARM_GROUP");

        move->setGoal("home");

        task_.add(std::move(move));
    }

};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<PickPlaceNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
}