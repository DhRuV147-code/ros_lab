#include <memory>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "robot_msgs/action/robot_task.hpp"

using RobotTask = robot_msgs::action::RobotTask;
using GoalHandleRobotTask = rclcpp_action::ServerGoalHandle<RobotTask>;

class TaskActionServer : public rclcpp::Node {
public:
  TaskActionServer()
  : Node("task_action_server")
  {
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<RobotTask>(
      this,
      "robot_task",
      std::bind(&TaskActionServer::handle_goal, this, _1, _2),
      std::bind(&TaskActionServer::handle_cancel, this, _1),
      std::bind(&TaskActionServer::handle_accepted, this, _1)
    );
  }

private:
  rclcpp_action::Server<RobotTask>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RobotTask::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: task_number=%d", goal->task_number);
    if (goal->task_number < 1 || goal->task_number > 2) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRobotTask> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Canceling goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRobotTask> goal_handle)
  {
    std::thread(
      std::bind(&TaskActionServer::execute, this, goal_handle)
    ).detach();
  }

  bool move_to_joints(moveit::planning_interface::MoveGroupInterface &move_group, const std::vector<double> &joint_positions) {
    move_group.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group.execute(plan);
    }
    return success;
  }

  bool control_gripper(moveit::planning_interface::MoveGroupInterface &gripper_group, const std::vector<double> &positions) {
    gripper_group.setJointValueTarget(positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (gripper_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      gripper_group.execute(plan);
    }
    return success;
  }

  void execute(const std::shared_ptr<GoalHandleRobotTask> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing task...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<RobotTask::Feedback>();
    auto result = std::make_shared<RobotTask::Result>();
  
    // Create a separate MoveIt node for planning interfaces
    auto moveit_node = rclcpp::Node::make_shared("moveit_node");
    moveit::planning_interface::MoveGroupInterface arm_group(moveit_node, "arm");
    moveit::planning_interface::MoveGroupInterface gripper_group(moveit_node, "gripper");
  
    bool ok = false;
  
    if (goal->task_number == 1) {
      feedback->progress = "Moving to Task 1 position...";
      goal_handle->publish_feedback(feedback);
      ok = move_to_joints(arm_group, {1.57, 0.8, 1.57, 0, -0.8}) &&
           control_gripper(gripper_group, {0.0, 0.0});
    } else if (goal->task_number == 2) {
      feedback->progress = "Moving to Task 2 position...";
      goal_handle->publish_feedback(feedback);
      ok = move_to_joints(arm_group, {0, 0.8, 1.57, 0, -0.8}) &&
           control_gripper(gripper_group, {0.7, -0.7});
    }
  
    result->success = ok;
    if (ok) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Task %d succeeded.", goal->task_number);
    } else {
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Task %d failed.", goal->task_number);
    }
  }  
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<TaskActionServer>();
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  return 0;
}
