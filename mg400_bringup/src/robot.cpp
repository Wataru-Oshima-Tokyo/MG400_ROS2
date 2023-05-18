/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include <mg400_bringup/robot.hpp>
#include <functional>
#include <memory>

// MG400Robot::MG400Robot(std::shared_ptr<rclcpp::Node> node, std::string name)
//     : Node("mg400_action_server"), node_(node), 
//     action_server_(rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
//             this,
//             "mg400_server",
//             std::bind(&MG400Robot::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
//             std::bind(&MG400Robot::handle_cancel, this, std::placeholders::_1),
//             std::bind(&MG400Robot::handle_cancel, this, std::placeholders::_1)
//             )),
//       goal_{}, trajectory_duration_(1.0)
// {
//     index_ = 0;
//     memset(goal_, 0, sizeof(goal_));
// }

MG400Robot::MG400Robot(std::shared_ptr<rclcpp::Node> node, std::string name)
    : Node("mg400_action_server"), node_(node), goal_{}, trajectory_duration_(1.0),
    action_server_(rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "mg400_server",
            std::bind(&MG400Robot::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MG400Robot::handle_cancel, this, std::placeholders::_1),
            std::bind(&MG400Robot::handle_accepted, this, std::placeholders::_1)
            ))
      
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}




MG400Robot::~MG400Robot()
{
    RCLCPP_INFO(node_->get_logger(), "~MG400Robot");
}

void MG400Robot::init()
{
    std::string ip = node_->declare_parameter<std::string>("robot_ip_address", "192.168.1.6");

    trajectory_duration_ = node_->declare_parameter<double>("trajectory_duration", 0.3);
    // RCLCPP_INFO(node_->get_logger(), "trajectory_duration : %0.2f", trajectory_duration_);

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();


    //service registered here
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::EnableRobot>("EnableRobot", &MG400Robot::enableRobot));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::DisableRobot>("DisableRobot", &MG400Robot::disableRobot));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::ClearError>("ClearError", &MG400Robot::clearError));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::ResetRobot>("ResetRobot", &MG400Robot::resetRobot));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::SpeedFactor>("SpeedFactor", &MG400Robot::speedFactor));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::User>("User", &MG400Robot::user));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::Tool>("Tool", &MG400Robot::tool));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::RobotMode>("RobotMode", &MG400Robot::robotMode));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::PayLoad>("Payload", &MG400Robot::payload));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::DO>("DO", &MG400Robot::DO));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::DOExecute>("DOExecute", &MG400Robot::DOExecute));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::ToolDO>("ToolDO", &MG400Robot::toolDO));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::ToolDOExecute>("ToolDOExecute", &MG400Robot::toolDOExecute));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::AO>("AO", &MG400Robot::AO));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::AOExecute>("AOExecute", &MG400Robot::AOExecute));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::AccJ>("AccJ", &MG400Robot::accJ));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::AccL>("AccL", &MG400Robot::accL));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::SpeedJ>("SpeedJ", &MG400Robot::speedJ));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::SpeedL>("SpeedL", &MG400Robot::speedL));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::Arch>("Arch", &MG400Robot::arch));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::CP>("CP", &MG400Robot::cp));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::LimZ>("LimZ", &MG400Robot::limZ));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::SetArmOrientation>("SetArmOrientation", &MG400Robot::setArmOrientation));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::PowerOn>("PowerOn", &MG400Robot::powerOn));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::RunScript>("RunScript", &MG400Robot::runScript));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::StopScript>("StopScript", &MG400Robot::stopScript));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::PauseScript>("PauseScript", &MG400Robot::pauseScript));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::ContinueScript>("ContinueScript", &MG400Robot::continueScript));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::SetSafeSkin>("SetSafeSkin", &MG400Robot::setSafeSkin));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::SetObstacleAvoid>("SetObstacleAvoid", &MG400Robot::setObstacleAvoid));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::SetCollisionLevel>("SetCollisionLevel", &MG400Robot::setCollisionLevel));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::EmergencyStop>("EmergencyStop", &MG400Robot::emergencyStop));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::MovJ>("MovJ", &MG400Robot::movJ));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::MovL>("MovL", &MG400Robot::movL));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::JointMovJ>("JointMovJ", &MG400Robot::jointMovJ));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::Jump>("Jump", &MG400Robot::jump));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::RelMovJ>("RelMovJ", &MG400Robot::relMovJ));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::RelMovL>("RelMovL", &MG400Robot::relMovL));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::Arc>("Arc", &MG400Robot::arc));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::Circle>("Circle", &MG400Robot::circle));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::ServoJ>("ServoJ", &MG400Robot::servoJ));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::ServoP>("ServoP", &MG400Robot::servoP));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::Sync>("Sync", &MG400Robot::sync));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::StartTrace>("StartTrace", &MG400Robot::startTrace));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::StartPath>("StartPath", &MG400Robot::startPath));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::StartFCTrace>("StartFCTrace", &MG400Robot::startFCTrace));
    // server_tbl_.push_back(node_->create_service<mg400_msgs::srv::MoveJog>("MoveJog", &MG400Robot::moveJog));







}

// You need to implement the new goal, cancel and accepted handle functions for ROS 2

// void MG400Robot::feedbackHandle()
// {
//     control_msgs::msg::FollowJointTrajectoryFeedback feedback;

//     double current_joints[6];
//     getJointState(current_joints);

//     for (uint32_t i = 0; i < 6; i++)
//     {
//         feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
//         feedback.actual.positions.push_back(current_joints[i]);
//         feedback.desired.positions.push_back(goal_[i]);
//     }

//     // Implement logic to publish the feedback
// }





//--------- service here -------------

// bool MG400Robot::enableRobot(
//                 const std::shared_ptr<mg400_msgs::srv::EnableRobot::Request> request,
//                 std::shared_ptr<mg400_msgs::srv::EnableRobot::Response> response){return false;}

// bool MG400Robot::disableRobot(
//                 const std::shared_ptr<mg400_msgs::srv::DisableRobot::Request> request,
//                 std::shared_ptr<mg400_msgs::srv::DisableRobot::Response> response){return false;}

// bool MG400Robot::clearError(
//                 const std::shared_ptr<mg400_msgs::srv::ClearError::Request> request,
//                 std::shared_ptr<mg400_msgs::srv::ClearError::Response> response){return false;}

// bool MG400Robot::resetRobot(
//                 const std::shared_ptr<mg400_msgs::srv::ResetRobot::Request> request,
//                 std::shared_ptr<mg400_msgs::srv::ResetRobot::Response> response){return false;}

// bool MG400Robot::speedFactor(
//                 const std::shared_ptr<mg400_msgs::srv::SpeedFactor::Request> request,
//                 std::shared_ptr<mg400_msgs::srv::SpeedFactor::Response> response){return false;}

// bool MG400Robot::user(
//         const std::shared_ptr<mg400_msgs::srv::User::Request> request,
//         std::shared_ptr<mg400_msgs::srv::User::Response> response){return false;}

// bool MG400Robot::tool(
//         const std::shared_ptr<mg400_msgs::srv::Tool::Request> request,
//         std::shared_ptr<mg400_msgs::srv::Tool::Response> response){return false;}

// bool MG400Robot::robotMode(
//             const std::shared_ptr<mg400_msgs::srv::RobotMode::Request> request,
//             std::shared_ptr<mg400_msgs::srv::RobotMode::Response> response){return false;}

// bool MG400Robot::payload(
//             const std::shared_ptr<mg400_msgs::srv::PayLoad::Request> request,
//             std::shared_ptr<mg400_msgs::srv::PayLoad::Response> response){return false;}

// bool MG400Robot::DO(
//             const std::shared_ptr<mg400_msgs::srv::DO::Request> request,
//             std::shared_ptr<mg400_msgs::srv::DO::Response> response){return false;}

// bool MG400Robot::DOExecute(
//         const std::shared_ptr<mg400_msgs::srv::DOExecute::Request> request,
//         std::shared_ptr<mg400_msgs::srv::DOExecute::Response> response){return false;}

// bool MG400Robot::toolDO(
//         const std::shared_ptr<mg400_msgs::srv::ToolDO::Request> request,
//         std::shared_ptr<mg400_msgs::srv::ToolDO::Response> response){return false;}

// bool MG400Robot::toolDOExecute(
//         const std::shared_ptr<mg400_msgs::srv::ToolDOExecute::Request> request,
//         std::shared_ptr<mg400_msgs::srv::ToolDOExecute::Response> response){return false;}
        
// bool MG400Robot::AOExecute(
//         const std::shared_ptr<mg400_msgs::srv::AOExecute::Request> request,
//         std::shared_ptr<mg400_msgs::srv::AOExecute::Response> response){return false;}

// bool MG400Robot::accJ(
//         const std::shared_ptr<mg400_msgs::srv::AccJ::Request> request,
//         std::shared_ptr<mg400_msgs::srv::AccJ::Response> response){return false;}

// bool MG400Robot::accL(
//         const std::shared_ptr<mg400_msgs::srv::AccL::Request> request,
//         std::shared_ptr<mg400_msgs::srv::AccL::Response> response){return false;}

// bool MG400Robot::speedJ(
//         const std::shared_ptr<mg400_msgs::srv::SpeedJ::Request> request,
//         std::shared_ptr<mg400_msgs::srv::SpeedJ::Response> response){return false;}

// bool MG400Robot::speedL(
//         const std::shared_ptr<mg400_msgs::srv::SpeedL::Request> request,
//         std::shared_ptr<mg400_msgs::srv::SpeedL::Response> response){return false;}

// bool MG400Robot::arch(
//         const std::shared_ptr<mg400_msgs::srv::Arch::Request> request,
//         std::shared_ptr<mg400_msgs::srv::Arch::Response> response){return false;}

// bool MG400Robot::cp(
//         const std::shared_ptr<mg400_msgs::srv::CP::Request> request,
//         std::shared_ptr<mg400_msgs::srv::CP::Response> response){return false;}

// bool MG400Robot::limZ(
//         const std::shared_ptr<mg400_msgs::srv::LimZ::Request> request,
//         std::shared_ptr<mg400_msgs::srv::LimZ::Response> response){return false;}

// bool MG400Robot::setArmOrientation(
//         const std::shared_ptr<mg400_msgs::srv::SetArmOrientation::Request> request,
//         std::shared_ptr<mg400_msgs::srv::SetArmOrientation::Response> response){return false;}

// bool MG400Robot::powerOn(
//         const std::shared_ptr<mg400_msgs::srv::PowerOn::Request> request,
//         std::shared_ptr<mg400_msgs::srv::PowerOn::Response> response){return false;}
        
// bool MG400Robot::runScript(
//         const std::shared_ptr<mg400_msgs::srv::RunScript::Request> request,
//         std::shared_ptr<mg400_msgs::srv::RunScript::Response> response){return false;}

// bool MG400Robot::stopScript(
//         const std::shared_ptr<mg400_msgs::srv::StopScript::Request> request,
//         std::shared_ptr<mg400_msgs::srv::StopScript::Response> response){return false;}

// bool MG400Robot::pauseScript(
//         const std::shared_ptr<mg400_msgs::srv::PauseScript::Request> request,
//         std::shared_ptr<mg400_msgs::srv::PauseScript::Response> response){return false;}

// bool MG400Robot::continueScript(
//         const std::shared_ptr<mg400_msgs::srv::ContinueScript::Request> request,
//         std::shared_ptr<mg400_msgs::srv::ContinueScript::Response> response){return false;}

// bool MG400Robot::setSafeSkin(
//         const std::shared_ptr<mg400_msgs::srv::SetSafeSkin::Request> request,
//         std::shared_ptr<mg400_msgs::srv::SetSafeSkin::Response> response){return false;}

// bool MG400Robot::setObstacleAvoid(
//         const std::shared_ptr<mg400_msgs::srv::SetObstacleAvoid::Request> request,
//         std::shared_ptr<mg400_msgs::srv::SetObstacleAvoid::Response> response){return false;}

// bool MG400Robot::setCollisionLevel(
//         const std::shared_ptr<mg400_msgs::srv::SetCollisionLevel::Request> request,
//         std::shared_ptr<mg400_msgs::srv::SetCollisionLevel::Response> response){return false;}

// bool MG400Robot::emergencyStop(
//         const std::shared_ptr<mg400_msgs::srv::EmergencyStop::Request> request,
//         std::shared_ptr<mg400_msgs::srv::EmergencyStop::Response> response){return false;}

// bool MG400Robot::movJ(
//         const std::shared_ptr<mg400_msgs::srv::MovJ::Request> request,
//         std::shared_ptr<mg400_msgs::srv::MovJ::Response> response){return false;}

// bool MG400Robot::movL(
//         const std::shared_ptr<mg400_msgs::srv::MovL::Request> request,
//         std::shared_ptr<mg400_msgs::srv::MovL::Response> response){return false;}
        
// bool MG400Robot::jointMovJ(
//         const std::shared_ptr<mg400_msgs::srv::JointMovJ::Request> request,
//         std::shared_ptr<mg400_msgs::srv::JointMovJ::Response> response){return false;}

// bool MG400Robot::jump(
//         const std::shared_ptr<mg400_msgs::srv::Jump::Request> request,
//         std::shared_ptr<mg400_msgs::srv::Jump::Response> response){return false;}  

// bool MG400Robot::relMovJ(
//         const std::shared_ptr<mg400_msgs::srv::RelMovJ::Request> request,
//         std::shared_ptr<mg400_msgs::srv::RelMovJ::Response> response){return false;}

// bool MG400Robot::relMovL(
//         const std::shared_ptr<mg400_msgs::srv::RelMovL::Request> request,
//         std::shared_ptr<mg400_msgs::srv::RelMovL::Response> response){return false;}

// bool MG400Robot::arc(
//         const std::shared_ptr<mg400_msgs::srv::Arc::Request> request,
//         std::shared_ptr<mg400_msgs::srv::Arc::Response> response){return false;}  

// bool MG400Robot::circle(
//         const std::shared_ptr<mg400_msgs::srv::Circle::Request> request,
//         std::shared_ptr<mg400_msgs::srv::Circle::Response> response){return false;}

// bool MG400Robot::servoJ(
//         const std::shared_ptr<mg400_msgs::srv::ServoJ::Request> request,
//         std::shared_ptr<mg400_msgs::srv::ServoJ::Response> response){return false;}

// bool MG400Robot::servoP(
//         const std::shared_ptr<mg400_msgs::srv::ServoP::Request> request,
//         std::shared_ptr<mg400_msgs::srv::ServoP::Response> response){return false;}  

// bool MG400Robot::sync(
//         const std::shared_ptr<mg400_msgs::srv::Sync::Request> request,
//         std::shared_ptr<mg400_msgs::srv::Sync::Response> response){return false;}

// bool MG400Robot::startTrace(
//         const std::shared_ptr<mg400_msgs::srv::StartTrace::Request> request,
//         std::shared_ptr<mg400_msgs::srv::StartTrace::Response> response){return false;}

// bool MG400Robot::startPath(
//         const std::shared_ptr<mg400_msgs::srv::StartPath::Request> request,
//         std::shared_ptr<mg400_msgs::srv::StartPath::Response> response){return false;}  

// bool MG400Robot::startFCTrace(
//         const std::shared_ptr<mg400_msgs::srv::StartFCTrace::Request> request,
//         std::shared_ptr<mg400_msgs::srv::StartFCTrace::Response> response){return false;}

// bool MG400Robot::moveJog(
//         const std::shared_ptr<mg400_msgs::srv::MoveJog::Request> request,
//         std::shared_ptr<mg400_msgs::srv::MoveJog::Response> response){return false;}
