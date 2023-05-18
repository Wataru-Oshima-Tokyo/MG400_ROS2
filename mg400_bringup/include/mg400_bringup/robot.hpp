/**
 ***********************************************************************************************************************
 *
 * @author Wataru Oshima
 * @date   2023/05/16
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mg400_bringup/commander.hpp> // You will have to convert this to ROS2 as well
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <mg400_msgs/srv/enable_robot.hpp>
#include <mg400_msgs/srv/disable_robot.hpp>
#include <mg400_msgs/srv/clear_error.hpp>
#include <mg400_msgs/srv/reset_robot.hpp>
#include <mg400_msgs/srv/speed_factor.hpp>
#include <mg400_msgs/srv/user.hpp>
#include <mg400_msgs/srv/tool.hpp>
#include <mg400_msgs/srv/robot_mode.hpp>
#include <mg400_msgs/srv/pay_load.hpp>
#include <mg400_msgs/srv/do.hpp>
#include <mg400_msgs/srv/do_execute.hpp>
#include <mg400_msgs/srv/tool_do.hpp>
#include <mg400_msgs/srv/tool_do_execute.hpp>
#include <mg400_msgs/srv/ao.hpp>
#include <mg400_msgs/srv/ao_execute.hpp>
#include <mg400_msgs/srv/acc_j.hpp>
#include <mg400_msgs/srv/acc_l.hpp>
#include <mg400_msgs/srv/speed_j.hpp>
#include <mg400_msgs/srv/speed_l.hpp>
#include <mg400_msgs/srv/arch.hpp>
#include <mg400_msgs/srv/cp.hpp>
#include <mg400_msgs/srv/lim_z.hpp>
#include <mg400_msgs/srv/set_arm_orientation.hpp>
#include <mg400_msgs/srv/power_on.hpp>
#include <mg400_msgs/srv/run_script.hpp>
#include <mg400_msgs/srv/stop_script.hpp>
#include <mg400_msgs/srv/pause_script.hpp>
#include <mg400_msgs/srv/continue_script.hpp>
//#include <mg400_msgs/srv/get_hold_regs.hpp>
//#include <mg400_msgs/srv/set_hold_regs.hpp>
#include <mg400_msgs/srv/set_safe_skin.hpp>
#include <mg400_msgs/srv/set_obstacle_avoid.hpp>

#include <mg400_msgs/srv/set_collision_level.hpp>
#include <mg400_msgs/srv/emergency_stop.hpp>

#include <mg400_msgs/srv/mov_j.hpp>
#include <mg400_msgs/srv/mov_l.hpp>
#include <mg400_msgs/srv/jump.hpp>
#include <mg400_msgs/srv/arc.hpp>
#include <mg400_msgs/srv/sync.hpp>
#include <mg400_msgs/srv/circle.hpp>
#include <mg400_msgs/srv/servo_j.hpp>
#include <mg400_msgs/srv/start_trace.hpp>
#include <mg400_msgs/srv/start_path.hpp>
#include <mg400_msgs/srv/start_fc_trace.hpp>
#include <mg400_msgs/srv/move_jog.hpp>
#include <mg400_msgs/srv/servo_p.hpp>
#include <mg400_msgs/srv/rel_mov_j.hpp>
#include <mg400_msgs/srv/rel_mov_l.hpp>
#include <mg400_msgs/srv/joint_mov_j.hpp>
#include <mg400_msgs/msg/robot_status.hpp>

#include <sensor_msgs/msg/joint_state.hpp>


using namespace control_msgs;

/**
 * MG400Robot
 */
// class MG400Robot : public rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::Base
class MG400Robot : public rclcpp::Node
{
private:
    double goal_[6];
    uint32_t index_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr movj_timer_;
    double trajectory_duration_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;
    std::shared_ptr<CR5Commander> commander_;
    std::vector<rclcpp::ServiceBase::SharedPtr> server_tbl_;


public:
    MG400Robot(std::shared_ptr<rclcpp::Node> nh, std::string name);
    ~MG400Robot() override;
    /**
     * init
     */
    void init();
    void getJointState(double* point);
    void getToolVectorActual(double* val);
    bool isEnable() const;
    int robotStatus() const;
    bool isConnected() const;

protected:

    bool enableRobot(
                 const std::shared_ptr<mg400_msgs::srv::EnableRobot::Request> request,
                 std::shared_ptr<mg400_msgs::srv::EnableRobot::Response> response);

    bool disableRobot(
                    const std::shared_ptr<mg400_msgs::srv::DisableRobot::Request> request,
                    std::shared_ptr<mg400_msgs::srv::DisableRobot::Response> response);

    bool clearError(
                    const std::shared_ptr<mg400_msgs::srv::ClearError::Request> request,
                    std::shared_ptr<mg400_msgs::srv::ClearError::Response> response);

    bool resetRobot(
                    const std::shared_ptr<mg400_msgs::srv::ResetRobot::Request> request,
                    std::shared_ptr<mg400_msgs::srv::ResetRobot::Response> response);

    bool speedFactor(
                    const std::shared_ptr<mg400_msgs::srv::SpeedFactor::Request> request,
                    std::shared_ptr<mg400_msgs::srv::SpeedFactor::Response> response);

    bool user(
            const std::shared_ptr<mg400_msgs::srv::User::Request> request,
            std::shared_ptr<mg400_msgs::srv::User::Response> response);

    bool tool(
            const std::shared_ptr<mg400_msgs::srv::Tool::Request> request,
            std::shared_ptr<mg400_msgs::srv::Tool::Response> response);

    bool robotMode(
                const std::shared_ptr<mg400_msgs::srv::RobotMode::Request> request,
                std::shared_ptr<mg400_msgs::srv::RobotMode::Response> response);

    bool payload(
                const std::shared_ptr<mg400_msgs::srv::PayLoad::Request> request,
                std::shared_ptr<mg400_msgs::srv::PayLoad::Response> response);

    bool DO(
                const std::shared_ptr<mg400_msgs::srv::DO::Request> request,
                std::shared_ptr<mg400_msgs::srv::DO::Response> response);

    bool DOExecute(
            const std::shared_ptr<mg400_msgs::srv::DOExecute::Request> request,
            std::shared_ptr<mg400_msgs::srv::DOExecute::Response> response);

    bool toolDO(
            const std::shared_ptr<mg400_msgs::srv::ToolDO::Request> request,
            std::shared_ptr<mg400_msgs::srv::ToolDO::Response> response);

    bool toolDOExecute(
            const std::shared_ptr<mg400_msgs::srv::ToolDOExecute::Request> request,
            std::shared_ptr<mg400_msgs::srv::ToolDOExecute::Response> response);
            
    bool AOExecute(
            const std::shared_ptr<mg400_msgs::srv::AOExecute::Request> request,
            std::shared_ptr<mg400_msgs::srv::AOExecute::Response> response);

    bool accJ(
            const std::shared_ptr<mg400_msgs::srv::AccJ::Request> request,
            std::shared_ptr<mg400_msgs::srv::AccJ::Response> response);

    bool accL(
            const std::shared_ptr<mg400_msgs::srv::AccL::Request> request,
            std::shared_ptr<mg400_msgs::srv::AccL::Response> response);

    bool speedJ(
            const std::shared_ptr<mg400_msgs::srv::SpeedJ::Request> request,
            std::shared_ptr<mg400_msgs::srv::SpeedJ::Response> response);

    bool speedL(
            const std::shared_ptr<mg400_msgs::srv::SpeedL::Request> request,
            std::shared_ptr<mg400_msgs::srv::SpeedL::Response> response);

    bool arch(
            const std::shared_ptr<mg400_msgs::srv::Arch::Request> request,
            std::shared_ptr<mg400_msgs::srv::Arch::Response> response);

    bool cp(
            const std::shared_ptr<mg400_msgs::srv::CP::Request> request,
            std::shared_ptr<mg400_msgs::srv::CP::Response> response);

    bool limZ(
            const std::shared_ptr<mg400_msgs::srv::LimZ::Request> request,
            std::shared_ptr<mg400_msgs::srv::LimZ::Response> response);

    bool setArmOrientation(
            const std::shared_ptr<mg400_msgs::srv::SetArmOrientation::Request> request,
            std::shared_ptr<mg400_msgs::srv::SetArmOrientation::Response> response);

    bool powerOn(
            const std::shared_ptr<mg400_msgs::srv::PowerOn::Request> request,
            std::shared_ptr<mg400_msgs::srv::PowerOn::Response> response);
            
    bool runScript(
            const std::shared_ptr<mg400_msgs::srv::RunScript::Request> request,
            std::shared_ptr<mg400_msgs::srv::RunScript::Response> response);

    bool stopScript(
            const std::shared_ptr<mg400_msgs::srv::StopScript::Request> request,
            std::shared_ptr<mg400_msgs::srv::StopScript::Response> response);

    bool pauseScript(
            const std::shared_ptr<mg400_msgs::srv::PauseScript::Request> request,
            std::shared_ptr<mg400_msgs::srv::PauseScript::Response> response);

    bool continueScript(
            const std::shared_ptr<mg400_msgs::srv::ContinueScript::Request> request,
            std::shared_ptr<mg400_msgs::srv::ContinueScript::Response> response);

    bool setSafeSkin(
            const std::shared_ptr<mg400_msgs::srv::SetSafeSkin::Request> request,
            std::shared_ptr<mg400_msgs::srv::SetSafeSkin::Response> response);

    bool setObstacleAvoid(
            const std::shared_ptr<mg400_msgs::srv::SetObstacleAvoid::Request> request,
            std::shared_ptr<mg400_msgs::srv::SetObstacleAvoid::Response> response);

    bool setCollisionLevel(
            const std::shared_ptr<mg400_msgs::srv::SetCollisionLevel::Request> request,
            std::shared_ptr<mg400_msgs::srv::SetCollisionLevel::Response> response);

    bool emergencyStop(
            const std::shared_ptr<mg400_msgs::srv::EmergencyStop::Request> request,
            std::shared_ptr<mg400_msgs::srv::EmergencyStop::Response> response);

    bool movJ(
            const std::shared_ptr<mg400_msgs::srv::MovJ::Request> request,
            std::shared_ptr<mg400_msgs::srv::MovJ::Response> response);

    bool movL(
            const std::shared_ptr<mg400_msgs::srv::MovL::Request> request,
            std::shared_ptr<mg400_msgs::srv::MovL::Response> response);
            
    bool jointMovJ(
            const std::shared_ptr<mg400_msgs::srv::JointMovJ::Request> request,
            std::shared_ptr<mg400_msgs::srv::JointMovJ::Response> response);

    bool jump(
            const std::shared_ptr<mg400_msgs::srv::Jump::Request> request,
            std::shared_ptr<mg400_msgs::srv::Jump::Response> response);  

    bool relMovJ(
            const std::shared_ptr<mg400_msgs::srv::RelMovJ::Request> request,
            std::shared_ptr<mg400_msgs::srv::RelMovJ::Response> response);

    bool relMovL(
            const std::shared_ptr<mg400_msgs::srv::RelMovL::Request> request,
            std::shared_ptr<mg400_msgs::srv::RelMovL::Response> response);

    bool arc(
            const std::shared_ptr<mg400_msgs::srv::Arc::Request> request,
            std::shared_ptr<mg400_msgs::srv::Arc::Response> response);  

    bool circle(
            const std::shared_ptr<mg400_msgs::srv::Circle::Request> request,
            std::shared_ptr<mg400_msgs::srv::Circle::Response> response);

    bool servoJ(
            const std::shared_ptr<mg400_msgs::srv::ServoJ::Request> request,
            std::shared_ptr<mg400_msgs::srv::ServoJ::Response> response);

    bool servoP(
            const std::shared_ptr<mg400_msgs::srv::ServoP::Request> request,
            std::shared_ptr<mg400_msgs::srv::ServoP::Response> response);  

    bool sync(
            const std::shared_ptr<mg400_msgs::srv::Sync::Request> request,
            std::shared_ptr<mg400_msgs::srv::Sync::Response> response);

    bool startTrace(
            const std::shared_ptr<mg400_msgs::srv::StartTrace::Request> request,
            std::shared_ptr<mg400_msgs::srv::StartTrace::Response> response);

    bool startPath(
            const std::shared_ptr<mg400_msgs::srv::StartPath::Request> request,
            std::shared_ptr<mg400_msgs::srv::StartPath::Response> response);  

    bool startFCTrace(
            const std::shared_ptr<mg400_msgs::srv::StartFCTrace::Request> request,
            std::shared_ptr<mg400_msgs::srv::StartFCTrace::Response> response);

    bool moveJog(
            const std::shared_ptr<mg400_msgs::srv::MoveJog::Request> request,
            std::shared_ptr<mg400_msgs::srv::MoveJog::Response> response);



private:
    void feedbackHandle(std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> handle);
    rclcpp_action::GoalResponse handle_goal(const std::array<unsigned char, 16>& uuid,
        std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> handle);
    rclcpp_action::CancelResponse  handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> handle);
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> handle);
};