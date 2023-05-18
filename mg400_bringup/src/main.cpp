/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <mg400_msgs/msg/robot_status.hpp>
#include <mg400_msgs/msg/tool_vector_actual.hpp>
#include <mg400_bringup/robot.hpp>
#include <mg400_bringup/convert.hpp>
#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    // auto private_node = std::make_shared<rclcpp::Node>("MG400Robot");
    // try
    // {

    //     auto joint_state_pub = private_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);
    //     mg400_msgs::msg::RobotStatus robot_status_msg;
    //     auto robot_status_pub = private_node->create_publisher<mg400_msgs::msg::RobotStatus>("RobotStatus", 100);

    //     mg400_msgs::msg::ToolVectorActual tool_vector_actual_msg;
    //     auto tool_vector_pub = private_node->create_publisher<mg400_msgs::msg::ToolVectorActual>("ToolVectorActual", 100);

    //     MG400Robot robot(private_node, "/follow_joint_trajectory/follow_joint_trajectory");

    //     double rate_value = private_node->declare_parameter("JointStatePublishRate", 10.0);

    //     robot.init();
    //     rclcpp::Rate rate(rate_value);
    //     double position[6];
    //     while (rclcpp::ok())
    //     {
    //         //
    //         // publish joint state
    //         //
    //         robot.getJointState(position);
    //         joint_state_pub->publish(mg400_bringup::Convert::toJointState(position[0], position[1], position[2], position[3]));

    //         double val[6];
    //         robot.getToolVectorActual(val);
    //         tool_vector_actual_msg.x = val[0];
    //         tool_vector_actual_msg.y = val[1];
    //         tool_vector_actual_msg.z = val[2];
    //         tool_vector_actual_msg.r = val[3];
    //         tool_vector_pub->publish(tool_vector_actual_msg);

    //         //
    //         // publish robot status
    //         //
    //         robot_status_msg.is_enable = robot.isEnable();
    //         robot_status_msg.robot_status = robot.robotStatus();
    //         robot_status_msg.is_connected = robot.isConnected();
    //         robot_status_pub->publish(robot_status_msg);

    //         rclcpp::spin_some(private_node);
    //         rate.sleep();
    //     }
    // }
    // catch (const std::exception& err)
    // {
    //     RCLCPP_ERROR(private_node->get_logger(), "%s", err.what());
    //     return -1;
    // }

    rclcpp::shutdown();

    return 0;
}
