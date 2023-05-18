/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#ifndef CONVERT_H
#define CONVERT_H

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.h>

class Convert
{
public:
    static sensor_msgs::msg::JointState::SharedPtr toJointState(double j1, double j2, double j3, double j4)
    {
       sensor_msgs::msg::JointState::SharedPtr ptr = std::shared_ptr<sensor_msgs::msg::JointState>(new sensor_msgs::msg::JointState());

        ptr->header.stamp = rclcpp::Clock().now();
        ptr->name.push_back("j1");
        ptr->name.push_back("j2_1");
        ptr->name.push_back("j2_2");
        ptr->name.push_back("j3_1");
        ptr->name.push_back("j3_2");
        ptr->name.push_back("j4_1");
        ptr->name.push_back("j4_2");
        ptr->name.push_back("j5");

        ptr->position.push_back(j1);
        ptr->position.push_back(j2);
        ptr->position.push_back(j2);
        ptr->position.push_back(j3 - j2);
        ptr->position.push_back(-j2);
        ptr->position.push_back(-j3);
        ptr->position.push_back(j3);
        ptr->position.push_back(j4);


//        msg.header.stamp = ros::Time::now();

//        msg.position.push_back(j1_);
//        msg.position.push_back(j2_1_);
//        msg.position.push_back(j2_2_);
//        msg.position.push_back(j3_1_);
//        msg.position.push_back(j3_2_);
//        msg.position.push_back(j4_1_);
//        msg.position.push_back(j4_2_);
//        msg.position.push_back(j5_);

        return ptr;
    }
};

#endif    // CONVERT_H