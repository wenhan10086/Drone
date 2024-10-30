#ifndef STATE_FROM_MAVLINK_H
#define STATE_FROM_MAVLINK_H

/***************************************************************************************************************************
* state_from_mavros.h
*
* Author: SFL
*
* Update Time: 2021.4.16
*
* 主要功能：
*    本库函数主要用于连接wrzf_pkg与mavros两个功能包。
* 1、订阅mavros功能包发布的飞控状态量。状态量包括无人机状态、位置、经纬度、GPS状态、速度、角度、角速度。
*     注： 这里并没有订阅所有可以来自飞控的消息，如需其他消息，请参阅mavros代码。
*     注意：代码中，参与运算的角度均是以rad为单位，但是涉及到显示时或者需要手动输入时均以deg为单位。
*
***************************************************************************************************************************/

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <bitset>
#include "common_msg/DroneState.h"
#include <unordered_map>

class state_from_mavros
{
    public:
    //constructed function
    state_from_mavros(void):
        state_nh("~")
    {
        // 【订阅】无人机当前状态 - 来自飞控
        //  本话题来自飞控(通过Mavros功能包 /plugins/sys_status.cpp)
        state_sub = state_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &state_from_mavros::state_cb,this);

        // 【订阅】无人机当前位置 坐标系:ENU系 （此处注意，所有状态量在飞控中均为NED系，但在ros中mavros将其转换为ENU系处理。所以，在ROS中，所有和mavros交互的量都为ENU系）
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        position_sub = state_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &state_from_mavros::pos_cb,this);

        // 【订阅】无人机当前速度 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        velocity_sub = state_nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, &state_from_mavros::vel_cb,this);

        // 【订阅】无人机当前速度 坐标系:body系
        //  本话题来自飞控(通过Mavros功能包 /plugins/local_position.cpp读取), 对应Mavlink消息为LOCAL_POSITION_BODY (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
        velocity_body_sub = state_nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 10, &state_from_mavros::vel_body_cb,this);

        // 【订阅】无人机当前欧拉角 坐标系:ENU系
        //  本话题来自飞控(通过Mavros功能包 /plugins/imu.cpp读取), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
        attitude_sub = state_nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &state_from_mavros::att_cb,this);

        // 【订阅】无人机GPS当前状态 - 来自飞控
        //  本话题来自飞控(通过Mavros功能包)
        gps_sub = state_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/raw/fix", 10, &state_from_mavros::gps_cb,this);
        
        // 【订阅】无人机全局位置 - 来自飞控
        //  本话题来自飞控(通过Mavros功能包)
        global_sub = state_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &state_from_mavros::global_cb,this);

       
    }

    //变量声明 
    common_msg::DroneState _DroneState;


    private:

        ros::NodeHandle state_nh;

        ros::Subscriber state_sub;
        ros::Subscriber position_sub;
        ros::Subscriber velocity_sub;
        ros::Subscriber velocity_body_sub;
        ros::Subscriber attitude_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber global_sub;
        
        Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) 
        {
            // 确保四元数是单位四元数
            Eigen::Quaterniond normalized_q = q.normalized();

            // 提取四元数的分量
            double x = normalized_q.x();
            double y = normalized_q.y();
            double z = normalized_q.z();
            double w = normalized_q.w();

            // 计算滚转角（roll）
            double roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));

            // 计算俯仰角（pitch）
            double pitch = asin(2.0 * (w * y - z * x));

            // 计算偏航角（yaw）
            double yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

            return Eigen::Vector3d(roll, pitch, yaw);
        }       
        void state_cb(const mavros_msgs::State::ConstPtr &msg)
        {
            this->_DroneState.connected = msg->connected;
            this->_DroneState.armed = msg->armed;
            this->_DroneState.mode = msg->mode;
        }

        void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
        {
            this->_DroneState.gps_status = msg->status.status;
            this->_DroneState.latitude = msg->latitude;
            this->_DroneState.longitude = msg->longitude;
            this->_DroneState.altitude = msg->altitude;
        }

        void global_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
        {
            this->_DroneState.latitude = msg->latitude;
            this->_DroneState.longitude = msg->longitude;
            this->_DroneState.altitude = msg->altitude;
        }

        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            this->_DroneState.position[0] = msg->pose.position.x;
            this->_DroneState.position[1] = msg->pose.position.y;
            this->_DroneState.position[2] = msg->pose.position.z;
        }

        void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
        {
            this->_DroneState.velocity[0] = msg->twist.linear.x;
            this->_DroneState.velocity[1] = msg->twist.linear.y;
            this->_DroneState.velocity[2] = msg->twist.linear.z;
        }

        void vel_body_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
        {
            this->_DroneState.velocity_body[0] = msg->twist.linear.x;
            this->_DroneState.velocity_body[1] = msg->twist.linear.y;
            this->_DroneState.velocity_body[2] = msg->twist.linear.z;
        }

        void att_cb(const sensor_msgs::Imu::ConstPtr& msg)
        {
            Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
            //Transform the Quaternion to euler Angles
            Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
            
            this->_DroneState.attitude_q.w = q_fcu.w();
            this->_DroneState.attitude_q.x = q_fcu.x();
            this->_DroneState.attitude_q.y = q_fcu.y();
            this->_DroneState.attitude_q.z = q_fcu.z();

            this->_DroneState.attitude[0] = euler_fcu[0];
            this->_DroneState.attitude[1] = euler_fcu[1];
            this->_DroneState.attitude[2] = euler_fcu[2];

           this-> _DroneState.attitude_rate[0] = msg->angular_velocity.x;
           this-> _DroneState.attitude_rate[1] = msg->angular_velocity.y;
           this-> _DroneState.attitude_rate[2] = msg->angular_velocity.z;
        }

};


#endif