#ifndef UWB_POSITION_PUB_H
#define UWB_POSITION_PUB_H

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<mavros_msgs/CommandBool.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/State.h>
#include"nlink_parser/LinktrackAnchorframe0.h"
#include"nlink_parser/LinktrackNodeframe2.h"
#include<eigen3/Eigen/Dense>
#include"state_from_mavlink.h"
#include"common_msg/DroneState.h"


class uwb_position_pub
{   
    public:
    uwb_position_pub(void):
        uwb_position_pub_node("~")
        {
            pose_t_drone.pose.position.x=0;
            pose_t_drone.pose.position.y=0;
            pose_t_drone.pose.position.z=0;

            pose_t_drone.pose.orientation.x=0;
            pose_t_drone.pose.orientation.y=0;
            pose_t_drone.pose.orientation.z=0;
            pose_t_drone.pose.orientation.w=1;

            sub_from_uwb=uwb_position_pub_node.subscribe<nlink_parser::LinktrackNodeframe2>("/nlink_linktrack_nodeframe2",10,&uwb_position_pub::position_from_uwb_cb,this);
            pub_to_mavlink=uwb_position_pub_node.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10);
            opticalFlowRad_sub=uwb_position_pub_node.subscribe<mavros_msgs::OpticalFlowRad>("/mavros/px4flow/raw/optical_flow_rad",10,&uwb_position_pub::opticalFlowRad_cb,this);
        }

    geometry_msgs::PoseStamped pose_t_drone;

    float opticalFlowRad_distance;

    void pub_position_to_drone();

    private:
        ros::NodeHandle uwb_position_pub_node;
        ros::Publisher pub_to_mavlink;
        ros::Subscriber sub_from_uwb;
        ros::Subscriber opticalFlowRad_sub;

        void position_from_uwb_cb(const nlink_parser::LinktrackNodeframe2::ConstPtr & msg)
        {
            pose_t_drone.header.stamp=ros::Time::now();
            pose_t_drone.header.frame_id="map";

            pose_t_drone.pose.position.x=msg->pos_3d[0];
            pose_t_drone.pose.position.y=msg->pos_3d[1];
            pose_t_drone.pose.position.z=msg->pos_3d[2];

            pose_t_drone.pose.orientation.x=msg->quaternion[0];
            pose_t_drone.pose.orientation.y=msg->quaternion[1];
            pose_t_drone.pose.orientation.z=msg->quaternion[2];
            pose_t_drone.pose.orientation.w=msg->quaternion[3];
        }

        void opticalFlowRad_cb(const mavros_msgs::OpticalFlowRad::ConstPtr & msg)
        {
            opticalFlowRad_distance=msg->distance;
        }

};

void uwb_position_pub::pub_position_to_drone()
{
    pose_t_drone.pose.position.z=opticalFlowRad_distance;
    std::cout<<pose_t_drone.pose.position.x<<""<<pose_t_drone.pose.position.y<<""<<pose_t_drone.pose.position.z<<""<<std::endl;
    pub_to_mavlink.publish(pose_t_drone);
}

#endif