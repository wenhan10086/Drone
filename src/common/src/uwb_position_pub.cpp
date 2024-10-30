#include"uwb_position_pub.h"
#include"command_to_mavros.h"
#include"state_from_mavlink.h"
#include"iostream"



int main(int argc,char ** argv)
{
    ros::init(argc,argv,"uwb_position_pub");

    uwb_position_pub _uwb_position_pub;
    state_from_mavros _state_from_mavros;
    command_to_mavros _command_to_mavros;
    Eigen::Vector3d _target_pose(1,1,1);
    ros::Rate rate(50);
    //读一些状态
    for(int i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }
    while(_state_from_mavros._DroneState.connected&&_state_from_mavros._DroneState.armed)
    {
        ros::spinOnce();
        rate.sleep();
    }
    //进入主循环
    while(ros::ok())
    {
        
        //进入状态机
        if(_state_from_mavros._DroneState.mode=="OFFBOARD")
        {
            _uwb_position_pub.pub_position_to_drone();
            rate.sleep();
            _command_to_mavros.send_pos_setpoint(_target_pose,0.01);
            rate.sleep();
            std::cout<< _state_from_mavros._DroneState.position[0]<<""
            <<_state_from_mavros._DroneState.position[1] <<""
            <<_state_from_mavros._DroneState.position[2]<<std::endl;
        
        }
        for(int i=0;i<5;i++)
        {
            ros::spinOnce();
            rate.sleep();
        }
  
    }

      return 0;

}
