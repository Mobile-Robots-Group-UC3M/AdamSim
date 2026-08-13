#include <ros/ros.h>


//Service headers
#include <inspire_hand/set_id.h>
#include <inspire_hand/set_redu_ratio.h>
#include <inspire_hand/set_clear_error.h>
#include <inspire_hand/set_save_flash.h>
#include <inspire_hand/set_reset_para.h>
#include <inspire_hand/set_force_clb.h>
#include <inspire_hand/set_gesture_no.h>
#include <inspire_hand/set_current_limit.h>
#include <inspire_hand/set_default_speed.h>
#include <inspire_hand/set_default_force.h>
#include <inspire_hand/set_user_def_angle.h>
#include <inspire_hand/set_pos.h>
#include <inspire_hand/set_angle.h>
#include <inspire_hand/set_force.h>
#include <inspire_hand/set_speed.h>
#include <inspire_hand/set_run_action.h>
#include <inspire_hand/get_pos_act.h>
#include <inspire_hand/get_angle_act.h>
#include <inspire_hand/get_force_act.h>
#include <inspire_hand/get_current.h>
#include <inspire_hand/get_error.h>
#include <inspire_hand/get_status.h>
#include <inspire_hand/get_temp.h>
#include <inspire_hand/get_pos_set.h>
#include <inspire_hand/get_angle_set.h>
#include <inspire_hand/get_force_set.h>

#include <cstdlib>
#include "hand_control.h"
//#include <vector>
#include <fstream>
#include <iostream>
#include <ctime>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hand_control_client");
    //  if (argc != 3)
    //  {
    //    ROS_INFO("usage: test hand");
    //    return 1;
    //  }

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<inspire_hand::set_pos>("inspire_hand/set_pos");
    ros::ServiceClient client1 = nh.serviceClient<inspire_hand::get_pos_act>("inspire_hand/get_pos_act");
    ros::ServiceClient client2 = nh.serviceClient<inspire_hand::set_speed>("inspire_hand/set_speed");
    ros::ServiceClient client3 = nh.serviceClient<inspire_hand::set_force>("inspire_hand/set_force");
    ros::ServiceClient client4 = nh.serviceClient<inspire_hand::get_force_act>("inspire_hand/get_force_act");

    inspire_hand::set_pos srv;
    inspire_hand::get_pos_act srv_getpos;
    inspire_hand::set_speed srv_speed;
    inspire_hand::set_force srv_setforce;
    inspire_hand::get_force_act srv_getforce_act;

    int i =0;
    std::vector<float> joint_pos(6);

    ofstream outfile0;
    ofstream outfile1;
    ofstream outfile2;
    ofstream outfile3;
    ofstream outfile4;
    ofstream outfile5;
    ofstream outfile_time_force;

    //  ofstream outfile_time;
    //  ofstream outfile_plan;
    outfile0.open("/home/robot/hand_ws/src/inspire_hand/src/data/0_force.txt");
    outfile1.open("/home/robot/hand_ws/src/inspire_hand/src/data/1_force.txt");
    outfile2.open("/home/robot/hand_ws/src/inspire_hand/src/data/2_force.txt");
    outfile3.open("/home/robot/hand_ws/src/inspire_hand/src/data/3_force.txt");
    outfile4.open("/home/robot/hand_ws/src/inspire_hand/src/data/4_force.txt");
    outfile5.open("/home/robot/hand_ws/src/inspire_hand/src/data/force_set.txt");
    outfile_time_force.open("/home/robot/hand_ws/src/inspire_hand/src/data/time_force.txt");

    //  outfile_time.open("/home/wukong/inspire_robot_test/src/data/time.txt");
    //  outfile_plan.open("/home/wukong/inspire_robot_test/src/data/0_plan.txt");

    ROS_INFO("READY!");
    //initialization fingers pos
    srv.request.pos0=200;
    srv.request.pos1=200;
    srv.request.pos2=200;
    srv.request.pos3=200;
    srv.request.pos4=200;
    srv.request.pos5=1800;
    client.call(srv);

    ros::Duration(0.015).sleep();

    srv_setforce.request.force0=500;
    srv_setforce.request.force1=500;
    srv_setforce.request.force2=500;
    srv_setforce.request.force3=500;
    srv_setforce.request.force4=500;

    client3.call(srv_setforce);
    ROS_INFO("set force OK");

    //    srv.request.pos0=1800;
    //    srv.request.pos1=1800;
    //    srv.request.pos2=1800;
    //    srv.request.pos3=1800;
    //    srv.request.pos4=1800;
    srv.request.pos5=1800;
    client.call(srv);
    ROS_INFO("target pos sent");

    srv_speed.request.speed0 = 200;
    srv_speed.request.speed1 = 200;
    srv_speed.request.speed2 = 200;
    srv_speed.request.speed3 = 200;
    srv_speed.request.speed4 = 200;

    if(client2.call(srv_speed)){
        ROS_INFO("speed set OK");
    }


    //fingers force test
    while(ros::ok()){
        if(client4.call(srv_getforce_act)){

            outfile0<<srv_getforce_act.response.curforce[0]<<endl;
            outfile1<<srv_getforce_act.response.curforce[1]<<endl;
            outfile2<<srv_getforce_act.response.curforce[2]<<endl;
            outfile3<<srv_getforce_act.response.curforce[3]<<endl;
            outfile4<<srv_getforce_act.response.curforce[4]<<endl;
            outfile_time_force<<ros::Time::now()<<endl;
            ROS_INFO("WRITE OK");
        }
        else {
            ROS_INFO("SERVICE GET FORCE NOT CALL");
        }



    }

    //  outfile_plan.close();
    //  outfile_time.close();
    outfile0.close();
    outfile1.close();
    outfile2.close();
    outfile3.close();
    outfile4.close();
    outfile5.close();
    outfile_time_force.close();



    return 0;
}
