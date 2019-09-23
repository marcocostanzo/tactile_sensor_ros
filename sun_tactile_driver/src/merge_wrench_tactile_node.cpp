/*
    ROS node to read convert voltages msg type

    Copyright 2018 Università della Campania Luigi Vanvitelli

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <ros/ros.h>

#include "sun_tactile_common/TactileStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"


using namespace std;

//*******GLOBAL ROS VARS******//
ros::Publisher pubCalib;
int num_voltages;
//----------------------------//

// ==== Tactile msg ====
std_msgs::Float64MultiArray out_msg;

void readV( const sun_tactile_common::TactileStamped::ConstPtr& msg  ){
	
    for(int i = 0 ; i < (num_voltages); i++){
        out_msg.data[7+i] = msg->tactile.data[i];
    }    
    out_msg.data[7+num_voltages] = msg->header.stamp.toSec();
    
    pubCalib.publish(out_msg);
	
}

void readW( const geometry_msgs::WrenchStamped::ConstPtr& msg  ){
	
    out_msg.data[0] = msg->wrench.force.x;
    out_msg.data[1] = msg->wrench.force.y;
    out_msg.data[2] = msg->wrench.force.z;
    out_msg.data[3] = msg->wrench.torque.x;
    out_msg.data[4] = msg->wrench.torque.y;
    out_msg.data[5] = msg->wrench.torque.z;      
    out_msg.data[6] = msg->header.stamp.toSec();

    pubCalib.publish(out_msg);
	
}


//==============MAIN================//

int main(int argc, char *argv[]){

    ros::init(argc,argv,"merge_wrench_tactile");

    ros::NodeHandle nh_public;
    ros::NodeHandle n("~");

    /**** CHECK PARAMS ****/
    string voltage_topic = string("");
    n.param("voltage_topic" , voltage_topic, string("/tactile") );
    string wrench_topic = string("");
    n.param("wrench_topic" , wrench_topic, string("/rft_data") );
    string out_topic = string("");
    n.param("out_topic" , out_topic, string("/calib_data") );
    n.param("num_voltages" , num_voltages, 25 );
    //double hz;
    //n->param("rate" , hz, 333.0 );
   
   // ======= PUBLISHER & SUB
   ros::Subscriber subTactile = nh_public.subscribe(voltage_topic ,1,readV);
   ros::Subscriber subWrench = nh_public.subscribe(wrench_topic ,1,readW);

   pubCalib = nh_public.advertise<std_msgs::Float64MultiArray>( out_topic ,1);

   out_msg.data.resize(6+1+num_voltages+1);

   /*ros::Rate loop_rate(hz);
   while (ros::ok())   {
       
       ros::spinOnce();
       pubCalib.publish(out_msg);
       loop_rate.sleep();
   }*/

   ros::spin();



}

