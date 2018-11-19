/*
    ROS node to rotate forces on the contact plane

    Copyright 2018 Università della Campania Luigi Vanvitelli

	Author: Marco Costanzo <marco.costanzo@unicampania.it>

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

#include "ros/ros.h"

#include "geometry_msgs/WrenchStamped.h"

#include "sun_tactile_common/TactileStamped.h"

#include "geometry_msgs/Vector3.h"

#include "TooN/TooN.h"
#include "TooN/SVD.h"

using namespace TooN;
using namespace std;

#define NUM_R 5
#define NUM_C 5

//Params
Matrix<NUM_R,NUM_C> XX = Data(
                            -1.0, -0.5, 0.0, 0.5, 1.0,
                            -1.0, -0.5, 0.0, 0.5, 1.0,
                            -1.0, -0.5, 0.0, 0.5, 1.0,
                            -1.0, -0.5, 0.0, 0.5, 1.0,
                            -1.0, -0.5, 0.0, 0.5, 1.0
                        );

Matrix<NUM_R,NUM_C> YY = -XX.T();

double R = 0.05; //m
double voltage_thr = 0.005;
Vector<3> s_P_sf = makeVector(0.0,0.0, -0.02); //m

ros::Publisher pub_force;
ros::Publisher pub_centroid;

Matrix<4,4> c_T_s = Identity;
void volt_Callback (const sun_tactile_common::TactileStamped::ConstPtr& msg) {
   
    Matrix<NUM_R,NUM_C> voltages;
    bool volts_low = true;
    double sumV = 0.0;
    double cx,cy;


    for(int i = 0; i< NUM_R ; i++){

        for(int j = 0; j<NUM_C; j++){
 
		    voltages[i][j] = msg->tactile.data[i*NUM_C + j];
            sumV += voltages[i][j];
  
            //check if volt < thr
            if( voltages[i][j] > voltage_thr){
                volts_low = false;
            }

            cx += voltages[i][j]*XX[i][j];
            cy += voltages[i][j]*YY[i][j];

        }
	}

    if(volts_low){
        cx = 0.0;
        cy = 0.0;
    } else{
        cx /= sumV;
        cy /= sumV;
    }

    Vector<3> sf_P_cont = makeVector( cx, cy, sqrt( pow(R,2) - pow(cx,2) - pow(cy,2) ) );

    Vector<3> n = sf_P_cont/R;

    Vector<3> s_x_cont = unit( makeVector(1.0,0.0,0.0) - n[0]*n);

    Vector<3> s_y_cont = n ^ s_x_cont;

    Matrix<4,4> s_T_cont = Identity;

    s_T_cont.T()[0].slice<0,3>() = s_x_cont;
    s_T_cont.T()[1].slice<0,3>() = s_y_cont;
    s_T_cont.T()[2].slice<0,3>() = n;

    s_T_cont.T()[3].slice<0,3>() = sf_P_cont + s_P_sf;

    SVD<> s_T_cont_SVD(s_T_cont);

    c_T_s = s_T_cont_SVD.get_pinv();

    geometry_msgs::Vector3 msg_centroid;
    msg_centroid.x = cx;
    msg_centroid.y = cy;

    pub_centroid.publish(msg_centroid);
  
}


void wrench_Callback (const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    
    Vector<3> s_force = makeVector( msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    Vector<3> s_torque = makeVector( msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);

    Vector<3> c_force = c_T_s.slice<0,0,3,3>() * s_force;
    Vector<3> c_torque = c_T_s.slice<0,0,3,3>() * ( (c_T_s.T()[3].slice<0,3>() ^ s_force) +  s_torque);

    geometry_msgs::WrenchStamped msg_out_force;
    msg_out_force.wrench.force.x = c_force[0];
    msg_out_force.wrench.force.y = c_force[1];
    msg_out_force.wrench.force.z = c_force[2];
    msg_out_force.wrench.torque.x = c_torque[0];
    msg_out_force.wrench.torque.y = c_torque[1];
    msg_out_force.wrench.torque.z = c_torque[2];

    geometry_msgs::WrenchStamped msg_out_grasp_force;
    msg_out_force.wrench.force.z = c_force[2];


    pub_force.publish(msg_out_force);
   
}

int main(int argc, char *argv[])
{
    

    ros::init(argc, argv, "rotate_force");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    string voltages_topic("");
    nh_private.param("voltages_topic" , voltages_topic, string("volts") );
    string in_force_topic("");
    nh_private.param("in_force_topic" , in_force_topic, string("in_force") );
    string out_force_topic("");
    nh_private.param("out_force_topic" , out_force_topic, string("out_force") );

    string out_centroid_topic("");
    nh_private.param("out_centroid_topic" , out_centroid_topic, string("out_centroid") );

    nh_private.param("radius" , R, 0.05 );
    nh_private.param("voltage_thr" , voltage_thr, 0.005 );

    double tmp_distance_sensor_sphere = -0.02;
    nh_private.param("distance_sensor_sphere" , tmp_distance_sensor_sphere, -0.02 );
    s_P_sf[2] = tmp_distance_sensor_sphere;

    double cell_distance_coeff = 0.007;
    nh_private.param("cell_distance_coeff" , cell_distance_coeff, 0.007 );

    XX = XX*cell_distance_coeff;
    YY = YY*cell_distance_coeff;

    ros::Subscriber sub_wrench = nh_public.subscribe(in_force_topic, 1, wrench_Callback);
    ros::Subscriber sub_volt = nh_public.subscribe(voltages_topic, 1, volt_Callback);

    pub_force = nh_public.advertise<geometry_msgs::WrenchStamped>(out_force_topic, 1);
    pub_centroid = nh_public.advertise<geometry_msgs::Vector3>(out_centroid_topic, 1);

    ros::spin();

    return 0;
}