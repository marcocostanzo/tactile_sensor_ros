/*
    ROS node to combine wrenches of fingers

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

#include <geometry_msgs/WrenchStamped.h>
#include <sun_ros_msgs/Float64Stamped.h>

#include <TooN/TooN.h>

using namespace std;
using namespace TooN;

//==========GLOBAL VARS========//
Matrix<3,3> e_R_0 = Identity, e_R_1 = Identity;
Vector<3> _0_r_e = Zeros, _1_r_e = Zeros;


//*******GLOBAL ROS VARS******//

//----------------------------//
//*******PUBLISHERS**********//
ros::Publisher pubWrench;
ros::Publisher pubGraspForce;
//----------------------------//
//**********MSGS*************//
geometry_msgs::WrenchStamped totalWrench_msg;
sun_ros_msgs::Float64Stamped graspFroce_msg;
//====================================//


//=========LOCAL FCNs INTERFACEs=======//

geometry_msgs::WrenchStamped trWrench( const geometry_msgs::WrenchStamped::ConstPtr& msg , const Matrix<3,3>& R, const Vector<3>& r);
void updateWrench();

//====================================//


//==========SERVICES=========//

//====================================//


//==========TOPICs CALLBKs=========//

geometry_msgs::WrenchStamped trWrench0;
double fz0 = 0.0;
void readWrench0( const geometry_msgs::WrenchStamped::ConstPtr& msg ){

	fz0 = msg->wrench.force.z;			 
    trWrench0 = trWrench( msg, e_R_0 , _0_r_e );	 
    updateWrench();

}

geometry_msgs::WrenchStamped trWrench1;
double fz1 = 0.0;
void readWrench1( const geometry_msgs::WrenchStamped::ConstPtr& msg ){
	 
	fz1 = msg->wrench.force.z;
    trWrench1 = trWrench( msg, e_R_1 , _1_r_e );
    updateWrench();

}

double distance_offset;
void readDistance( const sun_ros_msgs::Float64Stamped::ConstPtr& msg ){
	 
	double distance = ((msg->data)/2.0)+distance_offset;

	_0_r_e[2] = distance;
	_1_r_e[2] = distance;

}


//====================================//


//==============MAIN================//

int main(int argc, char *argv[]){

	ros::init(argc,argv,"combine_wrench");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    /***** Params **********/ // (FROM FILE!)
    e_R_0 = Data(1.0, 0.0,  0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0);

    e_R_1 = Data(-1.0, 0.0,  0.0,
                  0.0, 1.0,  0.0,
                  0.0, 0.0, -1.0);
   
   //From topic (OR FROM FILE)
    _0_r_e = makeVector(0.0, 0.0, 0.04);
    _1_r_e = makeVector(0.0, 0.0, 0.04);

    string topic_0_str("");
    nh_private.param("wrench0_topic" , topic_0_str, string("wrench0") );
    string topic_1_str("");
    nh_private.param("wrench1_topic" , topic_1_str, string("wrench1") );
    string topic_distance_str("");
    nh_private.param("distance_topic" , topic_distance_str, string("distance") );
	string frame_id("");
    string tf_prefix("");
    nh_private.param("tf_prefix" , tf_prefix, string("") );
    nh_private.param("frame_id" , frame_id, string("grasp_frame") );
    nh_private.param("distance_offset" , distance_offset, 0.0 );

    string out_topic_str("");
    nh_private.param("wrench_out_topic" , out_topic_str, string("total_wrench") );
	string grasp_force_topic_str("");
    nh_private.param("grasp_force_topic" , grasp_force_topic_str, string("grasp_force") );

    ros::Subscriber subWrench0 = nh_public.subscribe( topic_0_str ,1, readWrench0);
    ros::Subscriber subWrench1 = nh_public.subscribe( topic_1_str ,1, readWrench1);
	ros::Subscriber subDistance = nh_public.subscribe( topic_distance_str, 1, readDistance );;

    /*******INIT ROS PUB**********/
    //Force pub
	pubWrench = nh_public.advertise<geometry_msgs::WrenchStamped>( out_topic_str, 1);
	pubGraspForce = nh_public.advertise<sun_ros_msgs::Float64Stamped>( grasp_force_topic_str, 1); 
    /***************************/

	totalWrench_msg.header.frame_id = tf_prefix+frame_id;


/*============LOOP==============*/
    ros::spin();
/*==============================*/
	
	return 0;
}



/*===========LOCAL FCNs IMPL==========*/


geometry_msgs::WrenchStamped trWrench( const geometry_msgs::WrenchStamped::ConstPtr& msg , const Matrix<3,3>& R, const Vector<3>& r){

    Vector<3> f;
    f[0] = msg->wrench.force.x;
    f[1] = msg->wrench.force.y;
    f[2] = msg->wrench.force.z;

    Vector<3> m;
    m[0] = msg->wrench.torque.x;
    m[1] = msg->wrench.torque.y;
    m[2] = msg->wrench.torque.z;

    Vector<3> f_tr;
    Vector<3> m_tr;

    f_tr = R*f;
    m_tr = R*( (r ^ f) + m );

    geometry_msgs::WrenchStamped outMsg;

    outMsg.wrench.force.x = f_tr[0];
    outMsg.wrench.force.y = f_tr[1];
    outMsg.wrench.force.z = f_tr[2];

    outMsg.wrench.torque.x = m_tr[0];
    outMsg.wrench.torque.y = m_tr[1];
    outMsg.wrench.torque.z = m_tr[2];

    if(msg->header.stamp > totalWrench_msg.header.stamp)
        totalWrench_msg.header.stamp = msg->header.stamp;
    else
        totalWrench_msg.header.stamp += ros::Duration(1.0E-6);

    return outMsg;

}

void updateWrench(){

    totalWrench_msg.wrench.force.x = trWrench0.wrench.force.x + trWrench1.wrench.force.x;
    totalWrench_msg.wrench.force.y = trWrench0.wrench.force.y + trWrench1.wrench.force.y;
    totalWrench_msg.wrench.force.z = trWrench0.wrench.force.z + trWrench1.wrench.force.z;

    totalWrench_msg.wrench.torque.x = trWrench0.wrench.torque.x + trWrench1.wrench.torque.x;
    totalWrench_msg.wrench.torque.y = trWrench0.wrench.torque.y + trWrench1.wrench.torque.y;
    totalWrench_msg.wrench.torque.z = trWrench0.wrench.torque.z + trWrench1.wrench.torque.z;

    // totalWrench_msg.wrench.force.x = 2.0*trWrench1.wrench.force.x;
    // totalWrench_msg.wrench.force.y = trWrench1.wrench.force.y;
    // totalWrench_msg.wrench.force.z = trWrench1.wrench.force.z;

    // totalWrench_msg.wrench.torque.x = trWrench1.wrench.torque.x;
    // totalWrench_msg.wrench.torque.y = trWrench1.wrench.torque.y;
    // totalWrench_msg.wrench.torque.z = 2.0*trWrench1.wrench.torque.z;

	graspFroce_msg.data = 2.0*min(fabs(fz0),fabs(fz1));
    // graspFroce_msg.data = fabs(fz0)+fabs(fz1);
    // graspFroce_msg.data = 2.0*fabs(fz1);
    graspFroce_msg.header = totalWrench_msg.header;

    //totalWrench_msg.wrench.force.z = graspFroce_msg.data;

    pubWrench.publish(totalWrench_msg);
	pubGraspForce.publish(graspFroce_msg);

}
