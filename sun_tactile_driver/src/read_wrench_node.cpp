/*
    ROS node to calculate wrenches

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

#include <ros/ros.h>

#include <stdio.h>
#include <iostream>

#include <sun_tactile_common/TactileStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <ros/package.h>


//------new
#include <ANN/ANN.h>

using namespace std;
using namespace TooN;

//==========GLOBAL VARS========//
const int NUM_V = 25; //Length of voltage Vector
Vector<NUM_V> voltages = Zeros;
Vector<6> wrench = Zeros;
string fingerCode("");
ANN * myANN;
bool b_pca = false;
Vector<NUM_V> pca_mean;
Vector<>* volt_reduce;
Matrix<>* Ureduce;
string in_voltage_topic_str;
//*******GLOBAL ROS VARS******//
ros::NodeHandle* nh_public;
//----------------------------//
//*******PUBLISHERS**********//
ros::Publisher pubWrench;
//----------------------------//
//**********MSGS*************//
geometry_msgs::WrenchStamped msgWrench;
//====================================//


//=========LOCAL FCNs INTERFACEs=======//

//Model init, NB: Call it before Voltage-Subscriber definition and after Wrench-Publisher definition
void init_model( string path );
void init_ANN_model( string path );

//====================================//


//==========SERVICES=========//

//====================================//


//==========TOPICs CALLBKs=========//
void readV( const sun_tactile_common::TactileStamped::ConstPtr& msg  ){
    
	for(int i = 0; i<NUM_V; i++){
		voltages[i] = msg->tactile.data[i];
	}
    
    //Calculate wrench model
    if(b_pca){
        Vector<NUM_V> votages_pca_mean = voltages - pca_mean;
		//((*Ureduce).T())*votages_pca_mean;
        *volt_reduce = ((*Ureduce).T())*votages_pca_mean;
        wrench = myANN->compute( *volt_reduce );
    } else {
        wrench = myANN->compute( voltages );
    }       

    //Fill ROS msgs
    msgWrench.header.stamp = msg->header.stamp;
    msgWrench.header.frame_id = msg->header.frame_id;
    msgWrench.wrench.force.x = wrench[0];
	msgWrench.wrench.force.y = wrench[1];
	msgWrench.wrench.force.z = wrench[2];
	msgWrench.wrench.torque.x = wrench[3];
	msgWrench.wrench.torque.y = wrench[4];
	msgWrench.wrench.torque.z = wrench[5];

    //Publish
    pubWrench.publish( msgWrench );

}

//====================================//


//==============MAIN================//

int main(int argc, char *argv[]){

	ros::init(argc,argv,"read_wrench");

	ros::NodeHandle nh_private = ros::NodeHandle("~");
    nh_public = new ros::NodeHandle();

    
    /**** CHECK PARAMS ****/
    string path("");
    path = ros::package::getPath("sun_tactile_driver");
    path = path + "/Finger_files/";
    nh_private.param("fingerCode" , fingerCode, string("") );

    nh_private.param("in_voltage_topic" , in_voltage_topic_str, string("tactile_voltage/rect") );
    string wrench_topic_str;
    nh_private.param("wrench_topic" , wrench_topic_str, string("wrench") );

    if( fingerCode.empty() ){
        cout << BOLDRED << "Error! - No params for 'fingerCode' - stopping node... " << CRESET << endl;
        return -1; 
    }

	sleep(1);
    path = path + fingerCode;

    path = path + "/ANNCalib";

    //path = path + fingerCode + "/LinearCalib/K_" + fingerCode  + ".txt";
    
    //name_space = ros::this_node::getNamespace();

    cout << BOLDBLUE << "Finger " << fingerCode << endl;
    /*************************************************/

    /*******INIT ROS PUB**********/
    //Force pub
	pubWrench = nh_public->advertise<geometry_msgs::WrenchStamped>( wrench_topic_str ,1);
    /***************************/

    /*******INIT MODEL**********/
    init_model(path); //Don't move from here!
    /***************************/
	

    /*******INIT ROS SUB**********/
	//Voltage subscriber
	ros::Subscriber subVoltage = nh_public->subscribe( in_voltage_topic_str, 1, readV);
    /***************************/



/*============LOOP==============*/
    ros::spin();
/*==============================*/

	
	return 0;
}



/*===========LOCAL FCNs IMPL==========*/

void init_model(string path){

    init_ANN_model( path );
	cout << BOLDBLUE << "ANN Model" << CRESET << endl;
	//myANN->display(); //debug line

    cout << BOLDGREEN << "Model initialized!" << CRESET << endl;

}

void init_ANN_model( string path ){

		myANN = new ANN ( path );

        string pca_meta_path = path + "/pca.txt";
        Vector<1> pca_meta = readFileV(pca_meta_path.c_str(),1);

        if(pca_meta[0] > 0){
            b_pca = true;

            string pca_mean_path = path + "/pca_mean.txt";
            pca_mean = readFileV(pca_mean_path.c_str(),NUM_V);

            string Ureduce_path = path + "/Ureduce.txt";
            Ureduce = new Matrix<>(readFileM(Ureduce_path.c_str(),NUM_V,pca_meta[0]));
            volt_reduce = new Vector<>(Zeros(pca_meta[0]));

        } else {
            b_pca = false;
        }

}
