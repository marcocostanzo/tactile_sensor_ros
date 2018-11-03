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

#include <std_srvs/Empty.h>


//------new
#include <ANN/ANN.h>

using namespace std;
using namespace TooN;

//==========GLOBAL VARS========//
const int N_MEAN = 60; //Number of seamples used in the bias computation
const int NUM_V = 25; //Length of voltage Vector
Matrix<6,NUM_V> K; //Linear Calibration Matrix
Vector<NUM_V> voltages = Zeros, bias = Zeros, voltages_rect = Zeros;
Vector<6> wrench = Zeros;
bool voltageMessageArrived = false; //Used in the service RemoveBias
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
ros::Publisher pubVoltagesRect;
//----------------------------//
//**********MSGS*************//
geometry_msgs::WrenchStamped msgWrench;
sun_tactile_common::TactileStamped msgVoltageRect;
//====================================//


//=========LOCAL FCNs INTERFACEs=======//

//Model init, NB: Call it before Voltage-Subscriber definition and after Wrench-Publisher definition
void init_model( string path );
void init_ANN_model( string path );

//Calculate voltage bias
void _removeBias();

//====================================//


//==========SERVICES=========//

//Service to remove bias of voltages
bool removeBias(std_srvs::Empty::Request  &req, 
   		 		std_srvs::Empty::Response &res){
	_removeBias();
	return true;	
}

//====================================//


//==========TOPICs CALLBKs=========//
bool b_first_bias_calc = false;
void readV( const sun_tactile_common::TactileStamped::ConstPtr& msg  ){
    
	for(int i = 0; i<NUM_V; i++){
		voltages[i] = msg->tactile.data[i];
	}
    voltageMessageArrived = true;
    if(!b_first_bias_calc)
        return;

    voltages_rect = voltages - bias;
    
    //Calculate wrench model
    if(b_pca){
        Vector<NUM_V> votages_pca_mean = voltages_rect - pca_mean;
		//((*Ureduce).T())*votages_pca_mean;
        *volt_reduce = ((*Ureduce).T())*votages_pca_mean;
        wrench = myANN->compute( *volt_reduce );
    } else {
        wrench = myANN->compute( voltages_rect );
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

    msgVoltageRect.tactile.data.resize(25);
	msgVoltageRect.header.stamp = msg->header.stamp;
    msgVoltageRect.header.frame_id = msg->header.frame_id;
    msgVoltageRect.tactile.rows = msg->tactile.rows;
    msgVoltageRect.tactile.cols = msg->tactile.cols;
    msgVoltageRect.tactile.info = msg->tactile.info;

    for(int i=0; i<NUM_V ; i++)
        msgVoltageRect.tactile.data[i] = voltages_rect[i];

    //Publish
    pubWrench.publish( msgWrench );
	pubVoltagesRect.publish( msgVoltageRect );  

}

//====================================//


//==============MAIN================//

int main(int argc, char *argv[]){

	ros::init(argc,argv,"read_wrench");

	ros::NodeHandle nh_private = ros::NodeHandle("~");
    nh_public = new ros::NodeHandle();

    
    /**** CHECK PARAMS ****/
    string path("");
    path = ros::package::getPath("wsg_50_driver_sun");
    path = path + "/Finger_files/";
    nh_private.param("fingerCode" , fingerCode, string("") );

    nh_private.param("in_voltage_topic" , in_voltage_topic_str, string("tactile_voltage/raw") );
    string voltage_rect_topic_str;
    nh_private.param("voltage_rect_topic" , voltage_rect_topic_str, string("tactile_voltage/rect") );
    string wrench_topic_str;
    nh_private.param("wrench_topic" , wrench_topic_str, string("wrench") );
    string service_remove_bias_str;
    nh_private.param("service_remove_bias" , service_remove_bias_str, string("removeBias") );

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

    /******INIT ROS MSGS**********/
	msgVoltageRect.tactile.data.resize(NUM_V);
	/********************/

    /*******INIT ROS PUB**********/
    //Force pub
	pubWrench = nh_public->advertise<geometry_msgs::WrenchStamped>( wrench_topic_str ,1);
	//Voltage_rect pub
	pubVoltagesRect = nh_public->advertise<sun_tactile_common::TactileStamped>( voltage_rect_topic_str ,1);
    /***************************/

    /*******INIT MODEL**********/
    init_model(path); //Don't move from here!
    /***************************/
	

    /*******INIT ROS SUB**********/
	//Status subscriber
	ros::Subscriber subVoltage = nh_public->subscribe( in_voltage_topic_str, 1, readV);
	//Remove bias subscriber
	ros::ServiceServer serviceRemoveBias = nh_public->advertiseService( service_remove_bias_str, removeBias);
    /***************************/



/*============LOOP==============*/
    ros::spin();
/*==============================*/

	
	return 0;
}



/*===========LOCAL FCNs IMPL==========*/

//Calculate bias
void _removeBias(){

    cout << BOLDYELLOW << "REMOVING BIAS... ";

    Matrix<N_MEAN,NUM_V> lastVReads = Zeros; //buffer

    //redefinition of subscriber cause you can't use previouse subscriber in a service
    ros::Subscriber subVoltage = nh_public->subscribe( in_voltage_topic_str, 1, readV);

    //*****Fill lastVReads Matrix****//
    int _index = 0; //I don't use first 10 samples
    for(int i=0; i < N_MEAN + 10 ; i++){

        //Wait for Voltages
        while(!voltageMessageArrived){
		    ros::spinOnce();
            if(!voltageMessageArrived)
                sleep(0.01); //10ms
        }
        voltageMessageArrived = false;

        //Fill lastVReads Row
        lastVReads[_index] = voltages;

        //Update column index
	    _index = (_index+1)%N_MEAN;

	}
    //**************************************//

    //*****Calculate bias****//
	bias = Zeros;
	for(int i = 0; i<N_MEAN ; i++)
        bias = bias + lastVReads[i];

    bias = bias/N_MEAN;
    //**************************************//

    cout << "DONE!" << endl;
	
    cout << "-----------------" << endl << "NEW VOLTAGE BIAS =" << endl << bias << endl << "-----------------" << CRESET << endl;
    b_first_bias_calc = true;
}

void init_model(string path){

    init_ANN_model( path );
	cout << BOLDBLUE << "ANN Model" << CRESET << endl;
	//myANN->display(); //debug line
    
    //*****Remove initial bias****//
    _removeBias();
    //**************************************//

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
