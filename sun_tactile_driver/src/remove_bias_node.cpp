/*
    ROS node to remove bias from voltages

    Copyright 2019 Università della Campania Luigi Vanvitelli

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
#include <actionlib/server/simple_action_server.h>
#include <sun_tactile_common/ComputeBiasAction.h>
#include <sun_tactile_common/TactileStamped.h>

#ifndef SUN_COLORS
#define SUN_COLORS

/* ======= COLORS ========= */
#define CRESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLD    "\033[1m"       /* Bold */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
/*===============================*/

#endif

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 

//constants
#define NUM_SAMPLES_NOT_USE 10
#define MAX_WAIT_COUNT 100
#define TIME_TO_SLEEP_WAITING_SAMPLE 0.01

using namespace std;

/*GLOBAL ROS VARS*/
ros::Publisher pubV;
actionlib::SimpleActionServer<sun_tactile_common::ComputeBiasAction>* compute_bias_as;
string in_voltage_topic_str;

/*GLOBAL VARS*/
vector<float> bias;
vector<float> raw_valtages;
bool b_msg_arrived = false;
bool b_can_pub = false;
int default_num_samples_to_use;
int NUM_V;


/*
    Read voltages, remove bias and republish it
*/
void readV( const sun_tactile_common::TactileStamped::Ptr& msg  ){

    raw_valtages = msg->tactile.data;
    for(int i = 0; i<NUM_V; i++){
	    msg->tactile.data[i] = raw_valtages[i] - bias[i];
	}

    b_msg_arrived = true;

    if(b_can_pub) //This is to avoid publication with zero bias
        pubV.publish(msg);

}

/*
    Compute bias routine
*/
bool computeBias(int num_samples_to_use, bool b_from_as = false){

    cout << HEADER_PRINT BOLDYELLOW "Computing BIAS... " CRESET << endl;

    //redefinition of subscriber because you can't use previouse subscriber in a action
    ros::NodeHandle nh_public;
    ros::Subscriber subVoltage = nh_public.subscribe( in_voltage_topic_str, 1, readV);

    //Don't use first N samples    
    for( int i=0; i<NUM_SAMPLES_NOT_USE; i++ ){
        //Wait sample
        int wait_count = 0;
        b_msg_arrived = false;
        while(!b_msg_arrived){
            if( wait_count > MAX_WAIT_COUNT ){
                return false;
            }
            wait_count++;
            sleep(TIME_TO_SLEEP_WAITING_SAMPLE);
            ros::spinOnce();
        }
    }

    //Initializate bias to zero
    vector<double> bias_;
    for( int i=0; i<NUM_V; i++ )
        bias.push_back(0.0);

    //Collect Samples for bias computation
    for( int i=0; i<num_samples_to_use; i++ ){
        //Wait sample
        int wait_count = 0;
        b_msg_arrived = false;
        while(!b_msg_arrived){
            if( wait_count > MAX_WAIT_COUNT ){
                return false;
            }
            wait_count++;
            sleep(TIME_TO_SLEEP_WAITING_SAMPLE);
            ros::spinOnce();
        }
        for( int i=0; i<NUM_V; i++ )
            bias_[i] += raw_valtages[i];
        if(b_from_as){
            sun_tactile_common::ComputeBiasFeedback msg_feedbk;
            msg_feedbk.samples_left = i-num_samples_to_use;
            compute_bias_as->publishFeedback(msg_feedbk);
        }
    }

    //Compute bias
    for( int i=0; i<NUM_V; i++ )
        bias[i] = bias_[i]/num_samples_to_use;

    cout << HEADER_PRINT BOLDYELLOW "Computing BIAS..." GREEN " DONE!" << endl;

    return true;

}

/*
    Compute Bias Action CB
*/
void executeComputeBiasCB( const sun_tactile_common::ComputeBiasGoalConstPtr &goal ){
    
    sun_tactile_common::ComputeBiasResult remove_bias_result;

    if(goal->num_samples > 0){
        remove_bias_result.success = computeBias(goal->num_samples,true);
    } else {
        remove_bias_result.success = computeBias(default_num_samples_to_use,true);
    }

    if(remove_bias_result.success){
        remove_bias_result.msg = "DONE";
        remove_bias_result.bias.resize(NUM_V);
        remove_bias_result.bias = bias;
        compute_bias_as->setSucceeded(remove_bias_result);
    } else {
        remove_bias_result.msg = "ERROR - IS FINGERTIP ONLINE?";
        compute_bias_as->setAborted(remove_bias_result);
    }

}

void waitForVoltage(){

    ros::NodeHandle nh_public;
    ros::Subscriber subVoltage = nh_public.subscribe( in_voltage_topic_str, 1, readV);

    b_msg_arrived = false;
    while(!b_msg_arrived){
        sleep(TIME_TO_SLEEP_WAITING_SAMPLE);
        ros::spinOnce();
    }

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "remove_bias");

    ros::NodeHandle nh_public;
    ros::NodeHandle nh_private("~");

    /* PARAMS */
    //Input topic
    nh_private.param("in_voltage_topic" , in_voltage_topic_str, string("tactile_voltage/raw") );
    //Output Topic
    string out_voltage_topic_str;
    nh_private.param("out_voltage_topic" , out_voltage_topic_str, string("tactile_voltage/rect") );
    //Action compute bias
    string action_compute_bias_str;
    nh_private.param("action_compute_bias" , action_compute_bias_str, string("tactile_voltage/action_compute_bias") );
    //Default num samples to use in bias calculation
    nh_private.param("default_num_samples" , default_num_samples_to_use, 50 );
    //Voltages count
    nh_private.param("num_voltages" , NUM_V, 25 );

    //resize vectors
    bias.resize(NUM_V);
    raw_valtages.resize(NUM_V);

    cout << HEADER_PRINT YELLOW "Waiting for voltages..." CRESET << endl;
    waitForVoltage();

    //FIRST COMPUTATION OF BIAS
    {
        b_can_pub = false;
        bool b_result = false;
        while(ros::ok() && !b_result)
            b_result = computeBias(default_num_samples_to_use);
        if(!b_result){
            cout << HEADER_PRINT BOLDRED "Something goes wrong..." CRESET << endl;
            return -1;
        } 
        b_can_pub = true;
    }

    //Subscribers
    ros::Subscriber subVoltage = nh_public.subscribe( in_voltage_topic_str, 1, readV);

    //Publishers
    pubV = nh_public.advertise<sun_tactile_common::TactileStamped>( out_voltage_topic_str ,1);

    //Action Servers
    compute_bias_as = new actionlib::SimpleActionServer<sun_tactile_common::ComputeBiasAction>(
                    nh_public, 
                    action_compute_bias_str,
                    executeComputeBiasCB, 
                    false);

    compute_bias_as->start();

    ros::spin();

    delete compute_bias_as;

    return 0;
}