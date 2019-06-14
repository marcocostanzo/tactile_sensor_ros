/*
    ROS node to evaluate real rate of sensors (e.g. wifi-version)

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

using namespace std;

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 

ros::Duration last_delta_T;
sun_tactile_common::TactileStamped prev_voltages;
int NUM_V;
#define WIN_SIZE 400
vector<double> win;
int win_iter = 0;

void readV( const sun_tactile_common::TactileStamped::Ptr& msg  )
{

    //check equal
    for( int i=0; i<NUM_V; i++ )
    {
        if(msg->tactile.data[i]!=prev_voltages.tactile.data[i])
        {
            break;
        }
        if(i==(NUM_V-1))
        {
            return;
        }
    }

    last_delta_T = msg->header.stamp - prev_voltages.header.stamp;

    win[win_iter] = last_delta_T.toSec();
    win_iter = (win_iter+1)%WIN_SIZE;

    prev_voltages = *msg;

}


int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "tactile_eval_rate");
    
    ros::NodeHandle nh_public;
    ros::NodeHandle nh_private("~");

    /* PARAMS */
    //Input topic
    string in_voltage_topic_str;
    nh_private.param("in_voltage_topic" , in_voltage_topic_str, string("tactile_voltage/raw") );
    //Voltages count
    nh_private.param("num_voltages" , NUM_V, 25 );

    //Rate
    double hz,cut_freq;
    nh_private.param("rate" , hz, 1000.0 );
    nh_private.param("cut_freq" , cut_freq, 1.5 );

    prev_voltages.tactile.data.resize(NUM_V);
    win.resize(WIN_SIZE);

    double wc = 2.0*M_PI*cut_freq;
    double T = 1.0/hz;
    double alpha = (wc*T)/(1+wc*T);
    
    //Subscribers
    ros::Subscriber subVoltage = nh_public.subscribe( in_voltage_topic_str, 1, readV);

    ros::Rate loop_rate(hz);
    string filter_color_str, color_str, max_color_str, min_color_str;
    double delta_t_filtered = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();

        //Find max/min
        double max=-INFINITY, min=INFINITY;
        for( int i=0; i<WIN_SIZE; i++ )
        {
            if(win[i]>max) max = win[i];
            if(win[i]<min) min = win[i];
        }

        delta_t_filtered = alpha*last_delta_T.toSec() +(1-alpha)*delta_t_filtered;

        if(delta_t_filtered > 0.2) filter_color_str = BOLDRED;
        if(delta_t_filtered > 0.1) filter_color_str = RED;
        else if(delta_t_filtered > 0.05) filter_color_str = BOLDYELLOW;
        else if(delta_t_filtered > 0.02) filter_color_str = YELLOW;
        else filter_color_str = GREEN;

        if(last_delta_T.toSec() > 0.2) color_str = BOLDRED;
        if(last_delta_T.toSec() > 0.1) color_str = RED;
        else if(last_delta_T.toSec() > 0.05) color_str = BOLDYELLOW;
        else if(last_delta_T.toSec() > 0.02) color_str = YELLOW;
        else color_str = GREEN;

        if(max > 0.2) max_color_str = BOLDRED;
        if(max > 0.1) max_color_str = RED;
        else if(max > 0.05) max_color_str = BOLDYELLOW;
        else if(max > 0.02) max_color_str = YELLOW;
        else max_color_str = GREEN;

        if(min > 0.05) min_color_str = BOLDRED;
        if(min > 0.01) min_color_str = RED;
        else if(min > 0.005) min_color_str = BOLDYELLOW;
        else if(min > 0.003) min_color_str = YELLOW;
        else min_color_str = GREEN;

        printf("\033c");
        cout << in_voltage_topic_str << endl <<
                "Last deltaT = " << color_str << last_delta_T << CRESET << endl <<
                "delta_t = " << filter_color_str << delta_t_filtered << CRESET << endl <<
                "MAX = " << max_color_str << max << CRESET << endl <<
                "MIN = " << min_color_str << min << CRESET << endl;

        loop_rate.sleep();
    }
    


    return 0;
}