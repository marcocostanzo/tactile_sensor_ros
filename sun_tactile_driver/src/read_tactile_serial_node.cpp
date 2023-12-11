/*
    ROS node to read finger's voltages from serial port

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


#include "serial/serial.h"
#include <ros/ros.h>

#include "sun_tactile_common/TactileStamped.h"

#define ERROR_COLOR     "\033[1m\033[31m"      /* Bold Red */
#define WARN_COLOR      "\033[1m\033[33m"      /* Bold Yellow */
#define SUCCESS_COLOR   "\033[1m\033[32m"      /* Bold Green */
#define CRESET          "\033[0m"

#define CHAR_TO_SEND    'a'


using namespace std;

void set_serial_low_latency(const string& serial_port)
{
  cout << "Setting low_latency for " << WARN_COLOR << serial_port << CRESET << endl;
  string command = "setserial " + serial_port + " low_latency";
  int result = system(command.c_str());
  cout << "Setting low_latency for " << WARN_COLOR << serial_port << CRESET << " result:" << WARN_COLOR << result << CRESET << endl;
}


//==============MAIN================//

int main(int argc, char *argv[]){

    ros::init(argc,argv,"read_tactile_serial");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    /**** CHECK PARAMS ****/
    string serial_port = string("");
    nh_private.param("serial_port" , serial_port, string("/dev/ttyUSB0") );
    unsigned long baud = 0;
    string str_baud = string("");
    nh_private.param("baud_rate" , str_baud, string("500000") );
    sscanf(str_baud.c_str(), "%lu", &baud);
    int serialTimeout = 0;
    nh_private.param("serial_timeout" , serialTimeout, 1000 );
    int num_rows;
    nh_private.param("rows" , num_rows, 5 );
    int num_cols;
    nh_private.param("cols" , num_cols, 5 );
    string frame_id = string("");
    nh_private.param("frame_id" , frame_id, string("fingertip0") );
    string tf_prefix = string("");
    nh_private.param("tf_prefix" , tf_prefix, string("") );
    string topic_name = string("");
    nh_private.param("output_topic" , topic_name, string("/tactile") );

    /*** INIT SERIAL ****/	
    set_serial_low_latency(serial_port);
    serial::Serial my_serial(serial_port, baud, serial::Timeout::simpleTimeout(serialTimeout));

   /*** CHECK ***/
   if(!my_serial.isOpen()){
   	cout << ERROR_COLOR << "ERROR - SERIAL PORT " << WARN_COLOR << serial_port << ERROR_COLOR << " is not open!" << CRESET <<endl;
   	exit(-1);
   }
   cout << SUCCESS_COLOR << "SERIAL PORT " << WARN_COLOR << serial_port << SUCCESS_COLOR << " OPEN - OK" << CRESET << endl;
   
   // ==== Tactile msg ====
   double voltages_count = num_rows*num_cols;
   sun_tactile_common::TactileStamped finger_voltages;
   finger_voltages.tactile.data.resize(voltages_count);
   finger_voltages.header.frame_id = tf_prefix+frame_id;
   finger_voltages.tactile.rows = num_rows;
   finger_voltages.tactile.cols = num_cols;
   finger_voltages.tactile.info = "voltages";
   
   // ======= PUBLISHER
   ros::Publisher pubTactile = nh_public.advertise<sun_tactile_common::TactileStamped>( topic_name ,1);
   
   //init buffers
   const int dim_buffer = voltages_count*2;
   uint8_t b2write[1], readBytes[dim_buffer];
   //size_t bytes_wrote;
   b2write[0] = CHAR_TO_SEND;
 	
   //**** ROS MAIN LOOP  ***//
   while(ros::ok()){
   	
      /*	bytes_wrote = */ my_serial.write(b2write,1);

		my_serial.read(readBytes, dim_buffer);
			
		finger_voltages.header.stamp = ros::Time::now();
		
		for (int i = 0; i < voltages_count; i++) {

            finger_voltages.tactile.data[i] = (double)(readBytes[i*2] + (readBytes[i*2+1]&0b00001111)*256) * 3.3/4096.0;

		}
		
		pubTactile.publish(finger_voltages);	
        
   }

}

