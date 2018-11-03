#include "ros/ros.h"

#include "sun_tactile_common/TactileStamped.h"
#include "TF_MIMO/TF_MIMO_DIAGONAL.h"
#include "TF_SISO/TF_FIRST_ORDER_FILTER.h"


using namespace TooN;
using namespace std;

/* ======= COLORS ========= */
#define CRESET   "\033[0m"
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
/*===============================*/

#define HEADERCOLOR BOLDYELLOW
#define WARNCOLOR   BOLDYELLOW

#define HEADER_PRINT HEADERCOLOR "[Filter Tactile]: " CRESET 

ros::Publisher pubVoltagesFilter;
sun_tactile_common::TactileStamped msgVoltageFilter;

int num_voltages;
double* voltages_ptr;

//==========TOPICs CALLBKs=========//
void readV( const sun_tactile_common::TactileStamped::ConstPtr& msg  ){

    if( (msg->tactile.rows * msg->tactile.cols) != num_voltages ){
        cout << HEADER_PRINT << WARNCOLOR << "num_voltages != rows*cols" << CRESET << endl;
    }

	for(int ii = 0; ii<num_voltages; ii++){
        voltages_ptr[ii] = msg->tactile.data[ii];
	}

	msgVoltageFilter.header.stamp = msg->header.stamp;
    msgVoltageFilter.header.frame_id = msg->header.frame_id;
    msgVoltageFilter.tactile.info = msg->tactile.info;
    msgVoltageFilter.tactile.rows = msg->tactile.rows;
    msgVoltageFilter.tactile.cols = msg->tactile.cols;
	
}

//====================================//


int main(int argc, char *argv[])
{

    ros::init(argc,argv,"filter_voltages");

    ros::NodeHandle nh("~");

    /**** PARAMS ****/
    nh.param("num_voltages" , num_voltages, 25 );
    string str_in_topic = string("");
    nh.param("in_topic" , str_in_topic, string("/tactile") );
    string str_out_topic = string("");
    nh.param("out_topic" , str_out_topic, str_in_topic + string("/filter") );
    double cut_freq;
    nh.param("cut_freq" , cut_freq, 20.0 );
    double Hz;
    nh.param("rate" , Hz, 500.0 );
	/************************************/

    /******INIT ROS MSGS**********/
	msgVoltageFilter.tactile.data.resize(num_voltages);
    Vector<> voltages_filter = Zeros(num_voltages);
    double voltages[num_voltages];
    for(int ii = 0; ii < num_voltages; ii++) voltages[ii] = 0.0;
    voltages_ptr = voltages;
	/********************/

    /*******INIT ROS PUB**********/
	//Voltage_filter pub
	pubVoltagesFilter = nh.advertise<sun_tactile_common::TactileStamped>( str_out_topic, 1);
    /***************************/

    /*******INIT ROS SUB**********/
	//Status subscriber
	ros::Subscriber subVoltages = nh.subscribe( str_in_topic , 1, readV);
    /***************************/

    /******INIT FILTER************/
    TF_MIMO_DIAGONAL filter(    num_voltages,
                                TF_FIRST_ORDER_FILTER(cut_freq, 1.0/Hz), 
                                1.0/Hz
                            );
    /***************************/	

    /*============LOOP==============*/
    ros::Rate loop_rate(Hz);
	while(ros::ok()){

        Vector<> in_voltages = wrapVector( voltages , num_voltages );

        voltages_filter = filter.apply( in_voltages );
	
        //Fill msg
		for(int ii = 0; ii<num_voltages; ii++){
            msgVoltageFilter.tactile.data[ii] = voltages_filter[ii];
		}

		pubVoltagesFilter.publish( msgVoltageFilter );
		loop_rate.sleep();
      	ros::spinOnce();
			
	}
    
    /*==============================*/

    return 0;
}