#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_srvs/Trigger.h"

using namespace std;

ros::Publisher pubWrench;
std::string frame_id_str;

double y_base = 0.0;
geometry_msgs::WrenchStamped in_msg;

// bool update_fy_callbk(std_srvs::Trigger::Request  &req, 
//    		 		std_srvs::Trigger::Response &res)
// {
//     int N_MEAN = 100;
//     double y_base_tmp = 0;
//     for(int i=0; i<N_MEAN; i++)
//     {
//         ros::Duration(0.02).sleep();
//         ros::spinOnce();
//         y_base_tmp += in_msg.wrench.force.y;
//     }
//     y_base = y_base_tmp/N_MEAN;
//     ROS_WARN_STREAM("New fy = " << y_base);
//     res.success = true;

//     return true;
// }

char mod = 'a';
bool to_mod_a_callbk(std_srvs::Trigger::Request  &req, 
   		 		std_srvs::Trigger::Response &res)
{
    mod = 'a';
    ROS_WARN_STREAM("FY MOD A");
    res.success = true;

    return true;
}

bool to_mod_b_callbk(std_srvs::Trigger::Request  &req, 
   		 		std_srvs::Trigger::Response &res)
{
    int N_MEAN = 100;
    double y_base_tmp = 0;
    for(int i=0; i<N_MEAN; i++)
    {
        ros::Duration(0.02).sleep();
        ros::spinOnce();
        y_base_tmp += in_msg.wrench.force.y;
    }
    y_base = y_base_tmp/N_MEAN;
    mod = 'b';
    ROS_WARN_STREAM("MOD B: New fy = " << y_base);
    res.success = true;

    return true;
}



void readWrench( const geometry_msgs::WrenchStamped::ConstPtr& msg )
{

    geometry_msgs::WrenchStamped out_msg = *msg;
    in_msg = *msg;

    out_msg.wrench.force.x = -msg->wrench.force.x;
    out_msg.wrench.torque.x = -msg->wrench.torque.x;
    out_msg.wrench.torque.z = -msg->wrench.torque.z;

    // out_msg.wrench.force.y = msg->wrench.force.y - 2.0*y_base;

    // ROS_WARN_STREAM_THROTTLE(1.0, "in_y:" << msg->wrench.force.y << "\nbase:" << y_base << "\nout:" << out_msg.wrench.force.y);

    //mod1
    if(mod == 'a')
        out_msg.wrench.force.y = -msg->wrench.force.y;

    //mod2
    if(mod == 'b')
        out_msg.wrench.force.y = msg->wrench.force.y - 2.0*y_base;

    out_msg.header.frame_id = frame_id_str;

    pubWrench.publish(out_msg);

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "mirror_wrench");
    
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    string topic_in_str("");
    nh_private.param("in_wrench" , topic_in_str, string("/left/wrench") );
    string topic_out_str("");
    nh_private.param("out_wrench" , topic_out_str, string("/right/wrench") );

    std::string tf_prefix_str;
    nh_private.param("tf_prefix" , tf_prefix_str, string("") );
    nh_private.param("frame_id" , frame_id_str, string("") );
    frame_id_str = tf_prefix_str + frame_id_str;

    ros::Subscriber subWrench = nh_public.subscribe( topic_in_str ,1, readWrench);

    pubWrench = nh_public.advertise<geometry_msgs::WrenchStamped>( topic_out_str, 1);

    // ros::ServiceServer update_fy_serv = nh_private.advertiseService("update_fy", update_fy_callbk);
    ros::ServiceServer to_mod_a_serv = nh_private.advertiseService("to_mod_a", to_mod_a_callbk);
    ros::ServiceServer to_mod_b_serv = nh_private.advertiseService("to_mod_b", to_mod_b_callbk);


    ros::spin();

    return 0;
}