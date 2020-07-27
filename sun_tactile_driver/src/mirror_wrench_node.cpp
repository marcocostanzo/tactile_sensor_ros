#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"

using namespace std;

ros::Publisher pubWrench;
std::string frame_id_str;

void readWrench( const geometry_msgs::WrenchStamped::ConstPtr& msg )
{

    geometry_msgs::WrenchStamped out_msg = *msg;

    out_msg.wrench.force.x = -msg->wrench.force.x;
    out_msg.wrench.torque.x = -msg->wrench.torque.x;
    out_msg.wrench.torque.z = -msg->wrench.torque.z;

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

    ros::spin();

    return 0;
}