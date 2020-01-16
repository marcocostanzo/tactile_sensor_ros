#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"

using namespace std;

ros::Publisher pubWrench;

void readWrench( const geometry_msgs::WrenchStamped::ConstPtr& msg )
{

    geometry_msgs::WrenchStamped out_msg = *msg;

    out_msg.wrench.force.x = -msg->wrench.force.x;
    out_msg.wrench.torque.z = -msg->wrench.torque.z;

    pubWrench.publish(out_msg);

}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "rotate_wrench");
    
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public;

    string topic_in_str("");
    nh_private.param("in_topic" , topic_in_str, string("wrench0") );
    string topic_out_str("");
    nh_private.param("out_topic" , topic_out_str, string("wrench1") );

    ros::Subscriber subWrench = nh_public.subscribe( topic_in_str ,1, readWrench);

    pubWrench = nh_public.advertise<geometry_msgs::WrenchStamped>( topic_out_str, 1);

    ros::spin();

    return 0;
}