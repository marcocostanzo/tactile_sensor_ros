#include "ros/ros.h"
#include "sun_tactile_common/TactileStamped.h"

int num_voltages;
double volt_thr;

ros::Publisher pubVolt;
std::vector<float> prev_volts;

int check_warn_max_times = 5;
int check_warn_count = 0;

//==========TOPICs CALLBKs=========//
void readV(const sun_tactile_common::TactileStamped::ConstPtr& msg)
{
  if (prev_volts.size() == 0)
  {
    num_voltages = (msg->tactile.rows * msg->tactile.cols);
    prev_volts.resize(num_voltages);
    prev_volts = msg->tactile.data;
    return;
  }

  if ((msg->tactile.rows * msg->tactile.cols) != num_voltages)
  {
    ROS_ERROR_STREAM("Invalid num_voltages: " << (msg->tactile.rows * msg->tactile.cols));
    return;
  }

  // Check
  for (int i = 0; i < num_voltages; i++)
  {
    if (fabs(msg->tactile.data[i] - prev_volts[i]) > volt_thr)
    {
      if(check_warn_count < check_warn_max_times)
      {
        check_warn_count++;
        return;
      }
      else
      {
        ROS_ERROR_STREAM("tactile_logic_filter: invalid voltage for " << check_warn_count << " times on voltage #" << i );
      }
    }
  }

  check_warn_count = 0;

  prev_volts = msg->tactile.data;

  pubVolt.publish(msg);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tactile_logic_filter");

  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh_public = ros::NodeHandle();

  /**** PARAMS ****/
  std::string str_in_topic = std::string("");
  nh_private.param("in_topic", str_in_topic, std::string("/tactile"));
  std::string str_out_topic = std::string("");
  nh_private.param("out_topic", str_out_topic, str_in_topic + std::string("/logic_filter"));
  nh_private.param("volt_thr", volt_thr, 0.1);
  /************************************/

  prev_volts.clear();

  pubVolt = nh_public.advertise<sun_tactile_common::TactileStamped>(str_out_topic, 1);

  ros::Subscriber subVoltages = nh_public.subscribe(str_in_topic, 1, readV);

  ros::spin();

  return 0;
}
