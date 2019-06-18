#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ctime>

void agent_feedback_callback(const )
{

}



int main(int argc, char **argv)
{
  std::time_t seconds;
  std::time (&seconds);
  std::stringstream time_stream;
  time_stream << seconds;
  std::string agent_node_name = "simple_agent" + time_stream.str ();

  ros::init(argc, argv, agent_node_name);

  ros::NodeHandle agent_handler;

  ros::Publisher agent_pub = agent_handler.advertise("/agent_feedback", 10);

  ros::

}
