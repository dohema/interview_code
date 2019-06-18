#include "ros/ros.h"
#include "std_msgs/String.h"

void agent_feedback_callback(const )
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_mini_factory_server");

  ros::NodeHandle server_handler;

  ros::Subscriber sub = server_handler.subscribe("/agent_feedback",
                                                 10,
                                                 agent_feedback_callback);

}
