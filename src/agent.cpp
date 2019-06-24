#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ctime>

std::atomic<bool> new_task;
std::time_t receive_task_timepoint;

void agent_task_callback(const )
{

}

int main(int argc, char **argv)
{
  std::time_t seconds;
  std::time (&seconds);
  std::stringstream time_stream;
  time_stream << seconds;
  std::string agent_node_name = "simple_agent" + time_stream.str ();
  std::string agent_task_topic = agent_node_name + "_task";

  new_task.store(false);

  ros::init(argc, argv, agent_node_name);

  ros::NodeHandle agent_handler;

  ros::Publisher agent_feedback_pub = agent_handler.advertise("/agent_feedback", 10);
  ros::Publisher agent_register_pub = agent_handler.advertise("/agent_register", 10);

  ros::Subscriber agent_task_sub = agent_handler.subscribe(agent_task_topic,
                                                          10,
                                                          agent_task_callback);

  agent_register_pub.publish();

  while(true)
  {
    if (new_task.load())
    {

    }
    agent_feedbakc_pub.publish();
  }
}
