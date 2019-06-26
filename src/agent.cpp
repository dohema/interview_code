#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ctime>
#include <mutex>
#include "interview_code/task_command.h"
#include "interview_code/agent_register.h"
#include "interview_code/agent_feedback.h"

// agent state : "AGENT_FREE", "AGENT_BUSY", "AGENT_NONE"
std::string agent_state = "AGENT_FREE";
// store the timepoint when receive a task
std::time_t receive_task_timepoint;
// agent_id will be set by server response
//
std::mutex agent_mutex;
int agent_id = 0;
int task_tag = 0;

void agent_task_callback(const interview_code::task_command::ConstPtr& msg)
{
  agent_mutex.lock();
  if (msg->agent_id == agent_id)
  {
    if ("AGENT_BUSY" == agent_state)
    {
      ROS_ERROR("Can't send task to agent while agent is busy");
    }
    else
    {
      agent_state = "AGENT_BUSY";
      receive_task_timepoint = std::time(0); // set current time
      task_tag = msg->task_tag;
      ROS_INFO_STREAM("Agent : " << agent_id << " receive Task : " << msg->task_tag);
    }
  }
  agent_mutex.unlock();
}

int main(int argc, char **argv)
{
  if (argc > 2)
  {
    std::string delay_option = argv[1];
    if ("-t" == delay_option)
    {
      int delay_count = std::stoi(argv[2]);
      sleep(delay_count);
    }
  }

  std::time_t seconds = std::time(nullptr);
  std::time (&seconds);

  std::stringstream time_stream;
  time_stream << seconds;
  std::string agent_node_name = "simple_agent" + time_stream.str ();

  ros::init(argc, argv, agent_node_name);

  ros::NodeHandle agent_handler;

  ros::Rate loop_rate(10);

  ros::Subscriber agent_task_sub = agent_handler.subscribe("/agent_task",
                                                          1,
                                                          agent_task_callback);

  ros::Publisher agent_feedback_pub
    = agent_handler.advertise<interview_code::agent_feedback>("/agent_feedback", 1);

  ros::ServiceClient agent_register_client
    = agent_handler.serviceClient<interview_code::agent_register>("/agent_register");

  interview_code::agent_register register_msg;
  interview_code::agent_feedback agent_feedback;

  register_msg.request.node_name = agent_node_name;
  agent_register_client.call(register_msg);
  agent_id = register_msg.response.agent_id;

  ros::spinOnce();
  while(ros::ok())
  {
    agent_mutex.lock();
    // check agent has finished task or not
    if ("AGENT_BUSY" == agent_state && (std::difftime(std::time(0), receive_task_timepoint) > 4))
    {
      ROS_INFO_STREAM("Agent" << agent_id << " finished task : " << task_tag);
      agent_state = "AGENT_FREE";

      agent_feedback.agent_id = agent_id;
      agent_feedback.agent_state = agent_state;
      agent_feedback_pub.publish(agent_feedback);
    }
    agent_mutex.unlock();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
