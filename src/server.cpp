#include "ros/ros.h"
#include "std_msgs/String.h"
#include <mutex>
#include "interview_code/task_command.h"
#include "interview_code/agent_register.h"
#include "interview_code/agent_feedback.h"

std::list<int> task_list; // task list

// key : agent id; value : "AGENT_FREE", "AGENT_BUSY", "AGENT_NONE"
std::map<int, std::string> agent_state_map;

// key : agent id; value : agent node name
std::mutex agent_mutex;

// store agent count
int agent_count = 0;

void agent_feedback_callback(const interview_code::agent_feedback::ConstPtr& msg)
{
  agent_mutex.lock();
  int agent_id = msg->agent_id;

  if (agent_state_map.find(agent_id) != agent_state_map.end())
  {
    // update agent state
    agent_state_map.at(agent_id) = msg->agent_state;
  }
  else
  {
    ROS_ERROR("Agent should register first");
  }

  agent_mutex.unlock();
}

// agent will register to server first, server send back agent_id to agent node
bool agent_register_callback(interview_code::agent_register::Request &req,
                             interview_code::agent_register::Response &res)
{
  agent_mutex.lock();
  res.agent_id = agent_count + 1;
  agent_count++;
  agent_state_map[res.agent_id] = "AGENT_FREE";

  agent_mutex.unlock();
  return true;
}

int main(int argc, char **argv)
{
  std::cout << "server running" << std::endl;
  ros::init(argc, argv, "node_mini_factory_server");

  ros::NodeHandle server_handler;
  ros::Rate loop_rate(10);

  ROS_INFO("Server Running");
  ros::ServiceServer register_service = server_handler.advertiseService("/agent_register",
                                                                        agent_register_callback);

  ros::Subscriber feedback_sub = server_handler.subscribe("/agent_feedback",
                                                          10,
                                                          agent_feedback_callback);

  std::string agent_task_topic
    = "/agent_task";

  ros::Publisher agent_task_pub
    = server_handler.advertise<interview_code::task_command>(agent_task_topic, 1);

  // add task to task list
  for (int i = 1; i < 6; i++)
  {
    task_list.push_back(i);
  }

  while(ros::ok())
  {
    // scan call agent state, send task to agent if agent is free
    agent_mutex.lock();
    for (auto agent_iter = agent_state_map.begin();
         agent_iter != agent_state_map.end();
         agent_iter++)
    {
      /* add this delay to solve agent can't receive task bug,
       * ros service will send response to agent, then it can't receive task msg
       * if publisher publish msg to agent, delay a while for agent receive service response
       * */
      usleep(100000);

      // check agent is free or not
      if ("AGENT_FREE" == agent_iter->second)
      {
        // check task list is free or not
        if (!task_list.empty())
        {
          interview_code::task_command task;
          task.task_tag = task_list.front();
          task.agent_id = agent_iter->first;
          agent_iter->second = "AGENT_BUSY";

          agent_task_pub.publish(task);

          task_list.pop_front();
        }
      }
    }

    agent_mutex.unlock();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
