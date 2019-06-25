#include "ros/ros.h"
#include "std_msgs/String.h"
#include "interview_code/task_command.h"
#include "interview_code/agent_register.h"
#include "interview_code/agent_feedback.h"

std::list<int> task_list; // task list

// key : agent id; value : "AGENT_FREE", "AGENT_BUSY", "AGENT_NONE"
std::map<int, std::string> agent_state_map;

// key : agent id; value : agent node name
std::map<int, std::string> agent_node_name_map;

// store agent count
int agent_count = 1;

void agent_feedback_callback(const interview_code::agent_feedback::ConstPtr& msg)
{
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
}

// agent will register to server first, server send back agent_id to agent node
bool agent_register_callback(interview_code::agent_register::Request &req,
                             interview_code::agent_register::Response &res)
{
  res.agent_id = agent_count++;
  agent_node_name_map[res.agent_id] = req.node_name;
  agent_state_map[res.agent_id] = "AGENT_FREE";

  ROS_INFO_STREAM("Agent : " << res.agent_id << " register to server name : " << req.node_name);
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


  // add task to task list
  for (int i = 1; i < 6; i++)
  {
    task_list.push_back(i);
  }

  while(ros::ok())
  {
    // scan call agent state, send task to agent if agent is free
    for (auto agent_iter = agent_state_map.begin();
         agent_iter != agent_state_map.end();
         agent_iter++)
    {
      // check agent is free or not
      if ("AGENT_FREE" == agent_iter->second)
      {
        // check task list is free or not
        if (!task_list.empty())
        {
          // use publish method send task command to agent, use topic : agent_node_name + "_task"
          std::string agent_task_topic
            = "/" + agent_node_name_map.at(agent_iter->first) + "_task";

          ros::Publisher temp_task_pub
            = server_handler.advertise<interview_code::task_command>(agent_task_topic, 10);

          interview_code::task_command task;
          // send the first task in task list to agent
          task.task_tag = task_list.front();
          agent_iter->second = "AGENT_BUSY";
          temp_task_pub.publish(task);

          ROS_INFO_STREAM("Server send Task : " << task.task_tag << " to agent : " << agent_iter->first);
          // delete the first task in task list
          task_list.pop_front();

        }
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
