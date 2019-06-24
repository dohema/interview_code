#include "ros/ros.h"
#include "std_msgs/String.h"

std::list<int> task_list; // task list

// key : agent id; value : free(true) or busy(false)
std::map<int, bool> agent_status_map;

// key : agent id; value : agent node name
std::map<int, std::string> agent_node_name_map;

void agent_feedback_callback(const )
{

}

void agent_register_callback(const )
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_mini_factory_server");

  ros::NodeHandle server_handler;

  ros::Subscriber feedback_sub = server_handler.subscribe("/agent_feedback",
                                                          10,
                                                          agent_feedback_callback);

  ros::Subscriber register_sub = server_handler.subscribe("/agent_register",
                                                          10,
                                                          agent_register_callback);

  while(true)
  {
    for (auto agent_iter = agent_status_map.cbegin();
         agent_iter != agent_status_map.cend();
         agent_iter++)
    {
      if (agent_iter->second == "Free")
      {
        if (!task_list.empty())
        {
          std::string agent_task_topic
            = agent_node_name_map.at(agent_iter->first) + "_task";

          ros::Publisher temp_task_pub = server_handler.advertise(agent_task_topic, 10);

          temp_task_pub.publish();
        }
      }

    }

  }

}
