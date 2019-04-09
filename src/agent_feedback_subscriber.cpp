#include <decentralised_planning/path_planner.h>

void Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  cout << "The agent's current position : " << "\n" << "x : " << msg->x << "\n" << "y : " << msg->y << "\n" << "theta : " << msg->theta << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agent_feedback_subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("agent_feedback", 10, Callback);
  ros::spin();

  return 0;
}