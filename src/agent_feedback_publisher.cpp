#include <decentralised_planning/path_planner.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "agent_feedback_publisher");
    if (argc != 2)
    {
        ROS_INFO_STREAM("usage: agent_feedback_publisher [agent_<number>]");
    }
	else
	{
		ros::NodeHandle nh;
		ros::Publisher pub = nh.advertise<geometry_msgs::Pose2D>("agent_feedback", 10);;
		ros::Rate loop_rate(10);

		geometry_msgs::Pose2D msg;
		string agent = string(argv[1]);

		// Agent_id generation
		std::map< int, Pair > agent_map;
		agent_map = init_agent_map(agent_map);

		Pair q = get_agent_info(agent, agent_map); // Output is in the vector co-ord
		
		int q1 = q.first; int q2 = q.second;
		con_vec_grid(&q1, &q2);
		msg.x = q1;
		msg.y = q2;
		msg.theta = 0;

		while (ros::ok())
		{
			ROS_INFO(" Position of the agent (x, y, yaw) :  (%ld, %ld, %ld)", (long int) msg.x, (long int) msg.y, (long int) msg.theta);
			pub.publish(msg);
			ros::spin();
			loop_rate.sleep();	
		}
	}
	return 0;
}



