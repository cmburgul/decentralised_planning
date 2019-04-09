#include <decentralised_planning/path_planner.h>
#include "decentralised_planning/GetPath.h"

bool service_callback(decentralised_planning::GetPath::Request &req, 
					  decentralised_planning::GetPath::Response &res)
{	
	string agent = req.agent_id;
	int i2 = req.goal_pose.x;
	int j2 = req.goal_pose.y;
	
	// Agent_id generation
	std::map< int, Pair > agent_map;
	agent_map = init_agent_map(agent_map);

	Pair q = get_agent_info(agent, agent_map); // Output is in the vector co-ord
	
	int q1 = q.first; int q2 = q.second;
	con_vec_grid(&q1, &q2);

	// Roadmap generation
	vector< vector<node> > grid; // Initialize the grid object
	grid = initialise_grid(grid); // Initilize the grid

	vector< Pair > final_path;
	final_path = a_star_path_finder(q1, q2, i2, j2, grid);
	
	// Code for putting data from final_path vector to res.path
	for (auto i = final_path.begin(); i != final_path.end(); i++)
	{
		Pair p = *i;
		geometry_msgs::Pose2D data;
		data.x = p.first;
		data.y = p.second;
		data.theta = 0;
		res.path.push_back(data);
		//cout << "(" << p.first << ", " << p.second << ")-->";
	}
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "get_plan_server");
	
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("get_plan", service_callback);
	ros::Rate loop_rate(10);
	
	ROS_INFO("Ready to find the optimal Path");

	ros::spin();

	return 0;
}


