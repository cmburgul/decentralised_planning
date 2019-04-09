#include <decentralised_planning/path_planner.h>
#include "decentralised_planning/GetPath.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "update_goal_client");
    
	if (argc != 5)
    {
        ROS_INFO_STREAM("usage: rosrun update_goal_client [agent_<id_number>] [goal_x] [goal_y] [goal_yaw]");
    }
	
	else
	{
		ros::NodeHandle nh;
		ros::ServiceClient client = nh.serviceClient<decentralised_planning::GetPath>("update_goal");
		ros::Rate loop_rate(10);

		ROS_INFO("Searching optimal path using A* Search Algorithm");

        decentralised_planning::GetPath srv;
		srv.request.agent_id = string(argv[1]);
        srv.request.goal_pose.x = atoll(argv[2]);
        srv.request.goal_pose.y = atoll(argv[3]);
        srv.request.goal_pose.theta = atoll(argv[4]);
        
        if (client.call(srv))
        {
            ROS_INFO("Found the optimal Path");
            ROS_INFO("Open Rviz and Visualize the Path");

            ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

            float f = 0.0;
            while (ros::ok())
            {

                visualization_msgs::Marker points, line_strip, line_list;
                points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
                points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
                points.ns = line_strip.ns = line_list.ns = "points_and_lines";
                points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
                points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

                points.id = 0;
                line_strip.id = 1;
                line_list.id = 2;

                points.type = visualization_msgs::Marker::POINTS;
                line_strip.type = visualization_msgs::Marker::LINE_STRIP;
                line_list.type = visualization_msgs::Marker::LINE_LIST;

                // POINTS markers use x and y scale for width/height respectively
                points.scale.x = 0.2;
                points.scale.y = 0.2;

                // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
                line_strip.scale.x = 0.1;
                line_list.scale.x = 0.1;

                // Points are green
                points.color.g = 1.0f;
                points.color.a = 1.0;

                // Line strip is blue
                line_strip.color.b = 1.0;
                line_strip.color.a = 1.0;

                // Line list is red
                line_list.color.r = 1.0;
                line_list.color.a = 1.0;

                // Create the vertices for the points and lines
                for (uint32_t i = 0; i < 11; ++i)
                {
                for (uint32_t j = 0; j < 11; ++j)
                {
                    geometry_msgs::Point p;
                    p.x = i;
                    p.y = j;
                    p.z = 0.5;  

                for (auto i = srv.response.path.begin(); i != srv.response.path.end(); i++)
                {
                    geometry_msgs::Pose2D path_points = *i;
                    geometry_msgs::Point path_point;
                    //cout << "(" << p.first << ", " << p.second << ")-->";
                    path_point.x = path_points.x;
                    path_point.y = path_points.y;
                    path_point.z = 0.5;
                    line_strip.points.push_back(path_point);
                    //line_list.points.push_back(path_point);
                }

                    points.points.push_back(p);
                }

                }
                marker_pub.publish(points);
                marker_pub.publish(line_strip);
                //marker_pub.publish(line_list);

                loop_rate.sleep();

                f += 0.04;
            }

        }
        else
        {
            ROS_ERROR("Failed to call service get_plan");
            return 1;
        }
	}
	return 0;
}


