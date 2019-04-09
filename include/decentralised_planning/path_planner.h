#include <iostream>
#include <vector>
#include <utility>
#include <set>
#include <list>
#include <stack>
#include <math.h>
#include <float.h>
#include <map>
#include <sstream>
#include <cstdlib>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include <visualization_msgs/Marker.h>

using namespace std;

#define ROW 10
#define COL 10

typedef pair<int, int> Pair;

class node
{
public:
	int m_x;
	int m_y;
	double m_yaw;
	double m_cost;
	double m_g;
	double m_h;
	double m_f;
	int parent_i;
	int parent_j;
	bool flag;

	node()  //Default Constructor
	{

	}

	node(int x, int y, double yaw = 0, double cost = 0)// , double f = 999, double g = 999, double h = 999, int parent_x = -1, int parent_y = -1, bool symbol = false)
	{
		m_x = x;
		m_y = y;
		m_yaw = yaw;
		m_cost = cost;
	}

	~node()
	{

	}

	void setValues(int x, int y)
	{
		m_x = x;
		m_y = y;
	}
	void print()
	{
		cout << "( " << m_x << ", " <<  m_y <<")" << endl;
	}
};

void print_node( node a); 
// print_node - prints the node x and y values
// @param const node

void print_grid(const vector< vector<node> >&); 
// print_grid - prints the x, y, yaw, cost of each node of each node
// @param const vector< vector<node> >

int cal_distance(node a, node b);
// cal_distance - return distance between node a and node b
// @param node a, node b

bool isValid(int row, int col); 
// isValid - Checks weather the node is within the roadmap
// @param int row, int col

bool isDestination(node current, node goal); 
// isDestination - Checks weather is the current node is the goal node
// @param node current, node goal

void node_info(vector< vector<node> > grid); 
// node_info - Give information of the node

void con_grid_vec(int *i, int *j); 
// con_grid_vec - Converts the grid co-ordinates to the vector< vector<node> > co-ordinates

void con_vec_grid(int *i, int *j); 
// con_vec_grid - Converts the vector< vector<node> > co-ordintaes to grid co-ordinates

void print_node_grid_co(node z, vector< vector<node> > grid); 
// print_vec_grid - Prints info of the node taking inputs in the form of grid Co-ordintes

vector< Pair > tracePath(Pair start, Pair goal, vector< vector<node> > grid); 
// tracePath - This function will follow the parent nodes and gives a path to reach from start to goal

vector< Pair > a_star_path_finder(int i1, int j1, int i2, int j2, vector<vector<node>> grid);

vector< vector< node > > initialise_grid(vector< vector<node> > grid);

int num_extract(string s1);

std::map< int, Pair > init_agent_map(std::map< int, Pair > agent_map);

void print_agent(std::map< int, Pair > agent_map);

Pair get_agent_info(string agent, std::map< int, Pair > agent_map);


vector< Pair > tracePath(Pair start, Pair goal, vector< vector<node> > grid)
{
	vector<Pair> path_vector;
	int x = start.first;
	int y = start.second;
	int px = goal.first;
	int py = goal.second;

	path_vector.push_back(make_pair(px, py));
	while (!((grid[px][py].parent_i == x) && (grid[px][py].parent_j == y)))
	{
		int temp_i = grid[px][py].parent_i;
		int temp_j = grid[px][py].parent_j;
		px = temp_i;
		py = temp_j;
		path_vector.push_back(make_pair(px, py));
	}
	path_vector.push_back(make_pair(x, y));

	cout << "Start to end path" << endl;
	vector<Pair> FinalPath;
	for (auto i = path_vector.rbegin(); i != path_vector.rend(); i++)
	{
		Pair p = *i;
		FinalPath.push_back(p);
		//cout << "(" << p.first << ", " << p.second << ")-->";
	}

	cout << "\n" << "Vector content" << endl;
	for (auto i = FinalPath.begin(); i != FinalPath.end(); i++)
	{
		Pair p = *i;
		con_vec_grid(&p.first, &p.second);
		cout << "===> (" << p.first << ", " << p.second << ")";
	}
	return FinalPath;
}

bool isDestination(node current, node goal)
{
	return ((current.m_x == goal.m_x) && (current.m_y == goal.m_y));
}

bool isValid(int row, int col)
{
	return (row >= 0) && (row <= COL) && (col >= 0) && (col <= ROW);
}

int cal_distance(node a, node b)
{
	//return (sqrt((a.m_x - b.m_x)*(a.m_x - b.m_x) + (a.m_y - b.m_y)*(a.m_y - b.m_y))); // euclidean distance
	return ( ( abs(a.m_x - b.m_x) + abs(a.m_y - b.m_y) )*10 ); // Manhattan distance
}

void print_node(node z)
{
	cout << "Co-ordinates :" << "(" << z.m_x << ", " << z.m_y << ")" << '\n';
	cout << "Cost : " << z.m_cost << '\n';
	cout << "g : " << z.m_g << '\n';
	cout << "h : " << z.m_h << '\n';
	cout << "f : " << z.m_f << '\n';
	cout << "parent node" << "(" << z.parent_i << ", " << z.parent_j << ")" << endl;
	
}

void print_node_grid_co(node z, vector< vector<node> > grid)
{
	int a, b;
	a = z.m_x; b = z.m_y;
	int x, y;
	x = 10 - b;
	y = a;

	print_node(grid[x][y]);

}

void print_grid(const vector< vector<node> >& grid)
{
	for (int i = 0; i < grid.size(); i++)
	{
		for (int j = 0; j < grid[i].size(); j++)
		{
			cout <<  "(" << grid[i][j].m_x << ", " << grid[i][j].m_y <<  ")  ";
			//print_node(grid[i][j]);
		}
		cout << '\n';
	}
}

void node_info(vector< vector<node> > grid)
{
	cout << "Give Node x value :";
	int a;
	cin >> a;
	cout << endl;

	cout << "Give Node y value :";
	int b;
	cin >> b;
	cout << endl;
	
	int x, y;
	x = 10 - b;
	y = a;
	
	//con_grid_vec(&a, &b);
	print_node(grid[x][y]);
}

void con_grid_vec(int *i, int *j)
{
	int x, y;
	x = *i; y = *j;
	*i = ROW - y;
	*j = x;
}

void con_vec_grid(int *i, int *j)
{
	int x, y;
	x = *i; y = *j;
	*i = y;
	*j = ROW - x;
}

vector< Pair > a_star_path_finder(int i1,int j1,int i2,int j2, vector<vector<node>> grid)
{
	//int i1 = start.first; int j1 = start.second;
	//int i2 = goal.first; int j2 = goal.second;
	con_grid_vec(&i1, &j1); // Converted Start node in Grid Co-ordinates to Vec Co-ordinates 
	con_grid_vec(&i2, &j2); // Converted Goal node in Grid Co-ordinates to Vec Co-ordinates

	vector< Pair > zero_value;
	int z1 = 0; int z2 = 0;
	Pair z = make_pair(z1, z2);
	zero_value.push_back(z);

	// Initialization
	int i, j; // Index elements

	// Check weather the start and goal node are in the grid map
	if (isValid(grid[i1][j1].m_x, grid[i1][j1].m_y) && isValid(grid[i2][j2].m_x, grid[i2][j2].m_y) == false)
	{
		cout << "Source or Goal node is Invalid" << endl;
		return zero_value;
	}

	// Check weather the start node is the goal node
	if (isDestination(grid[i1][j1], grid[i2][j2]) == true)
	{
		cout << ("We are already at the destination") << endl;
		return zero_value;
	}

	// Initialize the parameters of the starting node
	grid[i1][j1].m_g = 0;
	grid[i1][j1].m_h = 0;
	grid[i1][j1].m_f = 0;
	grid[i1][j1].parent_i = i1;
	grid[i1][j1].parent_j = j1;

	// Print the vector coordinates of start and goal
	cout << "Start Node :" << endl;
	cout << "i1 : " << i1 << ", " << "j1 : " << j1 << endl;
	print_node(grid[i1][j1]);

	cout << "Goal Node :" << endl;
	cout << "i2 : " << i2 << ", " << "j2 : " << j2 << endl;
	print_node(grid[i2][j2]);

	// Initialize Pair's of start and goal
	Pair start(i1, j1);
	Pair goal(i2, j2);

	// Initialize Open List
	list <node> openList;

	// Inserting first node in the open list
	openList.push_back(grid[i1][j1]);

	bool foundDest = false;

	while (!openList.empty())
	{
		node a = openList.front();

		// Remove the node from the open list
		list<node>::iterator itr = openList.begin();
		openList.erase(itr);

		cout << "-------------------------Iteration-----------------------------" << endl;

		i = a.m_x; j = a.m_y;
		cout << "Parent Node :" << endl;
		cout << "(m_x, m_y) : " << "( " << i << ", " << j << ")" << endl;
		con_grid_vec(&i, &j);
		cout << "(i, j) : " << "(" << i << ", " << j << ")" << endl;
		cout << "Removed from the list : " << endl;
		print_node(grid[i][j]);
		grid[i][j].flag = true;

		double gNew, hNew, fNew;

		cout << "-----------front successor ------------" << endl;

		if ((isValid(i, j + 1)) == true)
		{
			print_node(grid[i][j+1]);
			if (isDestination(grid[i][j + 1], grid[i2][j2]) == true)
			{
				grid[i][j + 1].parent_i = i;
				grid[i][j + 1].parent_j = j;
				cout << "The destination cell is found" << endl;
				//print_node(grid[i][j + 1]);

				// Tracepath function gets the path
				return tracePath(start, goal, grid);
				//return zero_value;
			}

			else if (grid[i][j + 1].flag == false)
			{
				gNew = grid[i][j].m_g + 1;
				hNew = cal_distance(grid[i][j + 1], grid[i2][j2]);
				fNew = gNew + hNew;

				if (grid[i][j + 1].m_f == FLT_MAX || grid[i][j + 1].m_f > fNew)
				{
					openList.push_back(grid[i][j + 1]);

					grid[i][j + 1].m_f = fNew;
					grid[i][j + 1].m_g = gNew;
					grid[i][j + 1].m_h = hNew;
					grid[i][j + 1].parent_i = i;
					grid[i][j + 1].parent_j = j;
					//grid[i][j + 1].flag = true;
				}
				print_node(grid[i][j + 1]);
			}
			
		}

		cout << "-----------top successor---------------" << endl;

		if ((isValid(i - 1, j)) == true)
		{
			print_node(grid[i-1][j]);
			if (isDestination(grid[i - 1][j], grid[i2][j2]) == true)
			{
				grid[i - 1][j].parent_i = i;
				grid[i - 1][j].parent_j = j;
				cout << "The destination cell is found" << endl;
				//print_node(grid[i-1][j]);

				// Tracepath function gets the path
				return tracePath(start, goal, grid);
				//return zero_value;
			}

			// Check weather this node is in the closed list
			else if (grid[i - 1][j].flag == false)
			{
				gNew = grid[i][j].m_g + 1;
				hNew = cal_distance(grid[i - 1][j], grid[i2][j2]);
				fNew = gNew + hNew;

				if (grid[i - 1][j].m_f == FLT_MAX || grid[i - 1][j].m_f > fNew)
				{
					openList.push_back(grid[i - 1][j]);

					grid[i - 1][j].m_f = fNew;
					grid[i - 1][j].m_g = gNew;
					grid[i - 1][j].m_h = hNew;
					grid[i - 1][j].parent_i = i;
					grid[i - 1][j].parent_j = j;
					//grid[i - 1][j].flag = true;
				}
				print_node(grid[i - 1][j]);
			}
			
		}

		cout << "-----------left successor--------------" << endl;

		if ((isValid(i, j - 1)) == true)
		{
			print_node(grid[i][j-1]);
			if ((isDestination(grid[i][j - 1], grid[i2][j2])) == true)
			{
				grid[i][j - 1].parent_i = i;
				grid[i][j - 1].parent_j = j;
				cout << "The destination cell is found" << endl;
				//print_node(grid[i][j - 1]);

				// Tracepath function gets the path
				return tracePath(start, goal, grid);
				//return zero_value;
			}

			// Check weather this node is in the closed list
			else if (grid[i][j - 1].flag == false)
			{
				gNew = grid[i][j].m_g + 1;
				hNew = cal_distance(grid[i][j - 1], grid[i2][j2]);
				fNew = gNew + hNew;

				if (grid[i][j - 1].m_f == FLT_MAX || grid[i][j - 1].m_f > fNew)
				{
					openList.push_back(grid[i][j-1]);

					grid[i][j - 1].m_f = fNew;
					grid[i][j - 1].m_g = gNew;
					grid[i][j - 1].m_h = hNew;
					grid[i][j - 1].parent_i = i;
					grid[i][j - 1].parent_j = j;
					//grid[i][j - 1].flag = true;
				}
				print_node(grid[i][j - 1]);
			}
			
		}

		cout << "-----------down successor--------------" << endl;

		if ((isValid(i + 1, j)) == true)
		{
			print_node(grid[i + 1][j]);
			if (isDestination(grid[i + 1][j], grid[i2][j2]) == true)
			{
				grid[i + 1][j].parent_i = i;
				grid[i + 1][j].parent_j = j;
				cout << "The destination cell is found" << endl;

				// Tracepath function gets the path
				return tracePath(start, goal, grid);
				//return zero_value;
			}

			// Check weather this node is in the closed list
			else if (grid[i + 1][j].flag == false)
			{
				gNew = grid[i][j].m_g + 1;
				hNew = cal_distance(grid[i + 1][j], grid[i2][j2]);
				fNew = gNew + hNew;

				if (grid[i + 1][j].m_f == FLT_MAX || grid[i + 1][j].m_f > fNew)
				{
					openList.push_back(grid[i + 1][j]);

					grid[i + 1][j].m_f = fNew;
					grid[i + 1][j].m_g = gNew;
					grid[i + 1][j].m_h = hNew;
					grid[i + 1][j].parent_i = i;
					grid[i + 1][j].parent_j = j;
					//grid[i + 1][j].flag = true;
				}
				print_node(grid[i + 1][j]);
			}
		}
	}
	//node_info(grid);
}

void print_agent(std::map< int, Pair > agent_map)
{
	for (map<int, Pair>::iterator it = agent_map.begin(); it != agent_map.end(); it++)
	{
		cout << "agent_id : " << it->first << " Pair : (" << it->second.first << ", " << it->second.second << ")" << endl;
	}
}

Pair get_agent_info(string agent, std::map< int, Pair > agent_map)
{
	int agent_id = num_extract(agent);
	return agent_map[agent_id];
}

std::map< int, Pair > init_agent_map(std::map< int, Pair > agent_map)
{
	int agent_id = 1;
	for (int y = 0; y <= ROW; y++)
	{
		for (int x = 0; x <= COL; x++)
		{
			int i = x;
			int j = 10 - y;
			agent_map[agent_id] = make_pair(j, i);
			agent_id = agent_id + 1;
		}
	}
	return agent_map;
}

int num_extract(string s1)
{
	size_t pos;
	string token;
	int i;
	pos = s1.find("_");
	token = s1.substr(pos + 1, s1.length());
	i = stoi(token);
	return i;
}

vector< vector<node> > initialise_grid(vector< vector<node> > grid)
{
	for (int y = 0; y <= ROW; y++)
	{
		vector <node> row = {};
		for (int x = 0; x <= COL; x++)
		{
			// Sets the i, j  according to desired co-ordinates
			int i = x;
			int j = ROW - y;

			node node_(i, j);

			node_.m_f = FLT_MAX;
			node_.m_g = FLT_MAX;
			node_.m_h = FLT_MAX;
			node_.parent_i = -1;
			node_.parent_j = -1;
			node_.flag = false;

			row.push_back(node_);
		}
		grid.push_back(row);
	}
	return grid;
}