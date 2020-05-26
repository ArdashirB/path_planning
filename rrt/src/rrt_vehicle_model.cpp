#include "rrt/rrt.h"


RRT::RRT() : nh_("")
{
//   add subscribers and init functions
	occ_grid_sub_ =  nh_.subscribe("/occupancy_map", 1000, &RRT::occGridCallback, this);
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("rrt_visualization", 10);
    vis_pub_start_ = nh_.advertise<visualization_msgs::Marker>("start_visualization", 10);
    vis_pub_goal_ = nh_.advertise<visualization_msgs::Marker>("goal_visualization", 10);
	vis_pub_obstacle_ = nh_.advertise<visualization_msgs::Marker>("obstacle_visualization", 10);
    //obstacles = new Obstacles;
};

RRT::~RRT(){};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrt_node");
  RRT obj;
  ros::spin();
  return 0;
};


void RRT::occGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    occ_grid_ = std::make_shared<nav_msgs::OccupancyGrid>(*msg);
    callEverything();
}

void RRT::populateObstacles(){
    obstacles_.clear();
    auto p = [&](int column, int row){return (row + map_height_/2)*map_width_+ (column + map_width_/2);}; 
    for (int row = -map_height_/2; row < map_height_/2; row++){
        for (int column = -map_width_/2; column < map_width_/2; column ++){
            if (occ_grid_->data[p(column,row)] != 0){
                Vector2f obsPosition;
                obsPosition.x() = column;
                obsPosition.y() = row;
                // std::cout << row << "," << column << std::endl;
                obstacles_.push_back(obsPosition);
            }
        }
    }
}
void RRT::callEverything(){
    auto start_time = std::chrono::high_resolution_clock::now();
    map_width_ = occ_grid_->info.width;
    map_height_ = occ_grid_->info.height;
    robot_radius_ = 2.0f;
    start_pos_.x() = std::get<0>(start_node_);
    start_pos_.y() = std::get<1>(start_node_);
    end_pos_.x() = std::get<0>(goal_node_);
    end_pos_.y() = std::get<1>(goal_node_);
    root_ = new Node;
    root_->parent = NULL;
    root_->position = start_pos_;
	root_->theta = start_theta_;
    last_node_ = root_;
    nodes_.push_back(root_);
    step_size_ = 1; // time in secs
	step_time_ = 0.5;
	velocity_ = 10; // m/sec
	delta_bound_ = 0.1; //radians
	wheel_base_ = 0.45; // meters
	populateObstacles();
    planPath();
    viz();
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
    std::cout<<"Execution Time: "<< duration.count() << " milliseconds" << std::endl;
}

Node* RRT::getRandomNode()
{
    Node* ret;
    // Vector2f point(drand48() * map_width_ ,drand48() * map_height_);
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist1(-map_width_/2, map_width_/2);
	std::uniform_int_distribution<std::mt19937::result_type> dist2(-map_height_/2, map_height_/2);
    int it1 = dist1(rng);
    int it2 = dist2(rng);

    Vector2f point(it1, it2);
    if (point.x() >= -map_width_/2 && point.x() < map_width_/2 && point.y() >= -map_height_/2 && point.y() < map_height_/2) {
        ret = new Node;
        ret->position = point;
        // std::cout<<it1<<"\n";
        // std::cout<<it2<<"\n";
        return ret;
    }
    return NULL;
}

Node* RRT::nearest(Vector2f point)
{
    float min_dist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes_.size(); i++) {
        float dist = distance(point, nodes_[i]->position);
        if (dist < min_dist) {
            min_dist= dist;
            closest = nodes_[i];
        }
    }	
    return closest;
}

float RRT::distance(Vector2f &p, Vector2f &q)
{
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

std::tuple<Vector2f,float> RRT::newConfig(Node *q, Node *qNearest)
{

	float thetaNew{0};
	Vector2f newPosition;
	float thet_relative_vector{0};
	float theta{0};
	float delta{0};
	const float pi{3.142};
	// Relative angle 
	Vector2f to = q->position;
    Vector2f from = qNearest->position;
	thet_relative_vector = atan2((to - from).y(),(to - from).x());
	//std::cout << "********************" << std::endl;
	//std::cout << "theta relative: " << thet_relative_vector << std::endl; 
	theta = qNearest->theta - thet_relative_vector;
	
	if (theta > pi && theta <= 2*pi) {
		theta = 2*pi - theta;
	} else if (theta < -pi && theta >= -2*pi){
		theta = -2*pi - theta;
	} 
	
	//std::cout << "theta: " << theta << std::endl; 
	//std::cout << "********************" << std::endl;
	//theta = qNearest->theta + (velocity_/wheel_base_)*tan(delta)*step_time_
	delta = atan2(theta - qNearest->theta,(step_time_*velocity_/wheel_base_));
	
	if (delta <= delta_bound_ && delta >= -delta_bound_){
		newPosition.x() = qNearest->position.x() + velocity_*cos(theta)*step_time_;
		newPosition.y() = qNearest->position.y() + velocity_*sin(theta)*step_time_;
		thetaNew = qNearest->theta + theta;
		auto n = std::make_tuple(newPosition,thetaNew);
		return n;
	}
	else if (delta > 0){
		delta = delta_bound_;
		newPosition.x() = qNearest->position.x() + velocity_*cos(theta)*step_time_;
		newPosition.y() = qNearest->position.y() + velocity_*sin(theta)*step_time_;
		theta = qNearest->theta + (velocity_/wheel_base_)*tan(delta)*step_time_;
		thetaNew = qNearest->theta + theta;
		auto n = std::make_tuple(newPosition,thetaNew);
		return n;
	} else {
		delta = -delta_bound_;
		newPosition.x() = qNearest->position.x() + velocity_*cos(theta)*step_time_;
		newPosition.y() = qNearest->position.y() + velocity_*sin(theta)*step_time_;
		theta = qNearest->theta + (velocity_/wheel_base_)*tan(delta)*step_time_;
		thetaNew = qNearest->theta + theta;
		auto n = std::make_tuple(newPosition,thetaNew);
		return n;
}
//	else {
//		
//	}
}



bool RRT::isNodeCloseToObstacle(Vector2f &newpoint){
	for(auto obstacle:obstacles_){
		// Checking if the discrete point is close to obstacles
		if(distance(obstacle, newpoint) < robot_radius_) {// needs to be discretized point  
			return true;
		}
	}
	return false;
}

void RRT::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNearest->children.push_back(qNew);
    nodes_.push_back(qNew);
    last_node_ = qNew;
}

bool RRT::reached()
{
    if (distance(last_node_->position, end_pos_) < 5.0)
        return true;
    return false;
}

void RRT::planPath(){
	// RRT Algorithm
    path_.clear();
    nodes_.clear();
    nodes_.push_back(root_);
    
	for(int i = 0; i < max_iter_; i++) {
		Node *q = getRandomNode();
		if (q) {
			Node *qNearest = nearest(q->position);
			if (distance(q->position, qNearest->position) > step_size_) {
				std::tuple<Vector2f,float> new_tuple = newConfig(q, qNearest);
				Vector2f new_config = std::get<0>(new_tuple);
				float new_theta = std::get<1>(new_tuple);
				if (!isNodeCloseToObstacle(new_config)) {
                    Node *qNew = new Node;
					qNew->position = new_config;
					qNew->theta = new_theta;
					add(qNearest, qNew);
				}
			}
        }
        if (reached()) {
		    std::cout << "Reached Destination at iteration: "<< i << std::endl;
		    break;
        }
	}
    Node *q;
    if (reached()) {
        q = last_node_;
    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q = nearest(end_pos_);
        std::cout << "Exceeded max iterations!" << std::endl;
    }
    // generate shortest path to destination.
    while (q != NULL) {
        path_.push_back(q);
        q = q->parent;
		//std::cout << q->theta << std::endl;
    }
    
}

void RRT::vizStartAndGoal()
{
  visualization_msgs::Marker start;
  start.header.frame_id = "/velodyne";
  start.header.stamp = ros::Time();
  start.ns = "rrt_start_goal_points";
  start.id = 0;
  start.type = visualization_msgs::Marker::SPHERE;
  start.action = visualization_msgs::Marker::ADD;
//start.action = 2;
  start.pose.position.x = std::get<0>(start_node_);
  start.pose.position.y = std::get<1>(start_node_);
  start.pose.position.z = 0.0;
  start.pose.orientation.x = 0.0;
  start.pose.orientation.y = 0.0;
  start.pose.orientation.z = 0.0;
  start.pose.orientation.w = 1.0;
  start.scale.x = 3;
  start.scale.y = 3;
  start.scale.z = 3;
  start.color.a = 1.0; // Don't forget to set the alpha!
  start.color.r = 0.0;
  start.color.g = 1.0;
  start.color.b = 0.0;
  vis_pub_start_.publish( start );
  
  visualization_msgs::Marker goal;
  goal.header.frame_id = "/velodyne";
  goal.header.stamp = ros::Time();
  goal.ns = "rrt_start_goal_points1";
  goal.id = 0;
  goal.type = visualization_msgs::Marker::SPHERE;
  goal.action = visualization_msgs::Marker::ADD;
//  closest.action = 2;
  goal.pose.position.x = std::get<0>(goal_node_);
  goal.pose.position.y = std::get<1>(goal_node_);
  goal.pose.position.z = 0.0;
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = 0.0;
  goal.pose.orientation.w = 1.0;
  goal.scale.x = 3;
  goal.scale.y = 3;
  goal.scale.z = 3;
  goal.color.a = 1.0; // Don't forget to set the alpha!
  goal.color.r = 1.0;
  goal.color.g = 0.0;
  goal.color.b = 0.0;
  vis_pub_goal_.publish( goal );
  
  // visualizing obstacles
  visualization_msgs::Marker points {};
    points.header.frame_id = "/velodyne";
    points.header.stamp = ros::Time::now();
    points.ns = "rrt_path";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 1;
    points.scale.y = 1;
    // Points are green
    points.color.g = 0.5f;
    points.color.a = 1.0; 
	points.color.r = 1.0;
	points.color.b = 1.0;

    // Create the vertices for the points and lines
    for (auto obstacle: obstacles_)
    {
      geometry_msgs::Point p;
      p.x = double(obstacle.x());
      p.y = double(obstacle.y());
      p.z = 0;
      points.points.push_back(p);
    }
    vis_pub_obstacle_.publish(points);
  }
  

void RRT::vizPath(){
    visualization_msgs::Marker points {};
    visualization_msgs::Marker line_strip{};
    visualization_msgs::Marker line_list {};
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/velodyne";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "rrt_path";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    points.scale.x = 1;
    points.scale.y = 1;
    line_strip.scale.x = 1;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    // // Line list is red
    // line_list.color.r = 1.0;
    // line_list.color.a = 1.0;

    // Create the vertices for the points and lines
    for (auto node: path_)
    {
      geometry_msgs::Point p;
      p.x = double(node->position.x());
      p.y = double(node->position.y());
      p.z = 0;
      points.points.push_back(p);
      line_strip.points.push_back(p);
//      // The line list needs two points for each line
//      line_list.points.push_back(p);
//      line_list.points.push_back(p);
    }
    marker_pub_.publish(points);
    marker_pub_.publish(line_strip);
    //marker_pub.publish(line_list);
}

void RRT::viz(){

//  %Final Path
    vizPath();
//  %Start and goal points
    vizStartAndGoal();
}

