#include "rrt/rrt.h"


RRT::RRT() : nh_("")
{
//   add subscribers and init functions
	//occ_grid_sub_ =  nh_.subscribe("occupancy_map", 1000, &RRT::occGridCallback, this);
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("rrt_visualization", 10);
    vis_pub_start_ = nh_.advertise<visualization_msgs::Marker>("start_visualization", 10);
    vis_pub_goal_ = nh_.advertise<visualization_msgs::Marker>("goal_visualization", 10);
    //obstacles = new Obstacles;
    start_pos_.x() = std::get<0>(start_node_);
    start_pos_.y() = std::get<1>(start_node_);
    end_pos_.x() = std::get<0>(goal_node_);;
    end_pos_.y() = std::get<1>(goal_node_);;
    root_ = new Node;
    root_->parent = NULL;
    root_->position = start_pos_;
    last_node_ = root_;
    nodes_.push_back(root_);
    step_size_ = 1;


	planPath();
};

RRT::~RRT(){};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrt_node");
  RRT obj;
  ros::spin();
  return 0;
};


//void RRT::occGridCallback(const nav_msgs::OccupanyGrid::ConstPtr& msg)
//{
//  occ_grid_ = std::make_shared<nav_msgs::OccupancyGrid>(*msg);
//  map_width_ = occ_grid_->info.width;
//  map_height_ = occ_grid_->info.height;
//}

Node* RRT::getRandomNode()
{
    Node* ret;
    // Vector2f point(drand48() * map_width_ ,drand48() * map_height_);
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0, map_height_);
    int it1 = dist(rng);
    int it2 = dist(rng);

    Vector2f point(it1, it2);
    if (point.x() >= 0 && point.x() <= map_width_ && point.y() >= 0 && point.y() <= map_height_) {
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

int RRT::distance(Vector2f &p, Vector2f &q)
{
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

Vector2f RRT::newConfig(Node *q, Node *qNearest)
{
    Vector2f to = q->position;
    Vector2f from = qNearest->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size_ * intermediate;
    return ret;
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
    if (distance(last_node_->position, end_pos_) < 2.0)
        return true;
    return false;
}

void RRT::planPath(){
	// RRT Algorithm
    auto start_time = std::chrono::high_resolution_clock::now();
	for(int i = 0; i < max_iter_; i++) {
		Node *q = getRandomNode();
		if (q) {
			Node *qNearest = nearest(q->position);
			if (distance(q->position, qNearest->position) > step_size_) {
				Vector2f new_config = newConfig(q, qNearest);
				if (true) {//{!rrt->obstacles->isSegmentInObstacle(newConfig, qNearest->position)
					Node *qNew = new Node;
					qNew->position = new_config;
                    // std::cout<<qNew->position.x()<<"\n";
                    // std::cout<<qNew->position.y()<<"\n";
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
    }
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time);
    std::cout<<"Execution Time: "<< duration.count() << " milliseconds" << std::endl;
    viz();
}

void RRT::vizStartAndGoal()
{
  visualization_msgs::Marker start;
  start.header.frame_id = "/map";
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
  start.scale.x = 1;
  start.scale.y = 1;
  start.scale.z = 1;
  start.color.a = 1.0; // Don't forget to set the alpha!
  start.color.r = 0.0;
  start.color.g = 1.0;
  start.color.b = 0.0;
  vis_pub_start_.publish( start );
  
  visualization_msgs::Marker goal;
  goal.header.frame_id = "/map";
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
  goal.scale.x = 1;
  goal.scale.y = 1;
  goal.scale.z = 1;
  goal.color.a = 1.0; // Don't forget to set the alpha!
  goal.color.r = 1.0;
  goal.color.g = 0.0;
  goal.color.b = 0.0;
  vis_pub_goal_.publish( goal );
}

void RRT::vizPath(){
// %Tag(MARKER_INIT)%
    visualization_msgs::Marker points {};
    visualization_msgs::Marker line_strip{};
    visualization_msgs::Marker line_list {};
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "rrt_path";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
// %EndTag(MARKER_INIT)%

// %Tag(ID)%
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
// %EndTag(ID)%

// %Tag(TYPE)%
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
// %EndTag(TYPE)%

// %Tag(SCALE)%
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;
// %EndTag(SCALE)%

// %Tag(COLOR)%
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(HELIX)%
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
// %EndTag(HELIX)%

    marker_pub_.publish(points);
    marker_pub_.publish(line_strip);
    //marker_pub.publish(line_list);

}

void RRT::viz(){
  ros::Rate r(10);
  while (ros::ok())
  {
//  %Final Path
    vizPath();
//  %Start and goal points
    vizStartAndGoal();
    r.sleep();
  }
}

