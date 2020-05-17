#include "rrt/rrt.h"


RRT::RRT() : nh_("")
{
//   add subscribers and init functions
	//occ_grid_sub_ =  nh_.subscribe("occupancy_map", 1000, &RRT::occGridCallback, this);
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    //obstacles = new Obstacles;
    start_pos_.x() = 10.0;
    start_pos_.y() = 10.0;
    end_pos_.x() = 20.0;
    end_pos_.y() = 20.0;
    root_ = new Node;
    root_->parent = NULL;
    root_->position = start_pos_;
    last_node_ = root_;
    nodes_.push_back(root_);
    step_size_ = 3;
    max_iter_ = 3000;
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
    Vector2f point(drand48() * map_width_ ,drand48() * map_height_);
    if (point.x() >= 0 && point.x() <= map_width_ && point.y() >= 0 && point.y() <= map_height_) {
        ret = new Node;
        ret->position = point;
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
		for(int i = 0; i < max_iter_; i++) {
			Node *q = getRandomNode();
			if (q) {
				Node *qNearest = nearest(q->position);
				if (distance(q->position, qNearest->position) > step_size_) {
					Vector2f new_config = newConfig(q, qNearest);
					if (true) {//{!rrt->obstacles->isSegmentInObstacle(newConfig, qNearest->position)
						Node *qNew = new Node;
						qNew->position = new_config;
						add(qNearest, qNew);
					}
				}
			if (reached()) {
			std::cout << "Reached Destination" << std::endl;
			break;
				}
			}
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
			viz();
		}
}

void RRT::viz(){
  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {
// %Tag(MARKER_INIT)%
    visualization_msgs::Marker points, line_strip, line_list;
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

    r.sleep();

    f += 0.04;
  }
// %EndTag(FULLTEXT)%
}

