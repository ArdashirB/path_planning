#include "rrt/viz.h"

VIZ::VIZ() : nh("")
{
//   add subscribers and init functions
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("rrt_visualization", 10);
    vis_pub_start_ = nh_.advertise<visualization_msgs::Marker>("start_visualization", 10);
    vis_pub_goal_ = nh_.advertise<visualization_msgs::Marker>("goal_visualization", 10);
	vis_pub_obstacle_ = nh_.advertise<visualization_msgs::Marker>("obstacle_visualization", 10);
};
VIZ::~VIZ(){};

void VIZ::vizStartAndGoal(std::tuple<int,int> start_node, std::tuple<int,int> goal_node)
{
    visualization_msgs::Marker start;
    start.header.frame_id = "/velodyne";
    start.header.stamp = ros::Time();
    start.ns = "rrt_start_goal_points";
    start.id = 0;
    start.type = visualization_msgs::Marker::SPHERE;
    start.action = visualization_msgs::Marker::ADD;
    //start.action = 2;
    start.pose.position.x = std::get<0>(start_node);
    start.pose.position.y = std::get<1>(start_node);
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
    goal.pose.position.x = std::get<0>(goal_node);
    goal.pose.position.y = std::get<1>(goal_node);
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
}

void VIZ::vizObstacles(std::vector<Vector2f> obstacles){//TODO Pass by reference by making it friend class??
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
    for (auto obstacle: obstacles)
    {
      geometry_msgs::Point p;
      p.x = double(obstacle.x());
      p.y = double(obstacle.y());
      p.z = 0;
      points.points.push_back(p);
    }
    vis_pub_obstacle_.publish(points);
}
void VIZ::vizPath(std::vector<Node *> path){
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