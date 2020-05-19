#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <stdlib.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <Eigen/Core>
#include <tuple>
#include <random>
#include <chrono>

using namespace Eigen;

struct Node {
    std::vector<Node *> children;
    Node *parent;
    Vector2f position;
};


class RRT{
  public:
    RRT();
    ~RRT();

  private:
    ros::NodeHandle nh_;
    
    //Subscribers
    ros::Subscriber occupancy_grid_;
    
    //Publishers
	ros::Publisher marker_pub_;
	ros::Publisher vis_pub_start_ ;
    ros::Publisher vis_pub_goal_ ;
    
    //Member variables
    std::tuple<int, int> start_node_ {0, 0};
    std::tuple<int, int> goal_node_ {50, 50};
    std::shared_ptr <nav_msgs::OccupancyGrid> occ_grid_;
	
	int max_iter_ {3000};
	int step_size_ ;
	std::vector<Node *> nodes_;
	Node *root_, *last_node_;
    Vector2f start_pos_ , end_pos_;
	std::vector<Node *> path_;
    
    unsigned int map_width_ {60}; //get from subscribers
    unsigned int map_height_ {60};
    std::vector<std::tuple<int, int>> obstacles_;
    
    //Member functions
	Node* getRandomNode();
    Node* nearest(Vector2f point);
    int distance(Vector2f &p, Vector2f &q);
    Vector2f newConfig(Node *q, Node *qNearest);
    bool reached();
    void add(Node *qNearest, Node *qNew);
    void occGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void deleteNodes(Node *root);
	void planPath();
	void viz();
	void vizStartAndGoal();
    void vizPath();
    
    };

#endif
