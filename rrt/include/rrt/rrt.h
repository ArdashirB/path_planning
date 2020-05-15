#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class RRT{
  public:
    RRT();
    ~RRT();

  private:
    ros::NodeHandle nh_;
    
    //Subscribers
    ros::Subscriber occupancy_grid_;
    
    //Publishers
    
    //Member variables
    std::tuple<int, int> start_node {10, 10};
    std::tuple<int, int> goal_node {20, 20};
    std::shared_ptr <nav_msgs::Odometry> occ_grid_;
    
    unsigned int map_width_ {}; //get from subscribers
    unsigned int map_height_ {};
    std::vector<std::tuple<int, int>> obstacles_;
    
    //Member functions
    void occGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    
    };
