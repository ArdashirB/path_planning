#include "rrt/rrt.h"

RRT::RRT() : nh_("")
{
//   add subscribers and init functions
occ_grid_sub_ =  nh_.subscribe("occupancy_map", 1000, &RRT::occGridCallback, this);
};

RRT::~RRT(){};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrt_node");
  RRT obj;
  
  ros::spin();
  return 0;
};


void RRT::occGridCallback(const nav_msgs::OccupanyGrid::ConstPtr& msg)
{
  occ_grid_ = std::make_shared<nav_msgs::OccupancyGrid>(*msg);
  map_width_ = occ_grid_->info.width;
  map_height_ = occ_grid_->info.height;
}

void RRT::planpath(){
doing 
}

