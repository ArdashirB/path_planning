#ifndef VIZ_H
#define VIZ_H

#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <tuple>
#include <vector>

using namespace Eigen;

class VIZ{
    public:
        VIZ();
        ~VIZ();
    private:
        ros::NodeHandle nh;

        //Publishers
        ros::Publisher marker_pub_;
        ros::Publisher vis_pub_start_ ;
        ros::Publisher vis_pub_goal_ ;
        ros::Publisher vis_pub_obstacle_ ;

        //Member functions
		friend class RRT;
        void vizStartAndGoal(std::tuple<int,int>, std::tuple<int,int>);
        void vizObstacles(std::vector<Vector2f>);
        void vizPath(std::vector<Node *>);

};

#endif