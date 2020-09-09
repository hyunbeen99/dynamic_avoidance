#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <math.h>

#include <std_msgs/Bool.h>
#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/SegmentObstacle.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#define CAR_SPEED 2.1
#define DIST_HP 5

using namespace std;


class DynamicAvoidance {
	public:
		// value 
		int standard_distance_ = 0;
		
		// messages
		geometry_msgs::Point nearestPoint_;
		geometry_msgs::Point center_point_;
		geometry_msgs::Point point_;

		// node
		ros::NodeHandle nh_;
		ros::Publisher flag_pub_;
		ros::Subscriber sub_;

		// flag
		bool stop_once_flag_;

		void exect();
		double calcDistance (geometry_msgs::Point point_) {return sqrt(pow(point_.x , 2) + (point_.y , 2)) ;}
		void obstacleCallback (const obstacle_detector::Obstacles obs);
	
		DynamicAvoidance() {}
};

