#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <math.h>

#include <std_msgs/String.h>
#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/SegmentObstacle.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#ifndef DYNAMIC_AVOIDANCE
#define DYNAMIC_AVOIDANCE

using namespace std;


class DynamicAvoidance {
	public:
		//value 
		int status_ = 0;
		int angle_ = 15;
		int standard_distance_ = 0;
		

		//flags 
		int speed_zero_flag_;

		//messages
		geometry_msgs::Point nearestPoint_;
		geometry_msgs::Point center_point_;
		geometry_msgs::Point point_;
		ackermann_msgs::AckermannDriveStamped ackerData_;

		//node
		ros::NodeHandle nh_;
		ros::Publisher ackermann_pub_;
		ros::Subscriber sub_;

	
		DynamicAvoidance() {}
		void exect();
		double calcDistance (geometry_msgs::Point point_) {return pow(point_.x , 2) + (point_.y , 2) ;}
		void obstacleCallback (const obstacle_detector::Obstacles obs);
				
};

#endif
