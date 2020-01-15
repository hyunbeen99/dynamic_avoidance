#include "dynamic_avoidance.h"
#include "ros/ros.h"

using namespace std;



void DynamicAvoidance::exect() {

	ackermann_msgs::AckermannDriveStamped ackerData_;
	sub_ = nh_.subscribe("raw_obstacles", 1, &DynamicAvoidance::obstacleCallback, this);
	ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ctrl_cmd", 10);
}

void DynamicAvoidance::obstacleCallback (const obstacle_detector::Obstacles obs) {

	ackermann_pub_.publish(ackerData_);

	obstacle_detector::SegmentObstacle nearest_segment ;
	nearestPoint_.x = 100.0; 

	ackerData_.drive.speed = 4;
	ackerData_.drive.steering_angle = 0;

	double dist;
	
	for (auto i : obs.segments) {
		
		center_point_.x = (i.first_point.x + i.last_point.x) / 2;
		center_point_.y = (i.first_point.y + i.last_point.y) / 2;
		dist = calcDistance(center_point_);
		
		if (nearestPoint_.x > dist) {
			nearestPoint_ = center_point_;
			nearest_segment = i;
		}		
	}

	//for stop
	standard_distance_ = calcDistance(center_point_);
	if (nearest_segment.last_point.y > 0 && standard_distance_ == 6) {
		ackerData_.drive.speed = 0;
		ROS_INFO ("############stop##########END#MISSION#####");
		speed_zero_flag_ = 1;
		ros::Duration(5).sleep();
	}

}


int main(int argc, char** argv) {
	
	ros::init(argc, argv, "dynamic_avoidance_node");

	DynamicAvoidance node;

        ros::spin();

	return 0;
	
}





