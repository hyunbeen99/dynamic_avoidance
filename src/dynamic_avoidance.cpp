#include "dynamic_avoidance.h"
#include "ros/ros.h"

using namespace std;



void DynamicAvoidance::exect() {

	ackermann_msgs::AckermannDriveStamped ackerData_;
	sub_ = nh_.subscribe("raw_obstacles", 1, &DynamicAvoidance::obstacleCallback, this);
	ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ctrl_cmd", 10);
}

void DynamicAvoidance::obstacleCallback (const obstacle_detector::Obstacles obs) {

	obstacle_detector::SegmentObstacle nearest_segment ;
	nearestPoint_.x = 100.0; 

   for (auto i : obs.segments) {
		
		center_point_.x = (i.first_point.x + i.last_point.x) / 2;
		center_point_.y = (i.first_point.y + i.last_point.y) / 2;
		double dist = calcDistance(center_point_);
		
		if (nearestPoint_.x > dist) {
			nearestPoint_ = center_point_;
			nearest_segment = i;
		}		
	}

	//for stop
	standard_distance_ = calcDistance(nearestPoint_);
	//if (0 < nearest_segment.last_point.y && standard_distance_ < DIST_HP) {
	if (standard_distance_ < DIST_HP) {
		ackerData_.drive.speed = 0;
		ROS_INFO ("############stop##########END#MISSION#####");
		//ros::Duration(5).sleep();
		ackerData_.drive.brake = 200;
	} else {
		ackerData_.drive.speed = CAR_SPEED;
		ackerData_.drive.steering_angle = 0;
	}

	cout << "dist -> " << standard_distance_ << endl;

	ackermann_pub_.publish(ackerData_);


}


int main(int argc, char** argv) {
	
	ros::init(argc, argv, "dynamic_avoidance_node");

	DynamicAvoidance node;
	node.exect();
    	ros::spin();

	return 0;
	
}

