#include "dynamic_avoidance.h"
#include "ros/ros.h"

using namespace std;


void DynamicAvoidance::exect() {

	sub_ = nh_.subscribe("raw_obstacles", 1, &DynamicAvoidance::obstacleCallback, this);
	flag_pub_ = nh_.advertise<std_msgs::Bool>("/dynamic_stop_flag", 10);

	stop_once_flag_ = false;

}

void DynamicAvoidance::obstacleCallback (const obstacle_detector::Obstacles obs) {

	int cur_state = -1;
	nh_.getParam("/kuuve_state", cur_state);

	std_msgs::Bool stop_flag_to_kuuve_control;

	if(cur_state == 3) { // 3 is DYNAMIC state in KUUVe Control
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

		standard_distance_ = calcDistance(nearestPoint_);

		if (standard_distance_ < DIST_HP) {
			ROS_INFO ("############stop##########END#MISSION#####"); 
		
			stop_flag_to_kuuve_control.data = true;
			stop_once_flag_ = true;

		} else {
			stop_flag_to_kuuve_control.data = false;

			if(stop_once_flag_ == true){
				ros::shutdown();
			}
		}
		
		flag_pub_.publish(stop_flag_to_kuuve_control);
		cout << "dist -> " << standard_distance_ << endl;
	}
}


int main(int argc, char** argv) {
	
	ros::init(argc, argv, "dynamic_avoidance_node");

	DynamicAvoidance node;
	node.exect();
	ros::spin();

	return 0;
}

