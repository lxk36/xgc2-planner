/*
	FILE: tracking_controller_node.cpp
	---------------------------------
	tracking controller for px4-based quadcopter
*/
#include <xgc2_cerlab_tracking_controller/trackingController.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "tracking_controller_node");
	ros::NodeHandle nh;
	controller::trackingController tc (nh);
	ros::spin();

	return 0;
}