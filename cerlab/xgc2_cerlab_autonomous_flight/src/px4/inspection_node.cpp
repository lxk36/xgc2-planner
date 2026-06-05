#include <ros/ros.h>
#include <xgc2_cerlab_autonomous_flight/px4/inspection.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "auto_inspection_node");
	ros::NodeHandle nh;

	AutoFlight::inspector ip (nh);
	ip.run();

	ros::spin();
	return 0;
}
