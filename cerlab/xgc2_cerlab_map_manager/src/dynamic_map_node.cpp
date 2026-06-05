#include <ros/ros.h>
#include <xgc2_cerlab_map_manager/dynamicMap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "dyanmic_map_node");
	ros::NodeHandle nh;

	mapManager::dynamicMap m(nh);

	ros::spin();

	return 0;
}