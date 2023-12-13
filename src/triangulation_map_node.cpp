#include <ros/ros.h>
#include <triangulation_map/triangulationMap.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "triangulation_map_node");
	ros::NodeHandle nh;

	triangulationMap::triangulatorMap t;
	t.initTriangulatorMap(nh);

	ros::spin();

	return 0;
}
