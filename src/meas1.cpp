#include "Noisifier.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "meas1");

	ros::NodeHandle n;

	Noisifier noise;
	noise.init(1);
	
	while (ros::ok())
	{
		ros::spinOnce();
	}

	return 0;

}
