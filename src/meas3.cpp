#include "Noisifier.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "meas3");

	ros::NodeHandle n;

	Noisifier noise;
	noise.init(3);
	
	while (ros::ok())
	{
		ros::spinOnce();
	}

	return 0;

}
