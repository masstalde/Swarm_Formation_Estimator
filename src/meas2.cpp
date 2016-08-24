#include "Noisifier.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "meas2");

	ros::NodeHandle n;
	
	Noisifier noise;
	noise.init(2);
	
	while (ros::ok())
	{
		ros::spinOnce();
	}

	return 0;

}
