#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <math.h>
#include <string>
#include <cstring>

#include <sstream>

class Noisifier
{

	private:
		//Node handle, publisher and subscriber
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Publisher pub;

		//noise
		float noise[6];
		int agent;
		int first;
		int second;

	public:
		void init(int ag);
		void generate();
		void noiseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

};


