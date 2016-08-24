#include "Noisifier.h"

void Noisifier::init(int ag) {

	agent = ag;
	first = agent;	
	second = (agent)%3+1;

	std::stringstream ss;

	ss << "z" << agent << "vec";

	std::string pub_str = ss.str(); 

	sub = n.subscribe("t0vec", 1000, &Noisifier::noiseCallback, this);
	pub = n.advertise<std_msgs::Float32MultiArray>(pub_str, 100);
}

void Noisifier::generate() {
	for (int i = 0; i<7 ; i++) {
		srand(time(NULL)*agent);
		noise[i]=0.2*((float)rand()/ ((RAND_MAX)) - 0.5);
	}
	
}
 
void Noisifier::noiseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
	std_msgs::Float32MultiArray x_out;
	int vec;

	ROS_INFO("a: %d", agent);
	ROS_INFO("f: %d", first);
	ROS_INFO("s: %d", second);
	ROS_INFO("n: %f", noise[6]);

	if (noise[6]>-100) {
		vec = first;
	}
	else {
		vec = 0;
	}


	ROS_INFO("VEC: %d", vec);

	std::stringstream ss;
	std::stringstream sn;
	generate();

	x_out.data.clear();
	ROS_INFO("%d\n",vec);

	for (int i = 0; i < 3; i++) {
		float val=0;
		float noi=0;

		if (vec == 1 || vec ==3) {
			val+=msg->data[i]+noise[i];
			noi+=noise[i];
		}	

		if (vec == 2 || vec ==3) {
			val+=msg->data[i+6]+noise[i+3];
			noi+=noise[i+3];
		}

		sn << noi << " ";
		ss << val << " ";

		x_out.data.push_back(val);
	}

	x_out.data.push_back(vec);
	ss << vec;

	std::string str = ss.str();
	std::string nstr = sn.str();

	ROS_INFO("Z: %s", str.c_str());

	ROS_INFO("N: %s", nstr.c_str());

	pub.publish(x_out);
}



