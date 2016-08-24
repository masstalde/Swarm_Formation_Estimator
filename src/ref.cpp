#include "kalman_consensus.h"

using namespace kalman_consensus_ns;

bool SingleSystem::init(int ag) {
	
	config=ag;

	pub = n.advertise<std_msgs::Float32MultiArray>("trvec", 100);

	agent=0;
 
	x_<< 1,1,1,0,0,0,1,1,1,0,0,0;
}



double* SingleSystem::get_x_arr() {
	double x_arr[StateDim];	
	double *x_p = x_.data();
	for (int i=0; i<StateDim; i++) {
		x_arr[i]=x_p[i];
	}
	return x_arr;
}

void SingleSystem::publish_t() {
	std_msgs::Float32MultiArray x_out;
	for (int i = 0; i < StateDim; i++) {	
		x_out.data.push_back(x_(i));
	}
	pub.publish(x_out);
	print_stuff();
	x_out.data.clear();

}


void SingleSystem::print_stuff() {
	std::stringstream ss; 	

	ss << x_(0) << ", "<< x_(1) << ", "<< x_(2) << ", "<< x_(3) << ", "<< x_(4)
		 << ", "<< x_(5) << ", "<< x_(6) << ", "<< x_(7) << ", "<< x_(8) << ", "<< x_(9)
		 << ", "<< x_(10) << ", "<< x_(11);

	std::string str = ss.str(); 
	
	std::stringstream sa; 	

	sa << avail[0] << ", "<< avail[1] << ", "<< avail[2];

	std::string stra = sa.str(); 

	ROS_INFO("X: %s", str.c_str());
	ROS_INFO("Avail: %s", stra.c_str());
}

void SingleSystem::run_true() {
	if (config==1) {
		x_=x_;
		agent++;
	}
	if (agent%80==0) {
		x_=-x_;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "true");

	ros::NodeHandle n;
	
	ros::Rate loop_rate(20);

	SingleSystem s;
	s.init(1);
	
	while (ros::ok())
	{

		s.run_true();
		s.publish_t();

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;

}
