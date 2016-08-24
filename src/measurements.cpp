#include "kalman_consensus.h"

using namespace kalman_consensus_ns;

bool SingleSystem::init(int ag) {
	closed_ = false;
	
	agent = ag;
	std::stringstream ss_pub;

	ss_pub << "t" << agent << "vec";

	std::string pub_str = ss_pub.str(); 



	pub = n.advertise<std_msgs::Float32MultiArray>(pub_str, 100);
	pub_sim = n.advertise<std_msgs::Float32MultiArray>("tsvec", 100);

	t_sub_ref = n.subscribe("trvec", 1000, &SingleSystem::trCallback, this);
 
	int i = 0;
	int j = 0;

	avail[0]=0;
	avail[1]=0;
	avail[2]=0;

	while (i<ObsDim) {
		x_pred_(i) = 0;
		x_(i) = 1;
		v_(i) = 0;
		r_(i) = 0;

		n_(i)=0;

		u_(i) = 0;
	
		z_(i) = 0;
		w_(i) = 0;
		
		i++;
	}

	while (i<InDim) {
		x_pred_(i) = 0;
		x_(i) = 1;
		v_(i) =0;
		r_(i) = 0;

		n_(i)=0;

		u_(i) = 0;
		
		i++;
	}

	while (i<ComDim) {
		x_pred_(i) = 0;
		x_(i) = 1;
		v_(i) =0;
		r_(i) = 0;

		n_(i)=0;
		
		i++;
	}

	while (i<StateDim) {
		x_pred_(i) = 0;
		x_(i) = 1;
		v_(i) =0;
		r_(i) = 0;

		i++;
	}


	K_ << -15.7735, 0, 0, -4.6865, 0, 0, -4.2265, 0, 0, -1.7943, 0, 0,
			0, -15.7735, 0, 0, -4.6865, 0, 0, -4.2265, 0, 0, -1.7943, 0,
			0, 0, -15.7735, 0, 0, -4.6865, 0, 0, -4.2265, 0, 0, -1.7943,
			11.5470, 0, 0, 2.8922, 0, 0, -11.5470, 0, 0, -2.8922, 0, 0,
			0, 11.5470, 0, 0, 2.8922, 0, 0, -11.5470, 0, 0, -2.8922, 0,
			0, 0, 11.5470, 0, 0, 2.8922, 0, 0, -11.5470, 0, 0, -2.8922,
			4.2265, 0, 0, 1.7943, 0, 0, 15.7735, 0, 0, 4.6865, 0, 0,
			0, 4.2265, 0, 0, 1.7943, 0, 0, 15.7735, 0, 0, 4.6865, 0,
			0, 0, 4.2265, 0, 0, 1.7943, 0, 0, 15.7735, 0, 0, 4.6865;
   
N_bar_ <<	-15.7735, 0, 0, 0, 0, 0, -4.2265, 0, 0, 0, 0, 0,
					0, -15.7735, 0, 0, 0, 0, 0, -4.2265, 0, 0, 0, 0,
					0, 0, -15.7735, 0, 0, 0, 0, 0, -4.2265, 0, 0, 0,
					11.5470, 0, 0, 0, 0, 0, -11.5470, 0, 0, 0, 0, 0,
					0, 11.5470, 0, 0, 0, 0, 0, -11.5470, 0, 0, 0, 0,
					0, 0, 11.5470, 0, 0, 0, 0, 0, -11.5470, 0, 0, 0,
					4.2265, 0, 0, 0, 0, 0, 15.7735, 0, 0, 0, 0, 0,
					0, 4.2265, 0, 0, 0, 0, 0, 15.7735, 0, 0, 0, 0,
					0, 0, 4.2265, 0, 0, 0, 0, 0, 15.7735, 0, 0, 0;


	Eigen::Matrix<double, 6, 6> A;
	Eigen::Matrix<double, 6, 6> B_2;
	Eigen::Matrix<double, ComDim, ComDim> H_j;
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d O = Eigen::Matrix3d::Zero();

	
	A_ol_ = Eigen::MatrixXd::Zero(StateDim,StateDim);
	B_ = Eigen::MatrixXd::Zero(StateDim,InDim);

	A << 0, 0, 0, 1, 0, 0,
			 0, 0, 0, 0, 1, 0,
			 0, 0, 0, 0, 0, 1,
			 0, 0, 0, 0, 0, 0,
			 0, 0, 0, 0, 0, 0,
			 0, 0, 0, 0, 0, 0;

	B_2 << 0, 0, 0, 0, 0, 0, 
			   0, 0, 0, 0, 0, 0, 
			   0, 0, 0, 0, 0, 0, 
			   -1, 0, 0, 1, 0, 0, 
			 	 0, -1, 0, 0, 1, 0,
			   0, 0, -1, 0, 0, 1;


	for (int j=0; j<N-1;j++) {
		A_ol_.block(j*6,j*6,6,6) = A;
		B_.block(j*6,j*3,6,6) = B_2;
	}

	A_cl_=A_ol_-B_*K_;
	NB_=B_*N_bar_;

	C_[0] << I, O, O, O;
	C_[1] << O, O, I, O;
	C_[2] << I, O, I, O;

	H_[0] = Eigen::MatrixXd::Identity(ComDim,StateDim);
	H_[1] = Eigen::MatrixXd::Identity(ComDim,StateDim);
	H_[2] = Eigen::MatrixXd::Identity(ComDim,StateDim);

	P_ = Eigen::MatrixXd::Zero(StateDim,StateDim);
	P_pred_ = Eigen::MatrixXd::Zero(StateDim,StateDim);

	Q_ = Eigen::MatrixXd::Identity(StateDim,StateDim);
	
	R_[0] = Eigen::MatrixXd::Identity(ObsDim,ObsDim);
	R_[1] = Eigen::MatrixXd::Identity(ObsDim,ObsDim);
	R_[2] = Eigen::MatrixXd::Identity(ObsDim,ObsDim);

	N_[0] = Eigen::MatrixXd::Identity(ComDim,ComDim);
	N_[1] = Eigen::MatrixXd::Identity(ComDim,ComDim);
	N_[2] = Eigen::MatrixXd::Identity(ComDim,ComDim);

	last=ros::Time::now().toSec();
}

void SingleSystem::update_available() {
	if (avail[0]) {
		if (avail[1] && avail[2]) {
			C_r_3_.block(0,0,ObsDim,StateDim) = C_[avail[0]-1];
			R_r_3_.block(0,0,ObsDim,ObsDim) = R_[avail[0]-1];
			z_r_3_ << z_,
								tf_,
								ts_;
			config=3;
		}
		else if (avail[1]) {
			C_r_1_.block(0,0,ObsDim,StateDim) = C_[avail[0]-1];
			R_r_1_.block(0,0,ObsDim,ObsDim) = R_[avail[0]-1];
			C_r_1_.block(ObsDim,0,ComDim,StateDim) = H_[avail[1]-1];
			C_r_1_.block(ObsDim,0,ComDim,ComDim) = N_[avail[1]-1];
			z_r_1_ << z_,
								tf_;
			config=2;
		}
		else if (avail[2]) {
			C_r_1_.block(0,0,ObsDim,StateDim) = C_[avail[0]-1];
			R_r_1_.block(0,0,ObsDim,ObsDim) = R_[avail[0]-1];
			C_r_1_.block(ObsDim,0,ComDim,StateDim) = H_[avail[2]-1];
			R_r_1_.block(ObsDim,ObsDim,ComDim,ComDim) = N_[avail[2]-1];
			z_r_1_ << z_,
								ts_;
			config=2;
		}
		else {config=1;}
	}

	else {
		config = 3 + (bool)avail[1] + 2* (bool)avail[2];
		z_r_2_ << tf_,
							ts_;
	}
}



void SingleSystem::publish_t() {
	std_msgs::Float32MultiArray x_out, x_sim;
	for (int i = 0; i < StateDim; i++) {	
		srand(time(NULL));
		x_sim.data.push_back(x_(i));
		x_out.data.push_back(x_(i)+0.2*((float)rand()/ ((RAND_MAX)) - 0.5));
	}
	pub_sim.publish(x_sim);
	pub.publish(x_out);
	
	print_stuff();
	avail[0]=0;
	avail[1]=0;
	avail[2]=0;
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
	if (closed_) {

		x_ = A_cl_*x_;
	}

	else {

		double current =ros::Time::now().toSec();
		double dt = current-last;
		SSMatrix A= Eigen::Matrix<double, StateDim, StateDim>::Identity() + A_cl_*dt;
		last=current;

		ROS_INFO("**%f **\n",dt);
		x_ = A*x_ + NB_*r_*dt;
	}
}

void SingleSystem::trCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	int i=0;
	while (i<ComDim) {
		r_(i)=msg->data[i];
		i++;
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "true");

	ros::NodeHandle n;
	
	ros::Rate loop_rate(20);

	SingleSystem s;
	s.init(0);
	
	while (ros::ok())
	{

		s.run_true();
		s.publish_t();

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;

}
