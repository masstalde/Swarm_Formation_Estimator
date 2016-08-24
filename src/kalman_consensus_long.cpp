#include "kalman_consensus_long.h"

using namespace kalman_consensus_long_ns;



bool SingleSystem::init(int ag) {

	ROS_INFO("\nBeginning initialization\n_________________________\n");

	closed_ = false;
	
	ROS_INFO("Setting up publishers/subscribers...\n");

	agent = ag;
	std::stringstream ss_pub;
	std::stringstream ss_z;
	std::stringstream ss_t_f;
	std::stringstream ss_t_s;
	f = agent%3+1;
	s = (agent+1)%3+1;

	ss_pub << "t" << agent << "vec";
	ss_z << "z" << agent << "vec";
	ss_t_f << "t" << f << "vec";
	ss_t_s << "t" << s << "vec";


	std::string pub_str = ss_pub.str(); 
	std::string z_str = ss_z.str(); 
	std::string t_f_str = ss_t_f.str(); 
	std::string t_s_str = ss_t_s.str();


	ROS_INFO("**%s **\n",z_str.c_str());
	ROS_INFO("**%s **\n",t_f_str.c_str());
	ROS_INFO("**%s **\n",t_s_str.c_str());
	ROS_INFO("**%s **\n",pub_str.c_str());

	z_sub = n.subscribe(z_str, 1000, &SingleSystem::zCallback, this);
	t_sub_first = n.subscribe(t_f_str, 1000, &SingleSystem::tfCallback, this);
	t_sub_second = n.subscribe(t_s_str, 1000, &SingleSystem::tsCallback, this);
	t_sub_ref = n.subscribe("trvec", 1000, &SingleSystem::trCallback, this);
	pub = n.advertise<std_msgs::Float32MultiArray>(pub_str, 100);

	ROS_INFO("...complete.\n");

	ROS_INFO("Setting up fixed length vectors...\n");
 
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

		tf_(i)=0;
		ts_(i)=0;
		n_(i)=0;
	
		z_(i) = 0;
		w_(i) = 0;
		
		i++;
	}

	while (i<ComDim) {
		x_pred_(i) = 0;
		x_(i) = 1;
		v_(i) =0;
		r_(i) = 0;

		tf_(i)=0;
		ts_(i)=0;
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

	ROS_INFO("...complete.\n");

	ROS_INFO("Setting up system matrices...\n");


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

	ROS_INFO("...complete.\n");

	ROS_INFO("Setting up measurement and communication vectors/matrices...\n");

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

	ROS_INFO("...complete.\n");

	ROS_INFO("Setting up augmented placeholders...\n");

	C_r_1_ << C_[0],
						H_[f-1];

	z_r_1_ << z_,
						tf_;

	C_r_2_ << H_[f-1], 
				 	  H_[s-1];

	z_r_2_ << tf_,
						ts_;

	C_r_3_ << C_[0], 
				 	  H_[f-1], 
				 	  H_[s-1];

	z_r_3_ << z_,
						tf_,
						ts_;

	R_r_1_ = Eigen::MatrixXd::Zero(MixDim,MixDim);
	R_r_2_ = Eigen::MatrixXd::Zero(AllComDim,AllComDim);
	R_r_3_ = Eigen::MatrixXd::Zero(EstDim,EstDim);
	
	R_r_1_.block(0,0,ObsDim,ObsDim) = R_[0];
	R_r_2_.block(0,0,ComDim,ComDim) = N_[f-1];
	R_r_3_.block(0,0,ObsDim,ObsDim) = R_[0];

	R_r_1_.block(ObsDim,ObsDim,ComDim,ComDim) = N_[f-1];
	R_r_2_.block(ComDim,ComDim,ComDim,ComDim) = N_[s-1];
	R_r_3_.block(ObsDim,ObsDim,ComDim,ComDim) = N_[f-1];
	R_r_3_.block(MixDim,MixDim,ComDim,ComDim) = N_[s-1];

	ROS_INFO("...complete.\n");

	last=ros::Time::now().toSec();

	ROS_INFO("\n_________________________\nInitialization complete.\n");
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

void SingleSystem::predict() {

	if (closed_) {

		x_pred_ = A_cl_*x_;
		P_pred_ = A_cl_*P_*A_cl_.transpose() + Q_;
	}

	else {

		double current =ros::Time::now().toSec();
		double dt = current-last;
		last=current;
		SSMatrix A= Eigen::Matrix<double, StateDim, StateDim>::Identity() + A_cl_*dt;
		ROS_INFO("**%f **\n",dt);

		x_pred_ = A*x_ + NB_*r_*dt;
		P_pred_ = A*P_*A.transpose() + Q_;
	}


}

void SingleSystem::correct() {

	switch (config) {
		case 1:
			{
				OOMatrix M = C_[avail[0]-1]*P_pred_*C_[avail[0]-1].transpose()+R_[avail[0]-1];
				SOMatrix L_r = P_pred_*C_[avail[0]-1].transpose()*M.inverse();
				x_=x_pred_ + L_r*(z_-C_[avail[0]-1]*x_pred_);
				P_=P_pred_ - L_r*C_[avail[0]-1]*P_pred_;
				break;
			}

		case 2:
			{
				MMMatrix M = C_r_1_*P_pred_*C_r_1_.transpose()+R_r_1_;
				SMMatrix L_r = P_pred_*C_r_1_.transpose()*M.inverse();
				x_=x_pred_ + L_r*(z_r_1_-C_r_1_*x_pred_);
				P_=P_pred_ - L_r*C_r_1_*P_pred_;
				break;
			}

		case 3:
			{
				EEMatrix M = C_r_3_*P_pred_*C_r_3_.transpose()+R_r_3_;
				SEMatrix L_r = P_pred_*C_r_3_.transpose()*M.inverse();
				x_=x_pred_ + L_r*(z_r_3_-C_r_3_*x_pred_);
				P_=P_pred_ - L_r*C_r_3_*P_pred_;
				break;
			}

		case 4:
			{
				CCMatrix M = H_[avail[1]-1]*P_pred_*H_[avail[1]-1].transpose()+N_[avail[1]-1];
				SCMatrix L_r = P_pred_*H_[avail[1]-1].transpose()*M.inverse();
				x_=x_pred_ + L_r*(tf_-H_[avail[1]-1]*x_pred_);
				P_=P_pred_ - L_r*H_[avail[1]-1]*P_pred_;
				break;
			}

		case 5:
			{
				CCMatrix M = H_[avail[2]-1]*P_pred_*H_[avail[2]-1].transpose()+N_[avail[2]-1];
				SCMatrix L_r = P_pred_*H_[avail[2]-1].transpose()*M.inverse();
				x_=x_pred_ + L_r*(ts_-H_[avail[2]-1]*x_pred_);
				P_=P_pred_ - L_r*H_[avail[2]-1]*P_pred_;
				break;
			}

		case 6:
			{
				AAMatrix M = C_r_2_*P_pred_*C_r_2_.transpose()+R_r_2_;
				SAMatrix L_r = P_pred_*C_r_2_.transpose()*M.inverse();
				x_=x_pred_ + L_r*(z_r_2_-C_r_2_*x_pred_);
				P_=P_pred_ - L_r*C_r_2_*P_pred_;
				break;
			}
	}

}
	

void SingleSystem::perform_iteration() {
	predict();
	if (avail[0]&&avail[1]&&avail[2]) {
		correct();
	}

	else {
		x_=x_pred_;
	}

	avail[0]=0;
	avail[1]=0;
	avail[2]=0;
}

double* SingleSystem::get_x_arr() {
	double x_arr[StateDim];	
	double *x_p = x_.data();
	for (int i=0; i<StateDim; i++) {
		x_arr[i]=x_p[i];
	}

	x_p=x_arr;
	return x_p;
}

void SingleSystem::publish_t() {
	std_msgs::Float32MultiArray x_out;
	int r,c;

	for (int i = 0; i < StateDim; i++) {	
		x_out.data.push_back(x_(i));
	}

	x_out.data.push_back(agent);

	for (r=0; r<StateDim; r++) {
		for (c=0; c<StateDim; c++) {
			x_out.data.push_back(P_(r,c));
			}
	}
	
	pub.publish(x_out);
	print_stuff();
	x_out.data.clear();

}

void SingleSystem::zCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

	int i=0;
	while (i<ObsDim) {
		z_(i)=msg->data[i];
		i++;
	}

	avail[0]=msg->data[i];
}

void SingleSystem::tfCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	int i=0, r, c;
	while (i<ComDim) {
		tf_(i)=msg->data[i];
		i++;
	}

	avail[1]=msg->data[i];

	for (r=0; r<StateDim; r++) {
		for (c=0; c<StateDim; c++) {
			N_[f-1](r,c)=msg->data[i];
			i++;
		}
	}

}

void SingleSystem::tsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	int i=0, r, c;
	while (i<ComDim) {
		ts_(i)=msg->data[i];
		i++;
	}

	avail[2]=msg->data[i];

	for (r=0; r<StateDim; r++) {
		for (c=0; c<StateDim; c++) {
			N_[s-1](r,c)=msg->data[i];
			i++;
		}
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

void SingleSystem::print_stuff() {
	std::stringstream ss; 	

	ss << x_(0) << ", "<< x_(1) << ", "<< x_(2) << ", "<< x_(3) << ", "<< x_(4)
		 << ", "<< x_(5) << ", "<< x_(6) << ", "<< x_(7) << ", "<< x_(8) << ", "<< x_(9)
		 << ", "<< x_(10) << ", "<< x_(11);

	std::string str = ss.str(); 
	
	std::stringstream sa; 	

	sa << avail[0] << ", "<< avail[1] << ", "<< avail[2] << ", "
		 << z_r_1_(0) << ", " << z_r_2_(0) << ", " << z_r_3_(0);

	std::string stra = sa.str(); 

	ROS_INFO("X: %s", str.c_str());
	ROS_INFO("Avail: %s", stra.c_str());
}

void SingleSystem::run_true() {
	if (closed_) {
		x_=A_cl_*x_;
	}
}

void SingleSystem::run() {
	update_available();
	perform_iteration();
	publish_t();
}

