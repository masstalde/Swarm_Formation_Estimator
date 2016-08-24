#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <string>
#include <cstring>
#include <Eigen/Geometry>
#include <sstream>


namespace kalman_consensus_long_ns{

enum
{
	N = 3,	
	StateDim = 6*(N-1),
	InDim = 3*N,
	ObsDim = 3,
	ComDim = StateDim,
	AllComDim = (N-1)*ComDim,
	MixDim = ComDim + ObsDim,
	EstDim = ObsDim+AllComDim
};

class SingleSystem
{
	typedef Eigen::Matrix<double, StateDim, StateDim>					SSMatrix;
	typedef Eigen::Matrix<double, StateDim, InDim>					 	SIMatrix;
	typedef Eigen::Matrix<double, InDim, StateDim>					 	ISMatrix;
	typedef Eigen::Matrix<double, ObsDim, StateDim>				 		OSMatrix;
	typedef Eigen::Matrix<double, ComDim, StateDim>				 		CSMatrix;
	typedef Eigen::Matrix<double, AllComDim, StateDim>				ASMatrix;
	typedef Eigen::Matrix<double, MixDim, StateDim>						MSMatrix;
	typedef Eigen::Matrix<double, AllComDim, AllComDim>				AAMatrix;
	typedef Eigen::Matrix<double, ObsDim, ObsDim>							OOMatrix;
	typedef Eigen::Matrix<double, MixDim, MixDim>							MMMatrix;
	typedef Eigen::Matrix<double, ComDim, ComDim>							CCMatrix;
	typedef Eigen::Matrix<double, StateDim, EstDim>				 		SEMatrix;
	typedef Eigen::Matrix<double, StateDim, ObsDim>				 		SOMatrix;
	typedef Eigen::Matrix<double, StateDim, AllComDim>				SAMatrix;
	typedef Eigen::Matrix<double, StateDim, MixDim>						SMMatrix;
	typedef Eigen::Matrix<double, StateDim, ComDim>						SCMatrix;
	typedef Eigen::Matrix<double, EstDim, StateDim>				 		ESMatrix;
	typedef Eigen::Matrix<double, EstDim, EstDim>					 		EEMatrix;
	typedef Eigen::Matrix<double, StateDim, 1>								SVec;
	typedef Eigen::Matrix<double, InDim, 1>										IVec;
	typedef Eigen::Matrix<double, ObsDim, 1>									OVec;
	typedef Eigen::Matrix<double, ComDim, 1>									CVec;
	typedef Eigen::Matrix<double, MixDim, 1>								MVec;
	typedef Eigen::Matrix<double, AllComDim, 1>								AVec;
	typedef Eigen::Matrix<double, EstDim, 1>									EVec;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		//Node handle, publisher and subscriber
		ros::NodeHandle n;

		ros::Subscriber z_sub;
		ros::Subscriber t_sub_first;
		ros::Subscriber t_sub_second;
		ros::Subscriber t_sub_ref;
		ros::Publisher pub;

		//Availability flags
		int avail[N];
		int config;
		int agent, f, s;

		//Time
		double last;

		//Current State and Noise Vectors
		SVec x_; // Current state
		SVec x_pred_; // Predicted state
		SVec v_; // Gaussian Process noise

		SVec r_; // Current reference

		OVec z_; // Current state measurement
		OVec w_; // Measurement noise

		CVec tf_; // Current state communication
		CVec ts_; // Current state communication
		CVec n_; // Measurement noise

		IVec u_; // Input

		MVec z_r_1_; // Augmented measurement and communication
		AVec z_r_2_; // Augmented measurement and communication
		EVec z_r_3_; // Augmented measurement and communication

		//System Matrices
		SSMatrix A_ol_; // Open loop system dynamics
		SSMatrix A_cl_; // Closed loop system dynamics
		SSMatrix P_; // Covariance matrix of state vector estimate
		SSMatrix P_pred_; // Covariance matrix of predicted state vector estimate
		SSMatrix Q_; // Covariance matrix of process noise
		
		SIMatrix B_; // Input matrix
		OSMatrix C_[3]; // Output matrices
		CSMatrix H_[3]; // Agent 1 communication matrix

		ISMatrix K_; // Feedback Gains
		ISMatrix N_bar_;
		SSMatrix NB_;

		OOMatrix R_[3]; // Covariance matrix of measurement noise
		CCMatrix N_[3]; // Covariance matrix of communication noise

		MSMatrix C_r_1_; // Augmented ouptut and communication matrix
		ASMatrix C_r_2_; // Augmented ouptut and communication matrix
		ESMatrix C_r_3_; // Augmented ouptut and communication matrix
		MMMatrix R_r_1_; // Augmented measurement and communication covariance matrix
		AAMatrix R_r_2_; // Augmented measurement and communication covariance matrix
		EEMatrix R_r_3_; // Augmented measurement and communication covariance matrix

		// Open or Closed Loop operation
		bool closed_;


	public:
		bool init(int ag);
		void select_mode(bool clsd);
		void predict();
		void correct();
		void update_available();
		void perform_iteration();
		double* get_x_arr();
		void publish_t();
		void print_stuff();
		void zCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
		void tfCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
		void tsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
		void trCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
		void run_true();
		void run();
};
}
