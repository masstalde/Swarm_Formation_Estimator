#include "kalman_consensus.h"
#include "kalman_consensus_short.h"
#include "kalman_consensus_long.h"

// Switch between the two to communicate full estimate or only measurements
using namespace kalman_consensus_short_ns;
//using namespace kalman_consensus_long_ns;
//using namespace kalman_consensus_ns;

int main(int argc, char **argv) {
  ros::init(argc, argv, "agent2");
  ros::NodeHandle n;
	ros::Rate loop_rate(20);

	SingleSystem s;
	s.init(2);

	while (ros::ok()) {
		ros::spinOnce();
		s.run();
		loop_rate.sleep();
	}

  return 0;
}
