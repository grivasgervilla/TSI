#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Controller {
public:
	const static double FORWARD_SPEED_MPS = 0.2;
	const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
	const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
	const static float MIN_PROXIMITY_RANGE_M = 0.5;

	Controller();
	void startMoving();

private:
	ros::NodeHandle node;
	ros::Publisher command_pub;
	ros::Subscriber laser_sub;
	bool keep_moving;
	
	void moveFoward();
	void turn();
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan);
};
