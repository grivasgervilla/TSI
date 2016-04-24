#include "Controller.h"

int main (int argc, char **argv) {
	ros::init(argc, argv, "controller");

	Controller controller;

	controller.startMoving();

	return 0;
}
