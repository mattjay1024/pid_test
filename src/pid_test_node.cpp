#include <ros/ros.h>
#include <ros/console.h>

#include "pid_test/pid.h"

#include <array>
#include <cmath>

#define PI 3.14159

bool rough_equals(double x, double y, double threshold) { return (std::abs(x - y) < threshold); };

int main(int argc, char** argv) {
	ros::init(argc, argv, "pid_test_node");

	ros::NodeHandle node;

	std::array<double, 5> targets = {9.516, 12.153, 3 * PI, 51.582, 86.75309};
	double output_value = 0.0, last_output, update;

	ros::Rate r(5);
	int target = 0;
	
	PID controller;

	while(ros::ok() && target < targets.size()) {
		update = controller.update(output_value, PID::terms_t::P | PID::terms_t::I);
		last_output = output_value;
		output_value += update;

		ROS_INFO("Target: %f | Actual: %f | Update: %f", targets[target], last_output, update);
		
		if(rough_equals(output_value, targets[target], 0.005)) target++;
		
		r.sleep();
	}

	return 0;
}
