#include "../../includes/io/InputParser.hpp"
#include <cassert>
#include <sstream>

InputParser::InputParser(void) :
kp(), ki(), kd(), inertia(), driftRate(),
noiseStdDev(), actuatorDelay(0.0f), setpoints(){

}

InputParser::InputParser(const InputParser& inputParser) :
kp(inputParser.kp), ki(inputParser.ki), kd(inputParser.kd),
inertia(inputParser.inertia), driftRate(inputParser.driftRate),
noiseStdDev(inputParser.noiseStdDev), actuatorDelay(inputParser.actuatorDelay),
setpoints(inputParser.setpoints) {

}

InputParser& InputParser::operator=(const InputParser& inputParser) {

	if (this != &inputParser) {
		kp = inputParser.kp;
		ki = inputParser.ki;
		kd = inputParser.kd;
		inertia = inputParser.inertia;
		driftRate = inputParser.driftRate;
		noiseStdDev = inputParser.noiseStdDev;
		actuatorDelay = inputParser.actuatorDelay;
		setpoints = inputParser.setpoints;
	}

	return (*this);
}

void	InputParser::parseVector3f(const std::string& line, Vector3f& out) {

	std::istringstream	iss(line);
	float				components[3];

	for (int i = 0; i < 3; i++) {
		if (!(iss >> components[i])) {
			assert(false && "Error: InputParser: parseVector3f: expected 3 float components");
		}
	}

	out.setX(components[0]);
	out.setY(components[1]);
	out.setZ(components[2]);

	std::string	rest;
	if (iss >> rest) {
		assert(false && "Error: InputParser: parseVector3f: too many components in line");
	}
}
