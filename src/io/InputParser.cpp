#include "../../includes/io/InputParser.hpp"
#include <cassert>
#include <sstream>
#include <fstream>
#include <cmath>

InputParser::InputParser(void) :
kp(), ki(), kd(), inertia(), driftRate(),
noiseStdDev(), actuatorDelay(0.0f), lastTime(-1.0f),
setpoints() {

}

InputParser::InputParser(const InputParser& inputParser) :
kp(inputParser.kp), ki(inputParser.ki), kd(inputParser.kd),
inertia(inputParser.inertia), driftRate(inputParser.driftRate),
noiseStdDev(inputParser.noiseStdDev), actuatorDelay(inputParser.actuatorDelay),
lastTime(inputParser.lastTime), setpoints(inputParser.setpoints) {

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
		lastTime = inputParser.lastTime;
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

void	InputParser::parseVector3f(TiXmlElement* element, Vector3f& out) {

	const char*	xAttr;
	const char*	yAttr;
	const char*	zAttr;

	assert(element && "Error: InputParser: parseVector3f: null XML element");

	xAttr = element->Attribute("x");
	yAttr = element->Attribute("y");
	zAttr = element->Attribute("z");

	assert(xAttr && yAttr && zAttr && "Error: InputParser: parseVector3f: missing x/y/z attributes");

	out.setX(std::atof(xAttr));
	out.setY(std::atof(yAttr));
	out.setZ(std::atof(zAttr));

}

void	InputParser::parseFloat(const std::string& line, float& out) {

	std::istringstream	iss(line);

	if (!(iss >> out)) {
		assert(false && "Error: InputParser: parseFloat: expected a float component");
	}

	std::string	rest;
	if (iss >> rest) {
		assert(false && "Error: InputParser: parseFloat: too many components in line");
	}

}

void	InputParser::parseFloat(TiXmlElement* element, float& out) {

	const char*	value;

	assert(element && "Error: InputParser: parseFloat: null XML element");

	value = element->GetText();
	assert(value && "Error: InputParser: parseFloat: empty element");

	out = std::atof(value);

}

void	InputParser::parseSetpointLine(const std::string& line) {

	std::istringstream	iss(line);
	float				components[4];

	for (int i = 0; i < 4; i++) {
		if (!(iss >> components[i])) {
			assert(false && "Error: InputParser: parseSetpointLine: expected 4 float components");
		}
	}

	assert(components[0] > lastTime && "Error: InputParser: parseSetpointLine: setPoint time must be monotonically increasing");

	setpoints.push_back(std::make_pair(components[0],
		Vector3f(components[1], components[2], components[3])));
	lastTime = components[0];

	std::string	rest;
	if (iss >> rest) {
		assert(false && "Error: InputParser: parseSetpointLine: too many components in line");
	}

}

void	InputParser::parseSetpointLine(TiXmlElement* element) {

	const char*	timeStr;
	const char*	rollStr;
	const char*	pitchStr;
	const char*	yawStr;
	float		time;
	float		roll;
	float		pitch;
	float		yaw;

	assert(element && "Error: InputParser: parseSetpointLine: null element");

	timeStr = element->Attribute("time");
	rollStr = element->Attribute("roll");
	pitchStr = element->Attribute("pitch");
	yawStr = element->Attribute("yaw");

	assert(timeStr && rollStr && pitchStr && yawStr
		&& "Error: InputParser: parseSetpointLine: missing attribute(s)");

	time = std::atof(timeStr);
	assert(time > lastTime && "Error: InputParser: parseSetpointLine: setPoint time must be monotonically increasing");

	roll = std::atof(rollStr);
	pitch = std::atof(pitchStr);
	yaw = std::atof(yawStr);

	setpoints.push_back(std::make_pair(time, Vector3f(roll, pitch, yaw)));
	lastTime = time;

}

Vector3f	InputParser::getSetpointAt(float time) const {

	Vector3f	res;

	for (std::vector<std::pair<float, Vector3f> >::const_iterator i = setpoints.begin(); i != setpoints.end(); ++i) {
		if (i->first > time) {
			break ;
		}
		res = i->second;
	}

	return (res);

}

void	InputParser::loadConfigFromTXT(const std::string& filename) {

	std::ifstream	input;
	std::string		line;

	input.open(filename);
	assert(input.is_open() && "Error: InputParser: loadConfigFromTXT: invalid file");

	reset();

	std::getline(input, line);
	parseVector3f(line, kp);

	std::getline(input, line);
	parseVector3f(line, ki);

	std::getline(input, line);
	parseVector3f(line, kd);

	std::getline(input, line);
	parseVector3f(line, inertia);

	std::getline(input, line);
	parseVector3f(line, driftRate);

	std::getline(input, line);
	parseVector3f(line, noiseStdDev);

	std::getline(input, line);
	parseFloat(line, actuatorDelay);

	while (std::getline(input, line)) {
		parseSetpointLine(line);
	}

	if (std::getline(input, line)) {
		assert(false && "Error: InputParser: loadConfigFromTXT: unexpected extra content in file");
	}

	assert(input.eof() && "Error: InputParser: loadConfigFromTXT: file read incomplete");

}

void	InputParser::loadConfigFromXML(const std::string& filename) {

	std::ifstream	input;
	std::string		line;
	TiXmlDocument	xmlDocument(filename.c_str());

	assert (xmlDocument.LoadFile() && "Error: InputParser: loadConfigFromXML: cannot load XML file");

	TiXmlElement*	root = xmlDocument.FirstChildElement("AttitudeControllerConfig");
	assert (root && "Error: InputParser: loadConfigFromXML: invalid XML structure");

	TiXmlElement*	tmp = root->FirstChildElement("ControllerGains");
	assert (tmp && "Error: InputParser: loadConfigFromXML: missing ControllerGains section in XML file");

	reset ();

	parseVector3f(tmp->FirstChildElement("Kp"), kp);
	parseVector3f(tmp->FirstChildElement("Ki"), ki);
	parseVector3f(tmp->FirstChildElement("Kd"), kd);

	tmp = root->FirstChildElement("PhysicalProperties");
	assert (tmp && "Error: InputParser: loadConfigFromXML: missing PhysicalProperties section in XML file");

	parseVector3f(tmp->FirstChildElement("Inertia"), inertia);

	tmp = root->FirstChildElement("SensorCharacteristics");
	assert (tmp && "Error: InputParser: loadConfigFromXML: missing SensorCharacteristics section in XML file");

	parseVector3f(tmp->FirstChildElement("DriftRate"), driftRate);
	parseVector3f(tmp->FirstChildElement("NoiseStdDev"), noiseStdDev);

	tmp = root->FirstChildElement("ActuatorProperties");
	assert (tmp && "Error: InputParser: loadConfigFromXML: missing ActuatorProperties section in XML file");

	parseFloat(tmp->FirstChildElement("Delay"), actuatorDelay);

	tmp = root->FirstChildElement("SetpointSequence");
	assert(tmp && "Error: InputParser: loadConfigFromXML: missing SetpointSequence section");

	for (TiXmlElement* setpoint = tmp->FirstChildElement("Setpoint");
		setpoint != NULL; setpoint = setpoint->NextSiblingElement("Setpoint")) {
	parseSetpointLine(setpoint);
	}

}

void	InputParser::loadConfigFile(const std::string& filename) {

	std::string	ext;

	ext = filename.substr(filename.find_last_of('.') + 1);

	if (ext == "txt") {
		loadConfigFromTXT(filename);
	} else if (ext == "xml") {
		loadConfigFromXML(filename);
	} else {
		assert(false && "Error: InputParser: loadConfigFile: invalid extension");
	}

}

void	InputParser::reset(void) {

	kp = ki = kd = inertia = driftRate = noiseStdDev = Vector3f(0.0f, 0.0f, 0.0f);
	actuatorDelay = 0.0f;
	lastTime = -1.0f;
	setpoints.clear();

}

bool	InputParser::checkNumerics() const {

	if (!kp.checkNumerics() || !ki.checkNumerics() || !kd.checkNumerics()
		|| !inertia.checkNumerics() || !driftRate.checkNumerics() || !noiseStdDev.checkNumerics()
		|| isnan(actuatorDelay) || isinf(actuatorDelay) || actuatorDelay < 0.0f
		|| isnan(lastTime) || isinf(lastTime)
		|| inertia.getX() <= 0.0f || inertia.getY() <= 0.0f || inertia.getZ() <= 0.0f) {
			return (false);
	}

	for (std::vector<std::pair<float, Vector3f> >::const_iterator it = setpoints.begin();
		 it != setpoints.end(); ++it) {
		if (isnan(it->first) || isinf(it->first) || !it->second.checkNumerics()) {
			return (false);
		}
	}

	return (true);
}

InputParser::~InputParser(void) {

}
