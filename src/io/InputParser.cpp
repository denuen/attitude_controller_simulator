#include "../../includes/io/InputParser.hpp"
#include <cassert>
#include <sstream>
#include <fstream>
#include <cmath>

InputParser::InputParser(void) :
kp_(), ki_(), kd_(), inertia_(), driftRate_(),
noiseStdDev_(), actuatorDelay_(0.0f), lastTime_(-1.0f),
setpoints_() {

}

InputParser::InputParser(const InputParser& inputParser) :
kp_(inputParser.kp_), ki_(inputParser.ki_), kd_(inputParser.kd_),
inertia_(inputParser.inertia_), driftRate_(inputParser.driftRate_),
noiseStdDev_(inputParser.noiseStdDev_), actuatorDelay_(inputParser.actuatorDelay_),
lastTime_(inputParser.lastTime_), setpoints_(inputParser.setpoints_) {

}

InputParser&	InputParser::operator=(const InputParser& inputParser) {

	if (this != &inputParser) {
		kp_ = inputParser.kp_;
		ki_ = inputParser.ki_;
		kd_ = inputParser.kd_;
		inertia_ = inputParser.inertia_;
		driftRate_ = inputParser.driftRate_;
		noiseStdDev_ = inputParser.noiseStdDev_;
		actuatorDelay_ = inputParser.actuatorDelay_;
		lastTime_ = inputParser.lastTime_;
		setpoints_ = inputParser.setpoints_;
	}

	return (*this);
}

void	InputParser::parseVector3f(const std::string& line, Vector3f& out) {

	std::istringstream	iss(line);
	float				components[3];

	for (int i = 0; i < 3; i++) {
		if (!(iss >> components[i])) {
			assert(false && "Error: expected 3 float components");
		}
	}

	out.setX(components[0]);
	out.setY(components[1]);
	out.setZ(components[2]);

	std::string	rest;
	if (iss >> rest) {
		assert(false && "Error: too many components in line");
	}

}

void	InputParser::parseVector3f(TiXmlElement* element, Vector3f& out) {

	const char*	xAttr;
	const char*	yAttr;
	const char*	zAttr;

	assert(element && "Error: null XML element");

	xAttr = element->Attribute("x");
	yAttr = element->Attribute("y");
	zAttr = element->Attribute("z");

	assert(xAttr && yAttr && zAttr && "Error: missing x/y/z attributes");

	out.setX(std::atof(xAttr));
	out.setY(std::atof(yAttr));
	out.setZ(std::atof(zAttr));

}

void	InputParser::parseFloat(const std::string& line, float& out) {

	std::istringstream	iss(line);

	if (!(iss >> out)) {
		assert(false && "Error: expected a float component");
	}

	std::string	rest;
	if (iss >> rest) {
		assert(false && "Error: too many components in line");
	}

}

void	InputParser::parseFloat(TiXmlElement* element, float& out) {

	const char*	value;

	assert(element && "Error: null XML element");

	value = element->GetText();
	assert(value && "Error: empty element");

	out = std::atof(value);

}

void	InputParser::parseSetpointLine(const std::string& line) {

	std::istringstream	iss(line);
	float				components[4];

	for (int i = 0; i < 4; i++) {
		if (!(iss >> components[i])) {
			assert(false && "Error: expected 4 float components");
		}
	}

	assert(components[0] > lastTime_ && "Error: setPoint time must be monotonically increasing");

	setpoints_.push_back(std::make_pair(components[0],
		Vector3f(components[1], components[2], components[3])));
	lastTime_ = components[0];

	std::string	rest;
	if (iss >> rest) {
		assert(false && "Error: too many components in line");
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

	assert(element && "Error: null element");

	timeStr = element->Attribute("time");
	rollStr = element->Attribute("roll");
	pitchStr = element->Attribute("pitch");
	yawStr = element->Attribute("yaw");

	assert(timeStr && rollStr && pitchStr && yawStr
		&& "Error: missing attribute(s)");

	time = std::atof(timeStr);
	assert(time > lastTime_ && "Error: setPoint time must be monotonically increasing");

	roll = std::atof(rollStr);
	pitch = std::atof(pitchStr);
	yaw = std::atof(yawStr);

	setpoints_.push_back(std::make_pair(time, Vector3f(roll, pitch, yaw)));
	lastTime_ = time;

}

Vector3f	InputParser::getSetpointAt(float time) const {

	Vector3f	res;

	for (std::vector<std::pair<float, Vector3f> >::const_iterator i = setpoints_.begin(); i != setpoints_.end(); ++i) {
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

	input.open(const_cast<char*>(filename.c_str()));
	assert(input.is_open() && "Error: loadConfigFromTXT: invalid file");

	reset();

	std::getline(input, line);
	parseVector3f(line, kp_);

	std::getline(input, line);
	parseVector3f(line, ki_);

	std::getline(input, line);
	parseVector3f(line, kd_);

	std::getline(input, line);
	parseVector3f(line, inertia_);

	std::getline(input, line);
	parseVector3f(line, driftRate_);

	std::getline(input, line);
	parseVector3f(line, noiseStdDev_);

	std::getline(input, line);
	parseFloat(line, actuatorDelay_);

	while (std::getline(input, line)) {
		parseSetpointLine(line);
	}

	if (std::getline(input, line)) {
		assert(false && "Error: loadConfigFromTXT: unexpected extra content in file");
	}

	assert(input.eof() && "Error: loadConfigFromTXT: file read incomplete");

}

void	InputParser::loadConfigFromXML(const std::string& filename) {

	std::ifstream	input;
	std::string		line;
	TiXmlDocument	xmlDocument(filename.c_str());

	if (!xmlDocument.LoadFile()) {
		assert(false && "Error: Cannot load XML file");
		std::cerr << "Error: Cannot load XML file '" << filename << "'"
				<< std::endl;
		std::cerr << "TinyXML Error: " << xmlDocument.ErrorDesc() << std::endl;
		return;
	}

	TiXmlElement* root = xmlDocument.FirstChildElement("AttitudeControllerConfig");
	if (!root) {
		assert(false && "Error: invalid XML structure");
		std::cerr << "Error: Invalid XML structure - missing "
					 "'AttitudeControllerConfig' root element"
				  << std::endl;
		return;
	}

	TiXmlElement*	tmp = root->FirstChildElement("ControllerGains");
	assert (tmp && "Error: missing ControllerGains section in XML file");

	reset ();

	parseVector3f(tmp->FirstChildElement("Kp"), kp_);
	parseVector3f(tmp->FirstChildElement("Ki"), ki_);
	parseVector3f(tmp->FirstChildElement("Kd"), kd_);

	tmp = root->FirstChildElement("PhysicalProperties");
	assert (tmp && "Error: missing PhysicalProperties section in XML file");

	parseVector3f(tmp->FirstChildElement("Inertia"), inertia_);

	tmp = root->FirstChildElement("SensorCharacteristics");
	assert (tmp && "Error: missing SensorCharacteristics section in XML file");

	parseVector3f(tmp->FirstChildElement("DriftRate"), driftRate_);
	parseVector3f(tmp->FirstChildElement("NoiseStdDev"), noiseStdDev_);

	tmp = root->FirstChildElement("ActuatorProperties");
	assert (tmp && "Error: missing ActuatorProperties section in XML file");

	parseFloat(tmp->FirstChildElement("Delay"), actuatorDelay_);

	tmp = root->FirstChildElement("SetpointSequence");
	assert(tmp && "Error: missing setpoints_equence section");

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
		assert(false && "Error: invalid extension");
	}

}

void	InputParser::reset(void) {

	kp_ = ki_ = kd_ = inertia_ = driftRate_ = noiseStdDev_ = Vector3f(0.0f, 0.0f, 0.0f);
	actuatorDelay_ = 0.0f;
	lastTime_ = -1.0f;
	setpoints_.clear();

}

bool	InputParser::checkNumerics() const {

	if (!kp_.checkNumerics() || !ki_.checkNumerics() || !kd_.checkNumerics()
		|| !inertia_.checkNumerics() || !driftRate_.checkNumerics() || !noiseStdDev_.checkNumerics()
		|| std::isnan(actuatorDelay_) || std::isinf(actuatorDelay_) || actuatorDelay_ < 0.0f
		|| std::isnan(lastTime_) || std::isinf(lastTime_)
		|| inertia_.getX() <= 0.0f || inertia_.getY() <= 0.0f || inertia_.getZ() <= 0.0f) {
			return (false);
	}

	for (std::vector<std::pair<float, Vector3f> >::const_iterator it = setpoints_.begin();
		 it != setpoints_.end(); ++it) {
		if (std::isnan(it->first) || std::isinf(it->first) || !it->second.checkNumerics()) {
			return (false);
		}
	}

	return (true);
}

InputParser::~InputParser(void) {

}
