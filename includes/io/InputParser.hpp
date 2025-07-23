#ifndef INPUTPARSER_HPP
#define INPUTPARSER_HPP

#include "../physics/Vector3f.hpp"
#include <vector>
#include <string>
#include <tinyxml.h>

class InputParser {

	private:
		Vector3f								kp;
		Vector3f								ki;
		Vector3f								kd;
		Vector3f								inertia;
		Vector3f								driftRate;
		Vector3f								noiseStdDev;
		float									actuatorDelay;

		float									lastTime;

		std::vector<std::pair<float, Vector3f> >	setpoints;

		void	parseVector3f(const std::string& line, Vector3f& out);
		void	parseFloat(const std::string& line, float& out);
		void	parseSetpointLine(const std::string& line);

		void	parseVector3f(TiXmlElement* element, Vector3f& out);
		void	parseFloat(TiXmlElement* element, float& out);
		void	parseSetpointLine(TiXmlElement* element);

	public:
		InputParser();
		InputParser(const InputParser& inputParser);

		InputParser&			 operator=(const InputParser& inputParser);

		inline const Vector3f&	getInertia(void) const { return (inertia); }
		inline const Vector3f&	getKp(void) const { return (kp); }
		inline const Vector3f&	getKi(void) const { return (ki); }
		inline const Vector3f&	getKd(void) const { return (kd); }
		inline const Vector3f&	getNoiseStdDev(void) const { return (noiseStdDev); }
		inline const Vector3f&	getDriftRate(void) const { return (driftRate); }
		inline float			getActuatorDelay(void) const { return (actuatorDelay); }

		// Returns the next setpoint available at or before given time t
		Vector3f				getSetpointAt(float time) const;

		void					loadConfigFromXML(const std::string& filename);
		void					loadConfigFromTXT(const std::string& filename);

		void					loadConfigFile(const std::string& filename);

		void					loadSetpointsFile(const std::string& filename);

		void					reset();

		~InputParser();
};

#endif
