#ifndef INPUTPARSER_HPP
#define INPUTPARSER_HPP

#include "../physics/Vector3f.hpp"
#include <string>
#include <tinyxml.h>
#include <vector>

// Parses configuration files and setpoint data for the attitude control system
class InputParser {

	private:
		Vector3f	kp_;		 // Proportional gains for PID controller (x, y, z axes)
		Vector3f	ki_;		 // Integral gains for PID controller (x, y, z axes)
		Vector3f	kd_;		 // Derivative gains for PID controller (x, y, z axes)
		Vector3f	inertia_;	 // Moment of inertia tensor diagonal (Ixx, Iyy, Izz)
		Vector3f	driftRate_; // Sensor drift rates (deg/s for each axis)
		Vector3f	noiseStdDev_;	 // Gaussian noise standard deviation for sensors
		float		actuatorDelay_; // Actuator response delay in seconds

		float		lastTime_; // Last processed time for setpoint interpolation

		std::vector<std::pair<float, Vector3f> > setpoints_; // Time-ordered setpoint trajectory (time, attitude)

		// Parses Vector3f from text line format
		void	parseVector3f(const std::string& line, Vector3f& out);

		// Parses float value from text line format
		void	parseFloat(const std::string& line, float& out);

		// Parses setpoint entry from text line format
		void	parseSetpointLine(const std::string& line);

		// Parses Vector3f from XML element
		void	parseVector3f(TiXmlElement* element, Vector3f& out);

		// Parses float value from XML element
		void	parseFloat(TiXmlElement* element, float& out);

		// Parses setpoint entry from XML element
		void	parseSetpointLine(TiXmlElement* element);

	public:
		// Default constructor - initializes all parameters to zero
		InputParser();

		// Copy constructor
		InputParser(const InputParser& inputParser);

		// Copy assignment operator
		InputParser&	operator=(const InputParser& inputParser);

		// Returns the moment of inertia tensor diagonal
		inline Vector3f	getInertia(void) const { return (inertia_); }

		// Returns the proportional gains for PID controller
		inline Vector3f	getKp(void) const { return (kp_); }

		// Returns the integral gains for PID controller
		inline Vector3f	getKi(void) const { return (ki_); }

		// Returns the derivative gains for PID controller
		inline Vector3f	getKd(void) const { return (kd_); }

		// Returns the sensor noise standard deviation
		inline Vector3f	getNoiseStdDev(void) const { return (noiseStdDev_); }

		// Returns the sensor drift rates
		inline Vector3f	getDriftRate(void) const { return (driftRate_); }

		// Returns the actuator delay time
		inline float	getActuatorDelay(void) const { return (actuatorDelay_); }

		// Returns the setpoint available at or before given time t
		Vector3f		getSetpointAt(float time) const;

		// Loads configuration parameters from XML file
		void			loadConfigFromXML(const std::string& filename);

		// Loads configuration parameters from text file
		void			loadConfigFromTXT(const std::string& filename);

		// Automatically detects file format and loads configuration
		void			loadConfigFile(const std::string& filename);

		// Loads setpoint trajectory from file
		void			loadSetpointsFile(const std::string& filename);

		// Validates that all parameters are finite and positive where required
		bool			checkNumerics() const;

		// Resets all parameters to default values and clears setpoints
		void			reset();

		~InputParser();
};

#endif
