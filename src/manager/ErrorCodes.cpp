#include "../../includes/manager/ErrorCodes.hpp"

const char*	configErrorCodeToString(ErrorCode code);
const char*	initErrorCodeToString(ErrorCode code);
const char* runtimeErrorCodeToString(ErrorCode code);
const char*	timingErrorCodeToString(ErrorCode code);
const char*	sysErrorCodeToString(ErrorCode code);

const char*	errorCodeToString(ErrorCode code) {

	if (code == ERR_SUCCESS) {
		return ("ERR_SUCCESS");
	}

	int	intCode = static_cast<int>(code);

	if (intCode > 0 && intCode < 100) {
		return (configErrorCodeToString(code));
	} else if (intCode >= 100 && intCode < 200) {
		return (initErrorCodeToString(code));
	} else if (intCode >= 200 && intCode < 300) {
		return (runtimeErrorCodeToString(code));
	} else if (intCode >= 300 && intCode < 400) {
		return (timingErrorCodeToString(code));
	} else if (intCode >= 400 && intCode < 500) {
		return (sysErrorCodeToString(code));
	} else {
		return ("ERR_UNKNOWN");
	}
	
}

const char*	configErrorCodeToString(ErrorCode code) {

	switch (code) {
		case ERR_CNF_FILE_NOT_FOUND:
			return ("ERR_CNF_FILE_NOT_FOUND");
		case ERR_CNF_INVALID_FORMAT:
			return ("ERR_CNF_INVALID_FORMAT");
		case ERR_CNF_MISSING_PARAMETER:
			return ("ERR_CNF_MISSING_PARAMETER");
		case ERR_CNF_OUT_OF_RANGE:
			return ("ERR_CNF_OUT_OF_RANGE");
		default:
			return ("ERR_UNKNOWN");
	}

}

const char*	initErrorCodeToString(ErrorCode code) {

	switch (code) {
		case ERR_INIT_CONTROLLER_FAILED:
			return ("ERR_INIT_CONTROLLER_FAILED");
		case ERR_INIT_DYNAMICS_FAILED:
			return ("ERR_INIT_DYNAMICS_FAILED");
		case ERR_INIT_SENSORS_FAILED:
			return ("ERR_INIT_SENSORS_FAILED");
		case ERR_INIT_ACTUATOR_FAILED:
			return ("ERR_INIT_ACTUATOR_FAILED");
		default:
			return ("ERR_UNKNOWN");
	}

}

const char* runtimeErrorCodeToString(ErrorCode code) {

	switch (code) {
		case ERR_RT_SENSOR_FAILURE:
			return ("ERR_RT_SENSOR_FAILURE");
		case ERR_RT_CONTROL_DIVERGENCE:
			return ("ERR_RT_CONTROL_DIVERGENCE");
		case ERR_RT_ATTITUDE_LIMIT_EXCEEDED:
			return ("ERR_RT_ATTITUDE_LIMIT_EXCEEDED");
		case ERR_RT_CONTROL_SATURATION:
			return ("ERR_RT_CONTROL_SATURATION");
		case ERR_RT_NAN_DETECTED:
			return ("ERR_RT_NAN_DETECTED");
		default:
			return ("ERR_UNKNOWN");
	}

}

const char*	timingErrorCodeToString(ErrorCode code) {

	switch (code) {
		case ERR_T_DEADLINE_MISS:
			return ("ERR_T_DEADLINE_MISS");
		case ERR_T_EXCESSIVE_JITTER:
			return ("ERR_T_EXCESSIVE_JITTER");
		case ERR_T_TIME_REGRESSION:
			return ("ERR_T_TIME_REGRESSION");
		default:
			return ("ERR_UNKNOWN");
	}

}

const char*	sysErrorCodeToString(ErrorCode code) {

	switch (code) {
		case ERR_SYS_MEMORY_ALLOCATION:
			return ("ERR_SYS_MEMORY_ALLOCATION");
		case ERR_SYS_FILE_IO:
			return ("ERR_SYS_FILE_IO");
		case ERR_SYS_INVALID_STATE:
			return ("ERR_SYS_INVALID_STATE");
		default:
			return ("ERR_UNKNOWN");
	}

}
