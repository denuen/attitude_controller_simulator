#ifndef RIGID_BODY_SIMULATOR_HPP
#define RIGID_BODY_SIMULATOR_HPP

#include "../../includes/physics/Vector3f.hpp"

class RigidBodySimulator {

  private:
	float		pitch_; // Current pitch angle (rad)
	float		yaw_; // Current yaw angle (rad)
	float		roll_; // Current roll angle (rad)
	float		dt_; // Last simulation time step (s)

	Vector3f	omega_; // Body angular velocity vector (rad/s)
	Vector3f	inertia_; // Principal moments of inertia (Ixx,Iyy,Izz)
	Vector3f	inverseInertia_; // Cached inverse of inertia (1/Ixx,1/Iyy,1/Izz)

	// Recompute inverseInertia_ from inertia_
	void	computeInverseInertia();

	// Wrap Euler angles to [-pi, pi]
	void	normalizeAngles();

	// Integrate omega
	void	updateAngularVelocity(float dt, const Vector3f& torque);

	// Integrate Euler angles from omega_
	void	updateEulerAngles(float dt);

  public:
	// Constructs a rigid body simulator with zero angles, unit inertia
	RigidBodySimulator();

	// Constructs with specified inertia vector (Ixx,Iyy,Izz)
	explicit RigidBodySimulator(const Vector3f& inverseInertia);

	// Copy constructor
	RigidBodySimulator(const RigidBodySimulator& rbs);

	// Copy assignment operator
	RigidBodySimulator&	operator=(const RigidBodySimulator& rbs);

	// Sets pitch angle (rad) with numeric validation
	void				setPitch(float pitch);

	// Sets yaw angle (rad) with numeric validation
	void				setYaw(float yaw);

	// Sets roll angle (rad) with numeric validation
	void				setRoll(float roll);

	// Sets body angular velocity (rad/s)
	inline void			setOmega(const Vector3f& omega) { omega_ = omega; }

	// Sets principal moments of inertia and recompute inverse
	inline void			setInertia(const Vector3f& inertia);

	// Returns current pitch (rad)
	inline float		getPitch() const { return (pitch_); }

	// Returns current yaw (rad)
	inline float		getYaw() const { return (yaw_); }

	// Returns current roll (rad)
	inline float		getRoll() const { return (roll_); }

	// Returns last simulation time step (s)
	inline float		getDt() const { return (dt_); }

	// Returns current angular velocity vector (rad/s)
	inline Vector3f		getOmega() const { return (omega_); }

	// Returns principal moments of inertia (Ixx,Iyy,Izz)
	inline Vector3f		getInertia() const { return (inertia_); }

	// Returns cached inverse inertia (1/Ixx,1/Iyy,1/Izz)
	inline Vector3f		getInverseInertia() const { return (inverseInertia_); }

	// Advance simulation by dt with applied body-frame torque (N·m)
	void				update(float dt, const Vector3f& torque);

	// Validate internal numerics (finite, non-NaN, positive inertia)
	bool				checkNumerics() const;

	// Default destructor
	~RigidBodySimulator();
};

#endif
