#ifndef RIGIDBODYSIMULATOR_HPP
#define RIGIDBODYSIMULATOR_HPP

#include "../physics/Vector3f.hpp"
#include <cassert>

// Simulates rotational dynamics of a rigid body using Euler's rotational equations with gyroscopic coupling.
class RigidBodySimulator
{
	private:
		float		pitch;			// Current pitch angle in radians.
		float		yaw;			// Current yaw angle in radians.
		float		roll;			// Current roll angle in radians.
		float		dt;				// Last used time step in seconds.

		Vector3f	omega;			// Angular velocity vector (rad/s).
		Vector3f	inertia;		// Principal moments of inertia (Ixx, Iyy, Izz).
		Vector3f	inverseInertia;	// Precomputed inverse of inertia.

		// Computes the inverse of the inertia vector.
		void		compute_inverse_inertia();

		// Wraps pitch, yaw, and roll to the [-π, π] range.
		void		normalize_angles();

	public:
		// Constructs a rigid body with unit inertia and zero initial state.
		RigidBodySimulator();

		// Constructs a rigid body with specified principal inertia.
		explicit RigidBodySimulator(const Vector3f& inertia);

		// Copy constructor.
		RigidBodySimulator(const RigidBodySimulator& rbSim);

		// Copy assignment operator.
		RigidBodySimulator&		operator=(const RigidBodySimulator& rbSim);

		// Sets the pitch angle in radians.
		void					setPitch(const float pitch);

		// Sets the yaw angle in radians.
		void					setYaw(const float yaw);

		// Sets the roll angle in radians.
		void					setRoll(const float roll);

		// Sets the angular velocity vector.
		void					setOmega(const Vector3f& omega);

		// Sets the inertia vector and recomputes its inverse.
		void					setInertia(const Vector3f& inertia);

		// Returns the current pitch angle.
		inline float			getPitch(void) const { return (pitch); }

		// Returns the current yaw angle.
		inline float			getYaw(void) const { return (yaw); }

		// Returns the current roll angle.
		inline float			getRoll(void) const { return (roll); }

		// Returns the last time step used in update.
		inline float			getDt(void) const { return (dt); }

		// Returns the current angular velocity vector.
		inline const Vector3f&	getOmega(void) const { return (omega); }

		// Returns the current inertia vector.
		inline const Vector3f&	getInertia(void) const { return (inertia); }

		// Returns the inverse inertia vector.
		inline const Vector3f&	getInverseInertia(void) const { return (inverseInertia); }

		bool					checkNumerics(void) const;
		
		// Integrates angular dynamics using Euler's rotational equations including gyroscopic effects.
		void					update(float dt, const Vector3f& torque);

		// Destructor.
		~RigidBodySimulator();
};

#endif
