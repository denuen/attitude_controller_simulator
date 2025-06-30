#ifndef RIGIDBODYSIMULATOR_HPP
#define RIGIDBODYSIMULATOR_HPP

#include "Vector3f.hpp"
#include <cassert>

class RigidBodySimulator
{
	private:
		float		pitch;
		float		yaw;
		float		roll;
		float		dt;
		Vector3f	omega;
		Vector3f	inertia;
		Vector3f	inverseInertia;

		void		compute_inverse_inertia();
		void		normalize_angles();

	public:
		RigidBodySimulator();
		explicit RigidBodySimulator(const Vector3f& inertia);
		RigidBodySimulator(const RigidBodySimulator& rbSim);

		RigidBodySimulator& 			operator=(const RigidBodySimulator& rbSim);

		void							setPitch(const float pitch);
		void							setYaw(const float yaw);
		void							setRoll(const float roll);
		void							setOmega(const Vector3f& omega);
		void							setInertia(const Vector3f& inertia);

		inline const float&				getPitch(void) const { return (pitch); }
		inline const float&				getYaw(void) const { return (yaw); }
		inline const float&				getRoll(void) const { return (roll); }
		inline const float&				getDt(void) const { return (dt); }
		inline const Vector3f&			getOmega(void) const { return (omega); }
		inline const Vector3f&			getInertia(void) const { return (inertia); }
		inline const Vector3f&			getInverseInertia(void) const { return (inverseInertia); }

		void							update(float dt, const Vector3f& torque);

		~RigidBodySimulator();
};

#endif
