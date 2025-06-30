#ifndef RIGIDBODYSIMULATOR_HPP
#define RIGIDBODYSIMULATOR_HPP

#include "Vector3f.hpp"

class RigidBodySimulator
{
	private:
		float		pitch;
		float		yawl;
		float		roll;
		float		dt;
		Vector3f	omega;
		Vector3f	inertia;

	public:
		RigidBodySimulator();
		~RigidBodySimulator();
};

#endif
