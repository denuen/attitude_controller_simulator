#ifndef VECTOR3F_HPP
#define VECTOR3F_HPP

class	Vector3f {
	private:
		float	x;
		float	y;
		float	z;

	public:
		Vector3f(const float x = 0.0f, const float y = 0.0f, const float z = 0.0f);
		Vector3f(const Vector3f& vector);

		Vector3f&			 operator=(const Vector3f& vector);

		void				setVariables(const float x, const float y, const float z);
		inline const Vector3f&	getVector(void) const { return (*this); }

		void				setX(const float x);
		void				setY(const float y);
		void				setZ(const float z);
		inline float		getX(void) { return (x); }
		inline float		getY(void) { return (y); }
		inline float		getZ(void) { return (z); }

		~Vector3f();
};

float		dot(const Vector3f& vectorA, const Vector3f& vectorB);
Vector3f	cross(const Vector3f& vectorA, const Vector3f& vectorB);
#endif
