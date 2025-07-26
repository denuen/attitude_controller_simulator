#ifndef VECTOR3F_HPP
#define VECTOR3F_HPP

// Represents a 3D vector with single-precision floating-point components.
class Vector3f {
	private:
		float	x;
		float	y;
		float	z;

	public:
		// Constructs a vector with given x, y, z values (defaults to zero).
		Vector3f(const float x = 0.0f, const float y = 0.0f, const float z = 0.0f);

		// Copy constructor.
		Vector3f(const Vector3f& vector);

		// Assignment operator.
		Vector3f&		operator=(const Vector3f& vector);

		// Vector addition.
		Vector3f		operator+(const Vector3f& vector) const;

		// Vector subtraction.
		Vector3f		operator-(const Vector3f& vector) const;

		// Scalar multiplication.
		Vector3f		operator*(float scalar) const;

		bool			operator==(const Vector3f& vector) const;

		inline bool		operator!=(const Vector3f& vector) const { return (!((*this) == vector));}

		// Sets all three components.
		void			setVariables(const float x, const float y, const float z);

		// Setters for each individual component.
		void			setX(const float x);
		void			setY(const float y);
		void			setZ(const float z);

		// Getters for each individual component.
		inline float	getX(void) const { return (x); }
		inline float	getY(void) const { return (y); }
		inline float	getZ(void) const { return (z); }

		// Validates that all components are finite and not NaN.
		void			assertVectorCheck();

		// Destructor.
		~Vector3f();
};

// Computes the dot product of two vectors.
float		dot(const Vector3f& vectorA, const Vector3f& vectorB);

// Computes the cross product of two vectors.
Vector3f	cross(const Vector3f& vectorA, const Vector3f& vectorB);

// Performs element-wise multiplication of two vectors.
Vector3f	componentMultiply(const Vector3f& vectorA, const Vector3f& vectorB);

#endif
