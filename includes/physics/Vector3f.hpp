#ifndef VECTOR3F_HPP
#define VECTOR3F_HPP

// 3D vector object with single-precision floating-point components
class Vector3f {

	private:
		float	x_;
		float	y_;
		float	z_;

	public:
		// Construct a vector with given (x,y,z) components. Defaults to zero
		Vector3f(float x = 0.0f, float y = 0.0f, float z = 0.0f);

		// Copy constructor
		Vector3f(const Vector3f& v);

		// Assignment operator
		Vector3f&		operator=(const Vector3f& v);

		// Addition operator (floating-point secure)
		Vector3f		operator+(const Vector3f& v) const;

		// Addition operator (floating-point secure) 
		Vector3f&		operator+=(const Vector3f& v);

		// Subtraction operator (floating-point secure)
		Vector3f		operator-(const Vector3f& v) const;

		// Subtraction operator (floating-point secure)
		Vector3f&		operator-=(const Vector3f& v);

		// Moltiplication operator (scalar)
		Vector3f		operator*(float scalar) const;

		// Moltiplication operator (scalar)
		Vector3f&		operator*=(float scalar);

		// Division operator (scalar)
		Vector3f		operator/(float scalar) const;

		// Division operator (scalar)
		Vector3f&		operator/=(float scalar);

		// Comparison operator (floating-point secure)
		bool			operator==(const Vector3f& v) const;

		// Comparison operator (floating-point secure)
		inline bool		operator!=(const Vector3f& v) const { return (!((*this) == v)); }

		// Setters for each individual components
		void			setX(const float x);
		void			setY(const float y);
		void			setZ(const float z);

		// Sets all the three components
		void			setVariables(const float x, const float y, const float z);

		// Getters for each individual components.
		inline float	getX() const { return (x_); }
		inline float	getY() const { return (y_); }
		inline float	getZ() const { return (z_); }

		// Returns the magnitude (length) of the vector
		float			magnitude() const;

		// Validates that all components are finite and non NaN with assert
		void			assertCheck();

		/*
			Validates that alla components are finite and non NaN
			Returns 1 if the variables are valid, 0 otherwise
		*/
		bool			checkNumerics() const;

		// Default destructor
		~Vector3f();
};

// Computes the dot product of two vectors
float		dot(const Vector3f& a, const Vector3f& b);

// Computes the cross product of two vectors
Vector3f	cross(const Vector3f& a, const Vector3f& b);

// Performs element-wise multiplication of two vectors
Vector3f	componentMultiply(const Vector3f& a, const Vector3f& b);

#endif

