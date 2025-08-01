#include "../../includes/physics/Vector3f.hpp"
#include <math.h>
#include <cassert>
#include <iostream>

Vector3f::Vector3f(const float x, const float y, const float z) {

	assert(!std::isinf(x) && !std::isinf(y) && !std::isinf(z)
			&& !std::isnan(x) && !std::isnan(y) && !std::isnan(z)
			&& "Error: Vector3f constructor: values must be finite");

	this->x = x;
	this->y = y;
	this->z = z;

}

Vector3f::Vector3f(const Vector3f& vector) {

	x = vector.x;
	y = vector.y;
	z = vector.z;

}

Vector3f& Vector3f::operator=(const Vector3f& vector) {

	if (this != &vector) {
		x = vector.x;
		y = vector.y;
		z = vector.z;
	}

	return (*this);
}

void	Vector3f::setVariables(const float x, const float y, const float z) {

	assert(!std::isinf(x) && !std::isinf(y) && !std::isinf(z) &&
			!std::isnan(x) && !std::isnan(y) && !std::isnan(z) &&
			"Error: Vector3f setVariables: values must be finite");

	this->x = x;
	this->y = y;
	this->z = z;

}

void	Vector3f::setX(const float x) {

	assert(!std::isinf(x) && !std::isnan(x) && "Vector3f setX: value must be finite");
	this->x = x;

}

void	Vector3f::setY(const float y) {

	assert(!std::isinf(y) && !std::isnan(y) && "Error: Vector3f setY: value must be finite");
	this->y = y;

}

void	Vector3f::setZ(const float z) {

	assert(!std::isinf(z) && !std::isnan(z) && "Error: Vector3f setZ: value must be finite");
	this->z = z;

}

Vector3f Vector3f::operator+(const Vector3f& vector) const {

	return (Vector3f(x + vector.x, y + vector.y, z + vector.z));

}

Vector3f Vector3f::operator-(const Vector3f& vector) const {

	return (Vector3f(x - vector.x, y - vector.y, z - vector.z));

}

Vector3f Vector3f::operator*(float scalar) const {

	return (Vector3f(x * scalar, y * scalar, z * scalar));

}

bool	Vector3f::operator==(const Vector3f& vector) const {

	static const float epsilon = 1e-6f;

	return (std::fabs(x - vector.getX()) < epsilon &&
			std::fabs(y - vector.getY()) < epsilon &&
			std::fabs(z - vector.getZ()) < epsilon);
}

void	Vector3f::assertVectorCheck() {

	assert(!std::isnan(x) && !std::isinf(x) && "Error: Vector3f: NaN or Inf in vector component X");
	assert(!std::isnan(y) && !std::isinf(y) && "Error: Vector3f: NaN or Inf in vector component Y");
	assert(!std::isnan(z) && !std::isinf(z) && "Error: Vector3f: NaN or Inf in vector component Z");

}

bool	Vector3f::checkNumerics(void) const {

	if (std::isnan(x) || std::isinf(x)
		|| std::isnan(y) || std::isinf(y)
		|| std::isnan(z) || std::isinf(z)) {
			return (0);
		}

	return (1);
}

float	Vector3f::magnitude() const {

	float	result = sqrt(x * x + y * y + z * z);

	assert(!std::isnan(result) && !std::isinf(result) && result >= 0.0f
		&& "Error: Vector3f magnitude: result must be finite and non-negative");

	return (result);
}

float	dot(const Vector3f& vectorA, const Vector3f& vectorB) {

	float	result;

	result = vectorA.getX() * vectorB.getX() +
			vectorA.getY() * vectorB.getY() +
			vectorA.getZ() * vectorB.getZ();

	assert(!std::isinf(result) && !std::isnan(result)
		&& "Error: Vector3f dot product: result must be finite");

	return (result);
}

Vector3f cross(const Vector3f& vectorA, const Vector3f& vectorB) {

	float	x;
	float	y;
	float	z;

	x = vectorA.getY() * vectorB.getZ() - vectorA.getZ() * vectorB.getY();
	y = vectorA.getZ() * vectorB.getX() - vectorA.getX() * vectorB.getZ();
	z = vectorA.getX() * vectorB.getY() - vectorA.getY() * vectorB.getX();

	assert(!std::isinf(x) && !std::isnan(x)
			&& !std::isinf(y) && !std::isnan(y)
			&& !std::isinf(z) && !std::isnan(z)
			&& "Error: Vector3f cross product: result components must be finite");

	return (Vector3f(x, y, z));
}

Vector3f componentMultiply(const Vector3f& vectorA, const Vector3f& vectorB) {

	return (Vector3f(vectorA.getX() * vectorB.getX(),
					vectorA.getY() * vectorB.getY(),
					vectorA.getZ() * vectorB.getZ()));

}

Vector3f::~Vector3f() {

}
