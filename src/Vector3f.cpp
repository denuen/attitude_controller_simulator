#include "Vector3f.hpp"
#include "limits.h"

#include <iostream>
#include <cmath>

Vector3f::Vector3f(const float x, const float y, const float z) {

	if (std::isinf(x) || std::isinf(y) || std::isinf(z)
		|| std::isnan(x) || std::isnan(y) || std::isnan(z)) {
		std::cerr << "Error: oveflow detected in Vector3f istance. Resetting values to '0.0f'" << std::endl;
	} else {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	
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

	if (std::isinf(x) || std::isinf(y) || std::isinf(z)
		|| std::isnan(x) || std::isnan(y) || std::isnan(z)) {
		std::cerr << "Error: oveflow detected in Vector3f assignment. No changes were made." << std::endl;
	} else {
		this->x = x;
		this->y = y;
		this->z = z;
	}

}

void	Vector3f::setX(const float x) {

	if (isinf(x) || isnan(x)) {
		std::cerr << "Error: oveflow detected in Vector3f (x assignment). No changes were made." << std::endl;
	} else {
		this->x = x;
	}

}

void	Vector3f::setY(const float y) {

	if (isinf(y) || isnan(y)) {
		std::cerr << "Error: oveflow detected in Vector3f (y assignment). No changes were made." << std::endl;
	} else {
		this->y = y;
	}

}

void	Vector3f::setZ(const float z) {

	if (isinf(z) || isnan(z)) {
		std::cerr << "Error: oveflow detected in Vector3f (z assignment). No changes were made." << std::endl;
	} else {
		this->z = z;
	}

}

float	dot(Vector3f& vectorA, Vector3f& vectorB) {
	float	result;

	if ( std::isinf(vectorA.getX() * vectorB.getX()) || std::isnan(vectorA.getX() * vectorB.getX()) ) {
		std::cerr << "Error: oveflow detected in Vector3f (x dot). No changes were made." << std::endl;
	} else {
		result += vectorA.getX() * vectorB.getX();
	}

	if ( std::isinf(vectorA.getY() * vectorB.getY()) || std::isnan(vectorA.getY() * vectorB.getY()) ) {
		std::cerr << "Error: oveflow detected in Vector3f (y dot). No changes were made." << std::endl;
	} else {
		result += vectorA.getY() * vectorB.getY();
	}

	if ( std::isinf(vectorA.getZ() * vectorB.getZ()) || std::isnan(vectorA.getZ() * vectorB.getZ()) ) {
		std::cerr << "Error: oveflow detected in Vector3f (z assignment). No changes were made." << std::endl;
	} else {
		result += vectorA.getZ() * vectorB.getZ();
	}

	return (result);
}

Vector3f cross(Vector3f& vectorA, Vector3f& vectorB) {
	float	x;
	float	y;
	float	z;

	x = vectorA.getY() * vectorB.getZ() - vectorA.getZ() * vectorB.getY();
	y = vectorA.getZ() * vectorB.getX() - vectorA.getX() * vectorB.getZ();
	z = vectorA.getX() * vectorB.getY() - vectorA.getY() * vectorB.getX();

	if (std::isinf(x) || std::isnan(x) ||
		std::isinf(y) || std::isnan(y) ||
		std::isinf(z) || std::isnan(z)) {
		std::cerr << "Error: overflow or NaN detected in Vector3f cross product. Returning zero vector." << std::endl;
		return Vector3f(0.0f, 0.0f, 0.0f);
	}

	return Vector3f(x, y, z);
}

Vector3f::~Vector3f() {

}
