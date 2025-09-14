#include "../../includes/physics/Vector3f.hpp"
#include <cmath>
#include <cassert>
#include <iostream>

Vector3f::Vector3f(float x, float y, float z) :
 x_(x), y_(y), z_(z) {

	assert(!std::isnan(x_) && !std::isinf(x_) && "Error: NaN or Inf in x_ component");
	assert(!std::isnan(y_) && !std::isinf(y_) && "Error: NaN or Inf in y_ component");
	assert(!std::isnan(z_) && !std::isinf(z_) && "Error: NaN or Inf in z_ component");
}

Vector3f::Vector3f(const Vector3f& v) :
 x_(v.x_), y_(v.y_), z_(v.z_) {

	assert(!std::isnan(x_) && !std::isinf(x_) && "Error: NaN or Inf in x_ component");
	assert(!std::isnan(y_) && !std::isinf(y_) && "Error: NaN or Inf in y_ component");
	assert(!std::isnan(z_) && !std::isinf(z_) && "Error: NaN or Inf in z_ component");
}

Vector3f&	Vector3f::operator=(const Vector3f& v) {

	if (this != &v) {
		x_ = v.x_;
		y_ = v.y_;
		z_ = v.z_;
	}

	assert(!std::isnan(x_) && !std::isinf(x_) && "Error: NaN or Inf in x_ component");
	assert(!std::isnan(y_) && !std::isinf(y_) && "Error: NaN or Inf in y_ component");
	assert(!std::isnan(z_) && !std::isinf(z_) && "Error: NaN or Inf in z_ component");

	return (*this);
}

Vector3f	Vector3f::operator+(const Vector3f& v) const {
	return (Vector3f(x_ + v.x_, y_ + v.y_, z_ + v.z_));
}

Vector3f&	Vector3f::operator+=(const Vector3f& v) {
	*this = *this + v;
	assertCheck();
	return (*this);
}

Vector3f	Vector3f::operator-(const Vector3f& v) const {
	return (Vector3f(x_ - v.x_, y_ - v.y_, z_ - v.z_));
}

Vector3f&	Vector3f::operator-=(const Vector3f& v) {
	*this = *this - v;
	assertCheck();
	return (*this);
}

Vector3f	Vector3f::operator*(float scalar) const {
	return (Vector3f(x_ * scalar, y_ * scalar, z_ * scalar));
}

Vector3f&	Vector3f::operator*=(float scalar) {
	*this = *this * scalar;
	assertCheck();
	return (*this);
}

Vector3f	Vector3f::operator/(float scalar) const {
	static const float EPSILON = 1e-6f;
	assert(std::fabs(scalar) > EPSILON && "Error: division by zero or near-zero scalar");
	return (Vector3f(x_ / scalar, y_ / scalar, z_ / scalar));
}

Vector3f&	Vector3f::operator/=(float scalar) {
	*this = *this / scalar;
	assertCheck();
	return (*this);
}

bool	Vector3f::operator==(const Vector3f& v)const {
	static const float epsilon = 1e-6f;

	return (std::fabs(x_ - v.x_) < epsilon &&
			std::fabs(y_ - v.y_) < epsilon &&
			std::fabs(z_ - v.z_) < epsilon);
}

void	Vector3f::setX(const float x) {
	assert(!std::isinf(x) && !std::isnan(x)
		&& "Error: x value must be a valid/finite float value");
	x_ = x;
}

void	Vector3f::setY(const float y) {
	assert(!std::isinf(y) && !std::isnan(y)
		&& "Error: y value must be a valid/finite float value");
	y_ = y;
}

void	Vector3f::setZ(const float z) {
	assert(!std::isinf(z) && !std::isnan(z)
		&& "Error: z value must be a valid/finite float value");
	z_ = z;
}

void	Vector3f::setVariables(const float x, const float y, const float z) {

	assert(!std::isinf(x) && !std::isnan(x)
		&& !std::isinf(y) && !std::isnan(y)
		&& !std::isinf(z) && !std::isnan(z)
		&& "Error: values must be finite");

	x_ = x;
	y_ = y;
	z_ = z;
}

float	Vector3f::magnitude() const {
	Vector3f	tmp(std::fabs(x_), std::fabs(y_), std::fabs(z_));
	float		maxVar = tmp.x_;

	//Scaling to avoid overflow/underflow in sqrt
	if (tmp.y_ > maxVar) {
		maxVar = tmp.y_;
	}
	if (tmp.z_ > maxVar) {
		maxVar = tmp.z_;
	}
	if (maxVar == 0.0f) {
		return (0.0f);
	}

	tmp = Vector3f(x_ / maxVar, y_ / maxVar, z_ / maxVar);

	float	res = maxVar * std::sqrt(tmp.x_ * tmp.x_ + tmp.y_ * tmp.y_ + tmp.z_ * tmp.z_);

	assert(!std::isnan(res) && !std::isinf(res) && res >= 0.0f
		&& "Error: magnitude must be a valid/finite/non-negative float value");

	return (res);
}

void	Vector3f::assertCheck() {
	assert(!std::isnan(x_) && !std::isinf(x_) && "Error: NaN or Inf in x_ component");
	assert(!std::isnan(y_) && !std::isinf(y_) && "Error: NaN or Inf in y_ component");
	assert(!std::isnan(z_) && !std::isinf(z_) && "Error: NaN or Inf in z_ component");
}

bool	Vector3f::checkNumerics() const {
	if (std::isnan(x_) || std::isinf(x_)
		|| std::isnan(y_) || std::isinf(y_)
		|| std::isnan(z_) || std::isinf(z_)) {
			return (0);
		}
	return (1);
}

 Vector3f::~Vector3f() {

 }

float dot(const Vector3f& a, const Vector3f& b) {
	float res = a.getX() * b.getX() +
				a.getY() * b.getY() +
				a.getZ() * b.getZ();

	assert(!std::isinf(res) && !std::isnan(res)
		&& "Error: dot product must be a valid/finite float value");

	return (res);
}

Vector3f cross(const Vector3f& a, const Vector3f& b) {
	float	x;
	float	y;
	float	z;

	x = a.getY() * b.getZ() - a.getZ() * b.getY();
	y = a.getZ() * b.getX() - a.getX() * b.getZ();
	z = a.getX() * b.getY() - a.getY() * b.getX();

	return (Vector3f(x, y, z));
}

Vector3f componentMultiply(const Vector3f& a, const Vector3f& b) {
	return (Vector3f(a.getX() * b.getX(),
					a.getY() * b.getY(),
					a.getZ() * b.getZ()));
}

