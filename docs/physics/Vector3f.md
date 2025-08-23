# Vector3f Module API Reference

## Overview

The `Vector3f` module implements a three-dimensional vector class using single-precision floating-point components. It provides fundamental vector operations, mathematical functions, and robust numerical validation for 3D spatial computations in flight control systems. The class serves as the foundation for representing positions, velocities, angular velocities, torques, and other vector quantities throughout the simulation framework.

## Class: Vector3f

### Responsibilities

- Encapsulates three-dimensional Cartesian coordinates with single-precision arithmetic
- Provides fundamental vector algebra operations (e.g. addition, subtraction, scaling, ...)
- Implements component-wise access and manipulation with bounds checking
- Validates numerical integrity to prevent NaN and infinite value propagation
- Supports mathematical operations essential for flight dynamics calculations
- Maintains immutable semantics for mathematical operations while allowing controlled state modification

### Public API

#### Constructors & Rule of Three

```cpp
Vector3f(const float x = 0.0f, const float y = 0.0f, const float z = 0.0f)
```
Primary constructor with optional component initialization. Defaults to zero vector (0,0,0) if no parameters provided.

**Parameters:**
- `x`: X-component value (default: 0.0f)
- `y`: Y-component value (default: 0.0f)
- `z`: Z-component value (default: 0.0f)

**Usage:** Supports both explicit and implicit initialization patterns.

```cpp
Vector3f(const Vector3f& vector)
```
Copy constructor. Creates independent copy with identical component values.

```cpp
Vector3f& operator=(const Vector3f& vector)
```
Assignment operator. Provides value semantics with component-wise copying.

```cpp
~Vector3f()
```
Destructor. No dynamic memory management required.

#### Mathematical Operators

```cpp
Vector3f operator+(const Vector3f& vector) const
```
Vector addition operator. Computes component-wise sum: `(x₁+x₂, y₁+y₂, z₁+z₂)`.

**Immutability:** Does not modify either operand (argument)

```cpp
Vector3f operator-(const Vector3f& vector) const
```
Vector subtraction operator. Computes component-wise difference: `(x₁-x₂, y₁-y₂, z₁-z₂)`.

**Physical Meaning:** Displacement vector from second operand to first operand

```cpp
Vector3f operator*(float scalar) const
```
Scalar multiplication operator. Scales all components by scalar value: `(s·x, s·y, s·z)`.

**Parameters:**
- `scalar`: Scaling factor applied to all components

**Physical Meaning:** Vector scaling preserving direction (unless scalar is negative)

#### Comparison Operators

```cpp
bool operator==(const Vector3f& vector) const
```
Equality comparison operator. Performs exact floating-point comparison of all three components.

**Returns:** True if all components are exactly equal
**Note:** May be sensitive to floating-point precision issues; an epsilon constant of `1e-6f` has been applied to make the comparison secure.

```cpp
inline bool operator!=(const Vector3f& vector) const
```
Inequality operator implemented as negation of equality operator.

**Returns:** True if all components are not equal
**Note:** May be sensitive to floating-point precision issues; an epsilon constant of `1e-6f` has been applied to make the comparison secure.

#### Component Access Methods

```cpp
void setVariables(const float x, const float y, const float z)
```
Sets all three components simultaneously in a single operation.

**Parameters:**
- `x`, `y`, `z`: New component values

**Atomic Update:** Ensures consistent vector state during modification (via assert)

```cpp
void setX(const float x)
void setY(const float y)
void setZ(const float z)
```
Individual component setters for selective modification.

**Parameters:**
- Component-specific floating-point values

**Granular Control:** Enables modification of single components without affecting others
**Atomic Update:** Ensures consistent vector state during modification (via assert)

```cpp
inline float getX(void) const
inline float getY(void) const
inline float getZ(void) const
```
Component accessor methods with const-correctness.

**Returns:** Individual component values as single-precision floating-point
**Performance:** Inline implementation for zero-overhead access

#### Validation Methods

```cpp
void assertVectorCheck()
```
Validates vector integrity using assertion-based checking. Terminates program execution if invalid values detected.

**Validation Criteria:**
- No NaN (Not-a-Number) values in any component
- No infinite values in any component
- All components are representable floating-point numbers

**Usage:** Debug-mode validation for catching numerical errors early in development

```cpp
bool checkNumerics() const
```
Non-destructive numerical validation returning boolean result (production build)

**Returns:**
- `true` if all components are finite and valid
- `false` if any component is NaN, infinite, or otherwise invalid

**Production Use:** Suitable for runtime validation without program termination (the errors must be handled using the error handler)

#### Mathematical Operations

```cpp
float magnitude() const
```
Computes Euclidean norm (length) of the vector.

**Formula:** `√(x² + y² + z²)`

**Returns:** Non-negative floating-point magnitude value
**Physical Meaning:** Scalar length of vector in 3D space
**Applications:** Distance calculations, normalization, energy computations

### Free Functions

The module provides mathematical operations as free functions to maintain symmetry and support functional programming patterns and to be compliant with the c++98 std.

```cpp
float dot(const Vector3f& vectorA, const Vector3f& vectorB)
```
Computes dot (scalar) product of two vectors.

**Formula:** `A·B = Ax·Bx + Ay·By + Az·Bz`

**Returns:** Scalar result representing projection relationship
**Physical Meaning:**
- `A·B = |A||B|cos(θ)` where θ is angle between vectors
- Zero result indicates orthogonal vectors
- Positive result indicates acute angle
- Negative result indicates obtuse angle

```cpp
Vector3f cross(const Vector3f& vectorA, const Vector3f& vectorB)
```
Computes cross (vector) product following right-hand rule.

**Formula:**
```
A × B = (Ay·Bz - Az·By, Az·Bx - Ax·Bz, Ax·By - Ay·Bx)
```

**Returns:** Vector perpendicular to both input vectors
**Physical Meaning:**
- Magnitude equals area of parallelogram formed by input vectors
- Direction follows right-hand rule
- Essential for angular momentum, torque calculations

```cpp
Vector3f componentMultiply(const Vector3f& vectorA, const Vector3f& vectorB)
```
Performs element-wise (Hadamard) multiplication of vector components.

**Algorithm:** `(Ax·Bx, Ay·By, Az·Bz)`

**Returns:** Vector with component-wise products
**Applications:** Scaling operations, coordinate transformations, element-wise operations

### Usage Example

```cpp
#include "Vector3f.hpp"
#include <iostream>
#include <cmath>

// Basic vector operations
Vector3f position(1.0f, 2.0f, 3.0f);
Vector3f velocity(0.5f, -1.0f, 0.2f);
float dt = 0.1f;

// Kinematic update: position += velocity * dt
Vector3f displacement = velocity * dt;
Vector3f newPosition = position + displacement;

// Validation
newPosition.checkNumerics();
std::cout << "New position: ("
          << newPosition.getX() << ", "
          << newPosition.getY() << ", "
          << newPosition.getZ() << ")" << std::endl;
```

***
