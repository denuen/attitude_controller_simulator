# GaussianNoise Module – API Reference

## Overview

This module implements a Gaussian (normal) noise generator used by the sensor simulation components of the project. The implementation in code is a polar-form Box–Muller method (also known as the Marsaglia polar method) that produces pairs of independent standard normal variates and caches one value for the next call to improve throughput.

Design goals: deterministic, low-allocation, and computationally efficient noise suitable for real-time simulation and unit tests. The implementation intentionally uses only stack storage and the C standard library PRNG for portability and simplicity.

**Design Rationale:**

- Use a two-sample Box–Muller variant (Marsaglia polar method) to produce high-quality, independent normal variates while avoiding trigonometric functions (cos/sin) and reducing overhead.
- Keep the implementation allocation-free and header-clean to meet embedded/simulation constraints.
- Expose deterministic seeding via a static initializer so tests can reproduce sequences.

## Class: GaussianNoise

### Responsibilities

- Produce normally distributed values with given mean and standard deviation.
- Cache the second variate produced by the polar transform to halve the number of expensive transforms per sample.
- Provide a deterministic seed entry-point for reproducible test scenarios.
- Provide a minimal numerics check for the cached value.

## Public API

Constructors / Rule of Three

- ```cpp
  GaussianNoiseGenerator()
  ```

  ***Default constructor***. Initializes the internal cache as empty (no spare value).

- ```cpp
  GaussianNoiseGenerator(const GaussianNoiseGenerator& gng)
  ```

  ***Copy constructor***: copies cached state (`hasSpare` and `spare`) so a copy continues the same sequence of returned cached values.

- ```cpp
  GaussianNoiseGenerator& operator=(const GaussianNoiseGenerator& gng)
  ```

  ***Copy assignment*** with self-assignment protection. Copies cached state.

- ```cpp
  ~GaussianNoiseGenerator()
  ```

  ***Default destructor*** (no dynamic allocation).

&nbsp;

### Core methods

- ```cpp
  float generate(float mean, float stddev)
  ```

  Produces a normally distributed floating-point value distributed as $N(mean, stddev^2)$.

  Behavior details:
  - Validates inputs with assert(): `stddev >= 0.0f` and finite values.
  - If `stddev == 0.0f`, returns `mean` immediately.
  - If a cached spare value is present (`hasSpare == true`), the cached value is used (scaled and offset by mean/stddev), and `hasSpare` is cleared.
  - Otherwise the routine samples uniform values `u` and `v` in `[-1, 1]` using `std::rand()`, computes `s = u*u + v*v`, rejects when `s >= 1` or `s == 0`, then computes `factor = sqrt(-2 * log(s) / s)`. It sets `spare = v * factor`, `hasSpare = true`, and returns `mean + stddev * u * factor`.
  - Numerics: asserts that the returned value is finite before returning.

- ```cpp
  static void initSeed()
  ```

  Calls `std::srand(static_cast<unsigned int>(std::time(NULL)))` to seed the global C PRNG with the current time. This is a convenience for reproducible runs when a fixed time seed is used externally; calling this without a fixed seed makes runs non-reproducible.

- ```cpp
  bool checkNumerics() const
  ```

  Returns true when the cached `spare` value is finite. This is a minimal diagnostic and does not check global PRNG state or `hasSpare`.

- ```cpp
  inline void reset()
  ```

  Declared in the header: clears the cached spare by setting `hasSpare = false`.

&nbsp;

### Internal State

- `bool hasSpare` — indicates whether a valid `spare` value is cached from the previous polar transform.
- `float spare` — cached second standard-normal variate to be used on the next `generate()` call.

&nbsp;

### Algorithm and Numerical Details

**Implementation:**

the polar-form Box–Muller (Marsaglia polar) algorithm is used. Key points:

- Uniform sampling:

  `u` and `v` are independent uniform samples in $[-1, 1]$ constructed from `std::rand()` / `RAND_MAX` and scaled accordingly.
- Rejection sampling:

  the pair $(u,v)$ is accepted only when $s = u^2 + v^2$ is in the open interval $(0,1)$. This ensures the transformation produces correctly distributed normals.
- Transform: the code computes `factor = sqrt(-2 * log(s) / s)` and returns `u * factor` as one standard-normal variate while storing `v * factor` as the cached spare.

Numerical safeguards present in code:

- Assertions ensure the caller does not pass NaN/Inf and that stddev >= 0.
- The generated result is asserted to be finite before return.

Numerical limitations and considerations:

- To comply with the C++98 std, `std::rand()` was used. However, it limits the statistical quality of the uniform source; modern PRNGs (e.g., MT19937) provide better properties for long simulations.

&nbsp;

## Usage Example

```cpp
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cassert>
#include "sensor/GaussianNoise.hpp"

// Example: generate a large sample and report empirical mean and standard deviation.
// This demonstrates how to validate that the generator produces the requested
// standard deviation in a statistical sense. For deterministic behavior across
// runs, set a fixed seed with std::srand(...). Note: exact numeric output
// depends on the platform libc implementation of rand() and the seed used.

int main(void) {
  // Use a fixed seed to make this run reproducible on the same platform
  std::srand(12345u);
  // initSeed() additionally calls std::srand(time(NULL)) in the current
  // implementation; do not call it here if you want a fixed seed.

  GaussianNoiseGenerator gen;

  const int N = 10000;
  const float mean = 0.0f;
  const float sigma = 0.5f; // requested standard deviation

  double sum = 0.0;
  double sumsq = 0.0;

  for (int i = 0; i < N; ++i) {
    float x = gen.generate(mean, sigma);
    sum += x;
    sumsq += static_cast<double>(x) * static_cast<double>(x);
  }

  const double sample_mean = sum / static_cast<double>(N);
  const double sample_var = sumsq / static_cast<double>(N) - sample_mean * sample_mean;
  const double sample_std = std::sqrt(sample_var);

  std::cout.setf(std::ios::fixed);
  std::cout.precision(6);
  std::cout << "N=" << N << " sample_mean=" << sample_mean
        << " sample_std=" << sample_std << std::endl;

  // Basic runtime check used in tests
  assert(gen.checkNumerics());
  return 0;
}
```

Expected output (representative):

```text
N=10000 sample_mean=0.004573 sample_std=0.501267
```

Notes:

- The printed numbers above are a representative result from a single run using a fixed seed; exact values depend on the platform's implementation of `rand()` and on the seed. With the same libc and the same seed the sequence is reproducible.
- For unit tests prefer statistical assertions (e.g., sample_mean approximately equals requested mean within a small tolerance, sample_std approximately equals requested sigma within a percent tolerance) rather than exact-value equality.

&nbsp;

### Physical and Modeling Assumptions

- White-noise assumption: generated samples are independent and identically distributed (i.i.d.) across calls to `generate()` when a single generator instance is used and the underlying PRNG is independent. This is appropriate for modeling sensor white noise (e.g., high-frequency thermal noise) but does not model bias instability or temporally correlated noise.
- Zero-mean additive noise model: the API returns values distributed around the provided `mean` parameter; typical sensor usage is zero-mean with a specified standard deviation.
- Uncorrelated channels: use separate generator instances for independent noise sources to avoid reusing the cached spare across logically distinct signals.

&nbsp;

### Limitations, Safety, and Future Work

Limitations:

- PRNG quality: uses the C `rand()` family which has limited period and statistical quality for long Monte‑Carlo runs. For high-fidelity simulations, replace with a C++ `"<random>"` header based engine (for example, `std::mt19937`) and a `std::uniform_real_distribution<float>`.
- Thread-safety: the implementation is not thread-safe. `std::rand()` and the class' cached state are global or per-instance mutable state; external synchronization is required for concurrent use.

Possible planned extensions (theoretically):

- Add a configurable PRNG backend (policy or dependency injection) to allow swapping to std::mt19937 for better quality and to support seeding per-instance for reproducible parallel runs.
- Add an option to produce vectors of samples or to fill buffers efficiently when bulk generation is required.
- Provide a small test harness that validates sample statistics (mean, variance) against target parameters using a high-quality PRNG.
