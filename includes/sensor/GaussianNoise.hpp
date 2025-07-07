#ifndef GAUSSIANNOISE_HPP
#define GAUSSIANNOISE_HPP

// Generates Gaussian-distributed random noise using the Box-Muller transform.
class GaussianNoiseGenerator {

	private:
		bool hasSpare;	// Flag indicating whether a spare value is cached from the previous generation.
		float spare;	// Cached spare value from the Box-Muller transform.

	public:
		// Constructs a Gaussian noise generator with no cached values.
		GaussianNoiseGenerator();

		// Copy constructor.
		GaussianNoiseGenerator(const GaussianNoiseGenerator& gng);

		// Copy assignment operator.
		GaussianNoiseGenerator& operator=(const GaussianNoiseGenerator& gng);

		// Generates normally distributed random values with specified mean and standard deviation.
		float generate(float mean, float stddev);

		// Initializes the global random number generator with current time.
		static void initSeed();

		// Clears the cached spare value.
		inline void reset() { hasSpare = false; }

		// Destructor.
		~GaussianNoiseGenerator();
};

#endif
