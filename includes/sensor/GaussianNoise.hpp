#ifndef GAUSSIAN_NOISE_HPP
#define GAUSSIAN_NOISE_HPP

// Generates Gaussian-distributed random noise using the Box-Muller transform
class GaussianNoiseGenerator {

	private:
		float	spare_; // Cached spare value from the Box-Muller transform
		bool	hasSpare_; // Flag indicating whether a spare value is cached from the previous generation

	public:
		// Constructs a gng with no cached values
		GaussianNoiseGenerator();

		// Copy constructor
		GaussianNoiseGenerator(const GaussianNoiseGenerator& g);

		// Copy assignment operator
		GaussianNoiseGenerator&	operator=(const GaussianNoiseGenerator& g);

		// Generates normally distributed random values with specified mean and standard deviation
		float		generate(float mean, float stddev);

		// Initializes the global random number generator with the current time
		static void	initSeed();

		/*
			Checks the correctness of all the required values of the module.
			Returns 1 if they are correct, 0 otherwise.
		*/
		bool		checkNumerics();

		// Clears the cached spare value
		inline void	reset() { hasSpare_ = 0; }

		// Default destructor
		~GaussianNoiseGenerator();

};
#endif
