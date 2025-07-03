#ifndef GAUSSIANNOISE_HPP
#define GAUSSIANNOISE_HPP

class GaussianNoiseGenerator {

	private:
		bool	hasSpare;
		float	spare;

	public:
		GaussianNoiseGenerator();
		GaussianNoiseGenerator(const GaussianNoiseGenerator& gng);

		GaussianNoiseGenerator&	operator=(const GaussianNoiseGenerator& gng);
		
		float		generate(float mean, float stddev);
		static void	initSeed();

		~GaussianNoiseGenerator();
};

#endif
