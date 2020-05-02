#include "problem.hpp"
#include <random>
#include <cmath>

/**
 * @brief This method generates a random point in the solution space.
 * 
 * @details
 * This method should generate any point in the solution space, in a totally random way. It is used during the
 * initialization operations of the AMOSA archive.
 * 
 * @warning If the optimization problem is constrained, this function must generate points that respect these
 * constraints.
 */
void example_problem_t::randomize()
{
	std::random_device rd;
	std::default_random_engine dre(rd());
	x = (double) dre() / dre();
	y = (double) dre() / dre();
}

/**
 * @brief Ammplies very little changes to the point in the solution space, in order to generate a nearby solution.
 * 
 * @details
 * This method should generate points in the immediate neighborhood of a point in the solution space, so that a
 * first optimization of the initial solutions can be carried out with hill-climbing.
 * 
 * @warning If the optimization problem is constrained, this function must generate points that respect these
 * constraints.
 */
void example_problem_t::neighbor()
{
	std::random_device rd;
    std::default_random_engine gen(rd());
	std::bernoulli_distribution bd;
	x *= (bd(gen) ? 0.9 : 1.1);
	y *= (bd(gen) ? 0.9 : 1.1);
}

/**
 * @brief It significantly perturbs a point in the solution space.
 * 
 * @details
 * AMOSA makes use of this member in order to generate a solution that is in a different region of the solution
 * space.
 * 
 * @warning If the optimization problem is constrained, this function must generate points that respect these
 * constraints.
 */
void example_problem_t::perturbate()
{
	std::random_device rd;
    std::default_random_engine gen(rd());
	std::bernoulli_distribution bd;
	x *= (bd(gen) ? 0.13 : 7);
	y *= (bd(gen) ? 0.13 : 7);
}

/**
 * @brief Calculates the distance between two points in the solution space.
 * 
 * @details
 * The metric for the distance calculation is at the discretion of the user. This function is used to reduce the
 * size of the solution archive by clustering.
 * 
 * @param a_solution another point in the solution space.
 * 
 * @return double distance between the points.
 */
double example_problem_t::distance(const example_problem_t & solution) const
{
	double dist = std::sqrt(std::pow((x - solution.x), 2) + std::pow((y - solution.y), 2));
	return dist;
}

/**
 * @brief Returns a vector containing the fitness-function values calculated for the point in question.
 * 
 * @param ff_values vector containing the fitness-function values.
 */
void example_problem_t::get_fitness_values(std::vector<double>& ff_values) const
{
	ff_values.erase(ff_values.begin(), ff_values.end());
	ff_values.emplace_back(std::fabs(x - y));
	ff_values.emplace_back(1/(x + y));
}