#include "problem.hpp"
#include <random>
#include <cmath>

example_problem_t::example_problem_t(double tentative_x, double tentative_y) : x(tentative_x), y(tentative_y)
{
	compute_fitness();
}

example_problem_t::example_problem_t(const example_problem_t& ep) : x(ep.x), y(ep.y), ff_values(ep.ff_values) {}

const example_problem_t& example_problem_t::operator=(const example_problem_t& ep)
{
	if (this != &ep)
	{
		x = ep.x;
		y = ep.y;
		ff_values = ep.ff_values;
	}
	return *this;
}

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
	std::bernoulli_distribution bd;
	x = (double) dre() / dre() * (bd(dre) ? 1 : -1);
	y = (double) dre() / dre() * (bd(dre) ? 1 : -1);
	compute_fitness();
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
example_problem_t example_problem_t::neighbor() const
{
	std::random_device rd;
    std::default_random_engine gen(rd());
	std::bernoulli_distribution bd;
	return example_problem_t(x * (bd(gen) ? 0.9 : 1.1), y * (bd(gen) ? 0.9 : 1.1));
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
example_problem_t example_problem_t::perturbate() const
{
	std::random_device rd;
    std::default_random_engine gen(rd());
	std::bernoulli_distribution bd;
	return example_problem_t(x * (bd(gen) ? 0.13 : 7), y * (bd(gen) ? 0.13 : 7));
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

/* The f.f. values are copied into the values vector */
void example_problem_t::get_fitness_values(std::vector<double>& values) const
{
	values.erase(values.begin(), values.end());
	values.insert(values.begin(), ff_values.cbegin(), ff_values.cend());
}

/* compure both the fitness functions */
void example_problem_t::compute_fitness()
{
	ff_values.erase(ff_values.begin(), ff_values.end());
	fitness_function_1();
	fitness_function_2();
}

/* first fitness function: minimize the difference between x and y */
void example_problem_t::fitness_function_1()
{
	ff_values.push_back(std::fabs(x - y));
}

/* second fitness function: maximize the x+y i.e. minimize -(x+y) */
void example_problem_t::fitness_function_2()
{
	ff_values.push_back(-(x + y));
}
