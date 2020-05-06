#include "problem3.hpp"
#include <random>
#include <cmath>

static std::random_device rd;
static std::default_random_engine gen(rd());
static std::bernoulli_distribution bd;

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

void example_problem_t::randomize()
{
	do
	{
		x = (double) gen() / (double) gen();
		y = 5 * x + (double)(9 / (9 + gen() % 90));
	}
	while ((!constraint_1() || !constraint_2() || !constraint_3() || !constraint_4()));
	compute_fitness();
}

example_problem_t example_problem_t::neighbor() const
{
	double down = 0.9, up = 1.1;
	unsigned count = 3;
	example_problem_t new_solution;
	do
	{
		count = 0;
		double xs[4] = {x * down, x * up, x * down, x * up};
		double ys[4] = {y * down, y * down, y * up, y * up};
		do
		{
			new_solution.x = xs[count];
			new_solution.y = ys[count];
			count++;
		}
		while (count < 5 && !new_solution.meet_constraints());
		down += 0.001;
		up -= 0.001;
	} while(count == 5);
	new_solution.compute_fitness();
	return new_solution;
}

example_problem_t example_problem_t::perturbate() const
{
	double down = 0.2, up = 1.8;
	unsigned count = 3;
	example_problem_t new_solution;
	do
	{
		count = 0;
		double xs[4] = {x * down, x * up, x * down, x * up};
		double ys[4] = {y * down, y * down, y * up, y * up};
		do
		{
			new_solution.x = xs[count];
			new_solution.y = ys[count];
			count++;
		}
		while (count < 5 && !new_solution.meet_constraints());
		down += 0.01;
		up -= 0.01;
	} while(count == 5);
	new_solution.compute_fitness();
	return new_solution;
}

double example_problem_t::distance(const example_problem_t & solution) const
{
	double dist = std::sqrt(std::pow((x - solution.x), 2) + std::pow((y - solution.y), 2));
	return dist;
}

void example_problem_t::get_fitness_values(std::vector<double>& values) const
{
	values.erase(values.begin(), values.end());
	values.insert(values.begin(), ff_values.cbegin(), ff_values.cend());
}

void example_problem_t::compute_fitness()
{
	double value = 7 * x + y;
	ff_values.erase(ff_values.begin(), ff_values.end());
	ff_values.push_back(value);
}

bool example_problem_t::constraint_1() const
{
	return (4 * x + y) >= 12;
}

bool example_problem_t::constraint_2() const
{
	return (x >= 0);
}

bool example_problem_t::constraint_3() const
{
	return (5 * x - y) <= 0;
}

bool example_problem_t::constraint_4() const
{
	return (y >= 0);
}

bool example_problem_t::meet_constraints() const
{
	return	constraint_1() && constraint_2() && constraint_3() && constraint_4();
}

