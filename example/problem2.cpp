#include "problem2.hpp"
#include <random>
#include <cmath>

static std::random_device rd;
static std::default_random_engine gen(rd());
static std::bernoulli_distribution bd;

example_problem_t::example_problem_t() : x(1), y(1)
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

void example_problem_t::neighbor()
{
	double down = 0.9, up = 1.1;
	unsigned count = 3;
	double x_orig = x, y_orig = y;
	do
	{
		count = 0;
		double xs[4] = {x_orig * down, x_orig * up, x_orig * down, x_orig * up};
		double ys[4] = {y_orig * down, y_orig * down, y_orig * up, y_orig * up};
		do
		{
			x = xs[count];
			y = ys[count];
			count++;
		}
		while (count < 5 && (!constraint_1() || !constraint_2() || !constraint_3() || !constraint_4()));
		down += 0.001;
		up -= 0.001;
	} while(count == 5);
	compute_fitness();
}

void example_problem_t::perturbate()
{
	double down = 0.2, up = 1.8;
	unsigned count = 3;
	double x_orig = x, y_orig = y;
	do
	{
		count = 0;
		double xs[4] = {x_orig * down, x_orig * up, x_orig * down, x_orig * up};
		double ys[4] = {y_orig * down, y_orig * down, y_orig * up, y_orig * up};
		do
		{
			x = xs[count];
			y = ys[count];
			count++;
		}
		while (count < 5 && (!constraint_1() || !constraint_2() || !constraint_3() || !constraint_4()));
		down += 0.01;
		up -= 0.01;
	} while(count == 5);
	compute_fitness();
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

bool example_problem_t::constraint_1()
{
	return (4 * x + y) >= 12;
}

bool example_problem_t::constraint_2()
{
	return (x >= 0);
}

bool example_problem_t::constraint_3()
{
	return (5 * x - y) <= 0;
}

bool example_problem_t::constraint_4()
{
	return (y >= 0);
}