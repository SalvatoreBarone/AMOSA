#include <vector>
#include <iostream>
#include "amosa.hpp"
#include "problem.hpp"

int main()
{
	amosa::optimization_engine_t<example_problem_t> engine(3422, 24, 0.97, 1500, 100, 30);

	engine.run(8);

	std::vector<example_problem_t> archive = engine.get_pareto_front();
	std::vector<example_problem_t>::const_iterator it = archive.cbegin(), end = archive.cend();
	for (; it != end; it++)
	{
		std:: cout << it->get_x() << "\t" << it->get_y() << std::endl;
	}
}