#ifndef EXAMPLE2_PROBLEM_H
#define EXAMPLE2_PROBLEM_H

#include <vector>

/**
 * Encodes min: z = 7x + y
 * Constraints:
 * 	1.	4x + y 	>= 12
 *  2.  x 		>= 0
 *  3.  5x - y 	<= 0
 * 	4.	y 		>= 0
 * Solution: (0,12)
 */
class example_problem_t
{
	public:
		example_problem_t(double tentative_x = 1, double tentative_y = 1);
		example_problem_t(const example_problem_t&);
		const example_problem_t& operator=(const example_problem_t&);
		static example_problem_t randomize();
		example_problem_t neighbor() const;
		example_problem_t perturbate() const;
		double distance(const example_problem_t & a_solution) const;
		void get_fitness_values(std::vector<double>& values) const;
		double get_x() const {return x;}
		double get_y() const {return y;}
	private:
		double x;
		double y;
		std::vector<double> ff_values;
		void compute_fitness();
		bool constraint_1() const;
		bool constraint_2() const;
		bool constraint_3() const;
		bool constraint_4() const;
		bool meet_constraints() const;
};

#endif
