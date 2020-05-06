#ifndef EXAMPLE_PROBLEM_H
#define EXAMPLE_PROBLEM_H

#include <vector>

/**
 * This interface collects the members and the operations that the AMOSA optimization algorithm performs on a point in
 * the space of the solutions for a multi-objective optimization problem. To solve an optimization problem with AMOSA
 * it is necessary to define a proper class that abstracts the concept of solution space point.
 */
class example_problem_t
{
	public:

		example_problem_t(double tentative_x = 0, double tentative_y = 0);

		/* Members required by the amosa::optimization_engine_t template class */
		
		example_problem_t(const example_problem_t&);

		const example_problem_t& operator=(const example_problem_t&);
		
		/**
		 * This method should generate any point in the solution space.
		 * 
		 * This method should generate any point in the solution space, in a totally random way. It is used during the
		 * initialization operations of the AMOSA archive.
		 * 
		 * If the optimization problem is constrained, this function must generate points that respect these
		 * constraints.
		 */
		void randomize();

		/**
		 * Very little changes a point in the solution space to generate a nearby solution.
		 * 
		 * This method should generate points in the immediate vicinity of a point in the solution space, so that a
		 * first optimization of the initial solutions can be carried out.
		 * 
		 * If the optimization problem is constrained, this function must generate points that respect these
		 * constraints.
		 */
		example_problem_t neighbor() const;

		/** 
		 * It significantly perturbs a point in the solution space.
		 * 
		 * AMOSA makes use of this member in order to generate a solution that is in a different region of the solution
		 * space.
		 * 
		 * If the optimization problem is constrained, this function must generate points that respect these constraints.
		 */
		example_problem_t perturbate() const;

		/** 
		 * Calculates the distance between the point in question and another point in the solution space.
		 * 
		 * The metric for the distance calculation is at the discretion of the user. This function is used to reduce the
		 * size of the solution archive by clustering.
		 */
		double distance(const example_problem_t & a_solution) const;

		/**
		 * Returns a vector containing the fitness-function values calculated for the point in question.
		 * 
		 * This function is used very often during a single iteration of the algorithm, so it is good that its
		 * implementation does not compute the values of the fitness functions, but only returns their value.
		 */
		void get_fitness_values(std::vector<double>& values) const;

		/* Application-specific functions */

		double get_x() const {return x;}

		double get_y() const {return y;}

	private:
		double x;
		double y;
		std::vector<double> ff_values;

		/* fitness values computation functions */
		void compute_fitness();
		void fitness_function_1();
		void fitness_function_2();
};

#endif
