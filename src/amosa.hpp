/**
 * @file amosa.hpp
 * @author Salvatore Barone <salvator.barone@gmail.com>
 *
 * Copyright (C) 2020 Salvatore Barone <salvator.barone@gmail.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AMOSA_OPTIMIZATION_ENGINE_TEMPLATE_CLASS_H
#define AMOSA_OPTIMIZATION_ENGINE_TEMPLATE_CLASS_H

#include <thread>
#include <vector>
#include <random>
#include <limits>
#include <exception>
#include <iostream>

/**
 * @mainpage Archived Multi-Objective Simulated Annealing
 * 
 * This is a C++11 template implementation of the Archived Multi-Objective Simulated Annealing algorithm[1] leveraging
 * the C++ Standard Template Library.
 *  
 * @section using-amosa Using the AMOSA template library
 * 
 * The library consists of a single  header file, amosa.hpp, that must be included in tour project.
 * It implements the template class amosa::optimization_engine_t, which can be characterized by specifying the template
 * parameter. The amosa::optimization_engine_t class implements only the AMOSA algorithm: it does not offer any
 * mechanism to define a particular optimization problem.
 * 
 * The following example code shows how to instantiate a object of the amosa::optimization_engine_t class to solve an
 * optimization problem. After calling the class construction, specifying the template parameter and the different
 * characteristic parameters of the AMOSA algorithm, the amosa::optimization_engine_t::run method is called to start
 * the algorithm. You can specify how many threads to use to parallelize operations and reduce calculation time.
 * At the end, the solutions found are printed.
 * 
 * Note that there is no definition of the optimization problem.
 * 
 * @include example.cpp
 *  
 * @subsection define-a-problem Defining a problem
 * 
 * In order to use  the amosa::optimization_engine_t class  to solve an optimization problem, user have to define a
 * proper class that provides the abstraction of a point in the solution space point.
 * The following block of codes provides an example: the example_problem_t class defines the operations required by
 * the AMOSA optimization algorithm. In particular:
 * 1.   randomize: should generate any point in the solution space, in a totally random way. It is used during the
 * 		initialization operations of the AMOSA archive. If the optimization problem is constrained, this function must
 * 		generate points that meet such constraints.
 * 2.   neighbor: should generate points in the immediate neighborhood of a given point in the solution space, so that a
 * 		first optimization of the initial solutions can be carried out using gradient descent. If the optimization
 * 		problem is constrained, this function must generate points that meet these constraints.
 * 3.   perturbate: It sould significantly perturb a point in the solution space. AMOSA makes use of this member in
 * 		order to generate a solution that is in a different region of the solution space. If the optimization problem is
 * 		constrained, this function must generate points that respect these constraints.
 * 4.   distance: should compute the distance between the point in question and a different point in the solution space.
 * 		The metric for the distance calculation is at the discretion of the user. This function is used to reduce the
 * 		size of the pareto front during clustering.
 * 5.   get_fitness_values: should return a vector containing the fitness-function values computed for the point. This
 * 		function is used very often during a single iteration of the AMOSA algorithm, so it is good that its 
 * 		actual implementation does not compute the values of the fitness functions, but only returns their value.
 * 
 * @include problem.hpp
 * 
 * The following code block shows an example implementation for these functions. Note that the example implementation is
 * extremely simple and is designed to allow a new user how to use the amosa class::optimization_engine_t to solve a
 * problem, which is why the implementation is almost trivial.
 * 
 * @include problem.cpp
 * 
 * @subsection define-constrained Define a constrained problem
 * 
 * Consider the optimization problem of minimizing @f$ f(x,y) = 7 x + y @f$ under the following constraints:
 * 1. @f$ 4x + y \ge 12 @f$
 * 2. @f$ x \ge 0 @f$
 * 3. @f$ 5x - y \le 0 @f$
 * 4. @f$ y	\ge 0 @f$
 * 
 * Note that the example problem is extremely simple (it is not a multi-objective problem) and the optimum, which is
 * (0,12), can be easely found by hand. The following example shows how to encode a constrained optimization problem.
 * The efficient encoding of randomize, neighbor and perturbed functions can have beneficial effects on the algorithm
 * execution time. In particular, it is necessary to implement these functions so that a few cycles are needed to
 * obtain a point in the solution space that meets the constraints.
 * A good knowledge of the problem to be solved is therefore necessary, so that random points and points close to a
 * given point in the space of the solutions that respect the constraints can be generated.
 * 
 * @include problem2.hpp
 * 
 * A good technique to generate random points is to generate the value of variables taking into account the constraints
 * and the relationships between variables imposed by constraints.
 * A good technique to generate points close to a given point, is to define a variation interval first wide, then
 * narrower and narrower.
 * 
 * @include problem2.cpp
 * 
 * @subsection compile Compile and link.
 * The AMOSA template library is provided with a CMake file for installation purpose. Just run the following
 * @code
 * $ mkdir build
 * $ cd build
 * $ make
 * # make install
 * @endcode
 * This will put the amosa.hpp file in /usr/local/include, so you can easily include the library.
 * 
 * In order to compile a project using the AMOSA template library, the \a -std=c++11 flag must be added to the compiler
 * flags. Moreover, the \a pthread library must be linked with the executable.
 * 
 * @section reference References
 * 1. Bandyopadhyay, Sanghamitra, et al. "A simulated annealing-based multiobjective optimization algorithm: AMOSA."
 * 	  IEEE transactions on evolutionary computation 12.3 (2008): 269-283.
 */

namespace amosa
{
/**
 * @defgroup amosa AMOSA algorithm
 * @{
 * @brief Archived Multi-Objective Simulated Annealing algorithm template class.
 */

/**
 * @brief Optimization engine template class
 * 
 * @tparam amosa_point_t a class implementing the amosa::solution_space_point_t interface.
 */
template<typename amosa_point_t>
class optimization_engine_t
{
	public:
	
	/**
	 * @brief Construct a new optimization engine t object
	 * 
	 * @param initial_temperature 
	 * @param final_temperature 
	 * @param cooling_factor 
	 * @param num_iteration 
	 * @param archive_size_soft_limit 
	 * @param archive_size_hard_limit 
	 */
	optimization_engine_t(
			double initial_temperature = 3422,
			double final_temperature = 24,
			double cooling_factor = 0.9,
			unsigned int num_iteration = 1500,
			unsigned int archive_size_soft_limit = 50,
			unsigned int archive_size_hard_limit = 30) :
		initial_temperature(initial_temperature),
		final_temperature(final_temperature),
		cooling_factor(cooling_factor),
		num_iteration(num_iteration),
		archive_size_soft_limit(archive_size_soft_limit),
		archive_size_hard_limit(archive_size_hard_limit)
		{
			if (initial_temperature < final_temperature)
				throw std::invalid_argument("The initial temperature must be larger than final temperature");

			if (cooling_factor <= 0 || cooling_factor >= 1)
				throw std::invalid_argument("The cooling factor must be in the (0, 1) range");

			if (archive_size_hard_limit > archive_size_soft_limit)
				throw std::invalid_argument("The hard size limit must be smaller than the soft limit");
		}

	/**
	 * @brief The copy constructor is deleted.
	 */
	optimization_engine_t(const optimization_engine_t &) = delete;

	/**
	 * @brief Set the archive size soft limit.
	 * 
	 * @param soft_limit new soft limit.
	 * 
	 * @note The value will be set only if the optimization engine is not
	 * running.
	 */
	void set_archive_size_soft_limit(unsigned int soft_limit)
	{
		if (soft_limit < archive_size_hard_limit)
			throw std::invalid_argument("The soft size limit must be larger than the hard limit");
		archive_size_soft_limit = soft_limit;
	}

	/**
	 * @brief Set the archive size hard limit.
	 * 
	 * @param hard_limit new hard limit.
	 * 
	 * @note The value will be set only if the optimization engine is not
	 * running.
	 */
	void set_archive_size_hard_limit(unsigned int hard_limit)
	{
		if (hard_limit > archive_size_soft_limit)
			throw std::invalid_argument("The hard size limit must be smaller than the soft limit");
		archive_size_hard_limit = hard_limit;
	}

	/**
	 * @brief Set the initial temperature.
	 * 
	 * @param temperature new temperature value.
	 * 
	 * @note The value will be set only if the optimization engine is not
	 * running.
	 */
	void set_initial_temperature(double temperature)
	{
		if (temperature < final_temperature)
			throw std::invalid_argument("The initial temperature must be larger than final temperature");
		initial_temperature = temperature;
	}

	/**
	 * @brief Set the final temperature
	 * 
	 * @param temperature new temperature value.
	 * 
	 * @note The value will be set only if the optimization engine is not
	 * running.
	 */
	void set_final_temperature(double temperature)
	{
		if (temperature > initial_temperature)
			throw std::invalid_argument("The final temperature must be smaller than the initial temperature");
		final_temperature = temperature;
	}

	/**
	 * @brief Set the coling factor
	 * 
	 * @param factor new cooling factor.
	 * 
	 * @note The value will be set only if the optimization engine is not
	 * running.
	 */
	void set_coling_factor(double factor)
	{
		if (factor <= 0 || factor >= 1)
			throw std::invalid_argument("The cooling factor must be in the (0, 1) range");
		cooling_factor = factor;
	}

	/**
	 * @brief Set the numiteration.
	 * 
	 * @param iterations new amout of iteration.
	 * 
	 * @note The value will be set only if the optimization engine is not
	 * running.
	 */
	void set_numiteration(unsigned int iterations) noexcept 
	{
		num_iteration = iterations;
	}

	void run(unsigned int num_threads = 2);

	void run(const amosa_point_t&, unsigned int num_threads = 2);

	/**
	 * @brief Get the archive.
	 * 
	 * @return std::vector<amosa_point_t> the archive of Pareto-optimal solutions.
	 */
	std::vector<amosa_point_t> get_pareto_front() const
	{
		return pareto_archive;
	}

	private:
	
		double initial_temperature;
		double final_temperature;
		double cooling_factor;
		unsigned int num_iteration;
		unsigned int archive_size_soft_limit;
		unsigned int archive_size_hard_limit;

		std::vector<amosa_point_t> pareto_archive;

		void initialize_archive(unsigned);

		void initialize_archive_single_thread(unsigned, typename std::vector<std::vector<amosa_point_t>>::iterator);

		void initialize_archive_baseline(const amosa_point_t&, unsigned);

		void initialize_archive_baseline_single_thread(const amosa_point_t&, unsigned, typename std::vector<std::vector<amosa_point_t>>::iterator);

		void compute_minimum_dominance(double &, unsigned &);

		void archive_clustering(std::vector<amosa_point_t>&, bool = false);

		double compute_clustering_threshold(const std::vector<amosa_point_t>&) const;

		bool dominates(const amosa_point_t &, const amosa_point_t &) const;

		double amount_of_domination(const amosa_point_t&, const amosa_point_t&, const std::vector<double>&) const;
		
		void amounts_of_domination(const std::vector<amosa_point_t>&, const amosa_point_t&, const amosa_point_t&, double&, double&, double&) const;

		void compute_ranges(const std::vector<amosa_point_t>&, std::vector<double>&) const;

		bool accept_new_solution(double, double) const;

		bool accept_new_solution(double) const;

		unsigned remove_dominated_archived_solutions(const amosa_point_t&, std::vector<amosa_point_t>&);

		void add_candidate_solution(std::vector<amosa_point_t>&, const amosa_point_t&);

		void run_single_thread(typename std::vector<std::vector<amosa_point_t>>::iterator, double);
}; /* optimization_engine_t */

/**
 * @brief Run the optimization algorithm using either a single or multiple threads
 * 
 * @param num_threads number of threads. In order to speed up the algorithm, the number of iterations expected for each
 * temperature level is divided over several threads.
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::run(unsigned int num_threads)
{
	if (num_threads == 0)
		throw std::invalid_argument("The amount of parallel threads cannot be 0");

	initialize_archive(num_threads);

	// temperature partitioning and padding
	double current_temperature = initial_temperature;
	std::vector<double> temperatures;
	while (current_temperature > final_temperature)
	{
		temperatures.push_back(current_temperature);
		current_temperature *= cooling_factor;
	}
	while (temperatures.size() % num_threads != 0)
		temperatures.insert(temperatures.begin(), initial_temperature);

	std::vector<double>::const_iterator temperatures_it = temperatures.cbegin(), temperatures_end = temperatures.cend();
	while (temperatures_it != temperatures_end)
	{
		std::cout << "Current temperature " << *temperatures_it << std::endl;
		// each thread has its own private and temporary archive
		std::vector<std::vector<amosa_point_t>> private_archives(num_threads);
		typename std::vector<std::vector<amosa_point_t>>::iterator arc_it = private_archives.begin();
		std::vector<std::thread> threads_id;
		for (unsigned i = 0; i < num_threads; i++, arc_it++, temperatures_it++)
		{
			arc_it->erase(arc_it->begin(), arc_it->end()); // maybe can be removed

			// generating private archives
			arc_it->insert(arc_it->begin(), pareto_archive.begin(), pareto_archive.end());
			// spawning a new thread. Each thread processes a given temperature
			threads_id.push_back(
				std::thread(
					&optimization_engine_t<amosa_point_t>::run_single_thread,
					std::ref(*this), 
					arc_it,
					*temperatures_it));
		}

		// private and temporary archives must be merged
		std::vector<std::thread>::iterator thr_it;
		arc_it = private_archives.begin();
		pareto_archive.erase(pareto_archive.begin(), pareto_archive.end());
		for(thr_it = threads_id.begin(); thr_it != threads_id.end(); thr_it++)
		{
			thr_it->join();
			pareto_archive.insert(
				pareto_archive.end(),
				std::make_move_iterator(arc_it->begin()),
				std::make_move_iterator(arc_it->end()));
		}
		
		// Archive clustering is needed to reduce the amount of archive solutions.
		archive_clustering(pareto_archive, true);
		
		// Removes eventually dominated solutions from the archive
		typename std::vector<amosa_point_t>::iterator it = pareto_archive.begin();
		for (; it != pareto_archive.end(); it++)
			remove_dominated_archived_solutions(*it, pareto_archive);		
	}
}


/**
 * @brief Run the optimization algorithm, starting on a known solution, using either a single or multiple threads.
 *
 * @details
 * In case a non-trivial solution of the problem is available, even far from the optimal one, it is possible to start 
 * the algorithm from that solution. In this case the archive is initially built from this solution, instead of random
 * solutions. Once the archive is built from this solution, the algorithm continues normally.
 * 
 * @param baseline_solution Starting solution.
 * @param num_threads number of threads. In order to speed up the algorithm, the number of iterations expected for each
 * temperature level is divided over several threads.
 *
 * @tparam amosa_point_t
 * @param num_threads
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::run(const amosa_point_t& baseline_solution, unsigned int num_threads)
{
	if (num_threads == 0)
		throw std::invalid_argument("The amount of parallel threads cannot be 0");

	initialize_archive_baseline(baseline_solution, num_threads);

	// temperature partitioning and padding
	double current_temperature = initial_temperature;
	std::vector<double> temperatures;
	while (current_temperature > final_temperature)
	{
		temperatures.push_back(current_temperature);
		current_temperature *= cooling_factor;
	}
	while ((temperatures.size() % num_threads) != 0)
		temperatures.insert(temperatures.begin(), initial_temperature);

	std::vector<double>::const_iterator temperatures_it = temperatures.cbegin(), temperatures_end = temperatures.cend();
	while (temperatures_it != temperatures_end)
	{
		std::cout << "Current temperature " << *temperatures_it << std::endl;
		// each thread has its own private and temporary archive
		std::vector<std::vector<amosa_point_t>> private_archives(num_threads);
		typename std::vector<std::vector<amosa_point_t>>::iterator arc_it = private_archives.begin();
		std::vector<std::thread> threads_id;
		for (unsigned i = 0; i < num_threads; i++, arc_it++, temperatures_it++)
		{
			arc_it->erase(arc_it->begin(), arc_it->end()); // maybe can be removed

			// generating private archives
			arc_it->insert(arc_it->begin(), pareto_archive.begin(), pareto_archive.end());
			// spawning a new thread. Each thread processes a given temperature
			threads_id.push_back(
				std::thread(
					&optimization_engine_t<amosa_point_t>::run_single_thread,
					std::ref(*this), 
					arc_it,
					*temperatures_it));
		}

		// private and temporary archives must be merged
		std::vector<std::thread>::iterator thr_it;
		arc_it = private_archives.begin();
		pareto_archive.erase(pareto_archive.begin(), pareto_archive.end());
		for(thr_it = threads_id.begin(); thr_it != threads_id.end(); thr_it++)
		{
			thr_it->join();
			pareto_archive.insert(
				pareto_archive.end(),
				std::make_move_iterator(arc_it->begin()),
				std::make_move_iterator(arc_it->end()));
		}
		
		// Archive clustering is needed to reduce the amount of archive solutions.
		archive_clustering(pareto_archive, true);
		
		// Removes eventually dominated solutions from the archive
		typename std::vector<amosa_point_t>::iterator it = pareto_archive.begin();
		for (; it != pareto_archive.end(); it++)
			remove_dominated_archived_solutions(*it, pareto_archive);		
	}

}

/**
 * @brief Archive initialization.
 * 
 *
 * @note This function only manages workload partitioning among threads, in order to speed-up the initialization
 * process. See initialize_archive_single_thread().
 *
 * @param num_threads number of threads to be used to speed-up the initialization process.
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::initialize_archive(unsigned num_threads)
{
	// twice the soft limit archive size solutions will be generated, to improve diversity
	unsigned total_starting_solutions = 2 * archive_size_soft_limit / num_threads;
	
	// partitioning of the outer loop
	// each thread has its own private and temporary archive
	std::vector<std::vector<amosa_point_t>> private_archives(num_threads);
	typename std::vector<std::vector<amosa_point_t>>::iterator arc_it = private_archives.begin();
	std::vector<std::thread> threads_id;
	for (unsigned i = 0; i < num_threads; i++, arc_it++)
		threads_id.push_back(
			std::thread(
				&optimization_engine_t<amosa_point_t>::initialize_archive_single_thread,
				std::ref(*this), 
				total_starting_solutions, 
				arc_it));

	// private and temporary archives must be merged
	std::vector<std::thread>::iterator thr_it;
	arc_it = private_archives.begin();
	pareto_archive.erase(pareto_archive.begin(), pareto_archive.end());
	for(thr_it = threads_id.begin(); thr_it != threads_id.end(); thr_it++)
	{
		thr_it->join();
		pareto_archive.insert(
			pareto_archive.end(),
			std::make_move_iterator(arc_it->begin()),
			std::make_move_iterator(arc_it->end()));
	}
	
	// Archive clustering is needed to reduce the amount of archive solutions.
	archive_clustering(pareto_archive, true);
}

/**
 * @brief  Function performed by a single thread during the archive initialization procedure.
 *
 * @details
 * A number of random solutions is generated, then, each of these solutions is refined using hill-clinbimg, accepting a
 * new solution only if dominated the previous one. Hill-climbing is performed a fixed amount of iterations. Thereafter,
 * the non-dominated solutions obtained with hill-climbing are stored in the archive up to a maximum equals to the
 * archive size hard limit. In case the amount of non dominated solutions excedes the size limit, clustering is applied.
 *
 * @param solutions amount of solutions the thread has to generate
 * @param private_archive_it private archive for the thread
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::initialize_archive_single_thread(
	unsigned solutions,
	typename std::vector<std::vector<amosa_point_t>>::iterator private_archive_it)
{
	for (unsigned i = 0; i < solutions; i++)
	{
		amosa_point_t candidate_solution;
		candidate_solution.randomize();

		// Performing hill-climbing
		for (unsigned int j = 0; j < num_iteration; j++)
		{
			// generating a new solution
			amosa_point_t new_solution = candidate_solution.neighbor();
			if (dominates(new_solution, candidate_solution))
				candidate_solution = std::move(new_solution);
		}

		// After hill-climbing, candidate solution are always inserted in the archive. The archive clustering will
		// eventually remove duplicate or dominated solutions.
		private_archive_it->push_back(candidate_solution);
	}
}

/**
 * @brief Archive initialization.
 * 
 * @note This function only manages workload partitioning among threads, in order to speed-up the initialization
 * process. See initialize_archive_single_thread().
 *
 * @param num_threads number of threads to be used to speed-up the initialization process.
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::initialize_archive_baseline(const amosa_point_t& baseline_solution, unsigned num_threads)
{
	// twice the soft limit archive size solutions will be generated, to improve diversity
	unsigned total_starting_solutions = 2 * archive_size_soft_limit / num_threads;
	
	// partitioning of the outer loop
	// each thread has its own private and temporary archive
	std::vector<std::vector<amosa_point_t>> private_archives(num_threads);
	typename std::vector<std::vector<amosa_point_t>>::iterator arc_it = private_archives.begin();
	std::vector<std::thread> threads_id;
	for (unsigned i = 0; i < num_threads; i++, arc_it++)
		threads_id.push_back(
			std::thread(
				&optimization_engine_t<amosa_point_t>::initialize_archive_baseline_single_thread,
				std::ref(*this), 
				baseline_solution,
				total_starting_solutions, 
				arc_it));

	// private and temporary archives must be merged
	std::vector<std::thread>::iterator thr_it;
	arc_it = private_archives.begin();
	pareto_archive.erase(pareto_archive.begin(), pareto_archive.end());
	for(thr_it = threads_id.begin(); thr_it != threads_id.end(); thr_it++)
	{
		thr_it->join();
		pareto_archive.insert(
			pareto_archive.end(),
			std::make_move_iterator(arc_it->begin()),
			std::make_move_iterator(arc_it->end()));
	}
	
	// Archive clustering is needed to reduce the amount of archive solutions.
	archive_clustering(pareto_archive, true);

}

/**
 * @brief  Function performed by a single thread during the archive initialization procedure.
 *
 * @details
 * In case a non-trivial solution of the problem is available, even far from the optimal one, it is possible to start 
 * the algorithm from that solution. In this case the archive is initially built from this solution, instead of random
 * solutions. Once the archive is built from this solution, the algorithm continues normally.
 * 
 * The baseline solution is refined using hill-clinbimg, accepting a new solution only if dominated the previous one. 
 * Hill-climbing is performed a fixed amount of iterations. Thereafter, the non-dominated solutions obtained with 
 * hill-climbing are stored in the archive up to a maximum equals to the archive size hard limit. 
 * In case the amount of non dominated solutions excedes the size limit, clustering is applied.
 *
 * @param baseline_solution The starting solution 
 * @param solutions amount of solutions the thread has to generate
 * @param private_archive_it private archive for the thread
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::initialize_archive_baseline_single_thread(
		const amosa_point_t& baseline_solution, 
		unsigned solutions, 
		typename std::vector<std::vector<amosa_point_t>>::iterator private_archive_it)
{
	for (unsigned i = 0; i < solutions; i++)
	{
		amosa_point_t candidate_solution = baseline_solution.perturbate();

		// Performing hill-climbing
		for (unsigned int j = 0; j < num_iteration; j++)
		{
			// generating a new solution
			amosa_point_t new_solution = candidate_solution.neighbor();
			if (dominates(new_solution, candidate_solution))
				candidate_solution = std::move(new_solution);
		}

		// After hill-climbing, candidate solution are always inserted in the archive. The archive clustering will
		// eventually remove duplicate or dominated solutions.
		private_archive_it->push_back(candidate_solution);
	}
}

/**
 * @brief Archive clustering
 * 
 * @details
 * Archive clustering will eventually remove duplicate or dominated solutions from the archive, in order to reduce the
 * amount of archived solutions and to met the archive size hard limit.
 * 
 * For each of the solutions, the distance is calculated with all other solutions. If the distance is less than a
 * threshold, the considered solution is removed from the archive.
 * 
 * The threshold is dynamically computed as the average distance between the archived solutions.
 * 
 * @param archive archive to be clustered
 * @param force force clusterization, even if the size of the archive is less than the soft limit
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::archive_clustering(std::vector<amosa_point_t>& archive, bool force)
{
	unsigned count = 1, max_try = 10;
	if (force || archive.size() > archive_size_soft_limit)
		while ((archive.size() > archive_size_hard_limit) && (count < max_try))
		{
			
			// the threshold is computed
			double threshold = compute_clustering_threshold(archive) * count++;
			
			// the archive is scanned
			typename std::vector<amosa_point_t>::iterator it = archive.begin(), jt = std::next(it);
			
			// NOTE the archive.size() > archive_size_hard_limit has been added in order to avoid a complete destruction of
			// the archive
			while (archive.size() > archive_size_hard_limit && it != archive.end() && jt != archive.end())
					if (it->distance(*jt) < threshold)
						// if the distance between two solutions is less than the
						// thresholt, a solution is removed from the archive
						jt = archive.erase(jt);
					else
						jt++;
		}
}

/**
 * @brief Compute the average distance between archived solutions.
 * 
 * @return double  the average distance between archived solutions.
 */
template<typename amosa_point_t>
double optimization_engine_t<amosa_point_t>::compute_clustering_threshold(const std::vector<amosa_point_t>& archive) const
{
	double distance = 0;
	typename std::vector<amosa_point_t>::const_iterator arc_it = archive.cbegin(), arc_end = archive.cend(), jt;
	for (; arc_it != arc_end; arc_it++)
		for (jt = std::next(arc_it); jt != arc_end; jt++)
			distance += arc_it->distance(*jt);
	return distance / archive.size();
}

/**
 * @brief Removes eventually dominated solutions from the archive.
 * 
 * @param solution supposed dominant solution.
 * @return unsigned amount of removed solutions
 */
template<typename amosa_point_t>
unsigned optimization_engine_t<amosa_point_t>::remove_dominated_archived_solutions(
	const amosa_point_t& solution,
	std::vector<amosa_point_t>& archive)
{
	unsigned count = 0;
	typename std::vector<amosa_point_t>::const_iterator arc_it = archive.cbegin();
	while (arc_it != archive.cend())
	{
		if (dominates(solution, *arc_it))
		{
			count++;
			arc_it = archive.erase(arc_it);
		}
		else
			arc_it++;
	}
	return count;
}

/**
 * @brief Computes deltas needed by the optimization cycle
 * 
 * @param new_pt candidate solution
 * @param current_pt current solution
 * @param min minimum delta dominance
 * @param avg average delta dominance
 * @param avg_ci average delta dominance, including the current solution
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::amounts_of_domination(
	const std::vector<amosa_point_t>& archive,
	const amosa_point_t& new_pt,
	const amosa_point_t& current_pt,
	double& min,
	double&avg,
	double&avg_ci) const
{
	std::vector<double> R;
	unsigned count = 0;
	min = std::numeric_limits<double>::max();
	avg = 0;
	avg_ci = 0;

	compute_ranges(archive, R);
	typename std::vector<amosa_point_t>::const_iterator it = archive.cbegin(), it_end = archive.cend();
	for (; it != it_end; it++)
		if (dominates(*it, new_pt))
		{
			double amount = amount_of_domination(*it, new_pt, R);
			avg += amount;
			if (min > amount)
				min = amount;
			count++;
		}
	
	avg_ci = (avg + amount_of_domination(current_pt, new_pt, R)) / (count + 1);
	avg /= count;
}


/**
 * @brief Verifies the relationship of dominance between two points of the solution space.
 * 
 * @param solution_a a point of the solution space
 * @param solution_b a point of the solution space
 * 
 * @return true if solution_a dominates solution_b
 * @return false if solution_a does not dominate solution_b
 */
template<typename amosa_point_t>
bool optimization_engine_t<amosa_point_t>::dominates(
	const amosa_point_t & solution_a,
	const amosa_point_t & solution_b) const
{
	std::vector<double> ff_values_a, ff_values_b;
	solution_a.get_fitness_values(ff_values_a);
	solution_b.get_fitness_values(ff_values_b);

	std::vector<double>::const_iterator a_it, a_end = ff_values_a.cend(), b_it, b_end = ff_values_b.cend();
	for (a_it = ff_values_a.cbegin(), b_it = ff_values_b.cbegin(); a_it != a_end, b_it != b_end; a_it++, b_it++)
		if (*a_it > *b_it)
			return false;

	for (a_it = ff_values_a.cbegin(), b_it = ff_values_b.cbegin(); a_it != a_end, b_it != b_end; a_it++, b_it++)
		if (*a_it < *b_it)
			return true;

	return false;
}

/**
 * @brief Computes the amount of domination between two different solutions
 * 
 * @tparam amosa_solution_t 
 * @param solution_a first solution
 * @param solution_b second solution
 * @param R ranges of the objective functions.
 * @return double the amount of domination of solution_a on solution_b, 
 */
template<typename amosa_point_t>
double optimization_engine_t<amosa_point_t>::amount_of_domination (
	const amosa_point_t& solution_a, 
	const amosa_point_t& solution_b,
	const std::vector<double>& R) const
{
	double domination_amount = 1;
	std::vector<double> ff_values_a, ff_values_b;
	solution_a.get_fitness_values(ff_values_a);
	solution_b.get_fitness_values(ff_values_b);

	std::vector<double>::const_iterator a_it = ff_values_a.cbegin(), a_end = ff_values_a.cend();
	std::vector<double>::const_iterator b_it = ff_values_b.cbegin(), b_end = ff_values_b.cend();
	std::vector<double>::const_iterator R_it = R.cbegin(), R_end = R.cend();

	for (; a_it != a_end, b_it != b_end, R_it != R_end; a_it++, b_it++, R_it++)
		if (*a_it != *b_it)
			domination_amount *= (fabs(*a_it - *b_it) / *R_it);

	return domination_amount;
}

/**
 * @brief Compute the ranges of fitness-functions, taking only archived solutions into account
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::compute_ranges(
	const std::vector<amosa_point_t>& archive,
	std::vector<double>& R) const
{
	// Computing maximum and minimum value for each fitness function value
	// Initially, maximum and minimum are supposed to be equal to the first solution from the archive
	std::vector<double> minimums, maximums;
	archive.cbegin()->get_fitness_values(minimums);
	archive.cbegin()->get_fitness_values(maximums);

	// The whole archive is, then, scanned, updating the minimum and the maximum values for each
	// fitness function.
	typename std::vector<amosa_point_t>::const_iterator arc_it = std::next(archive.cbegin()), arc_end = archive.cend();
	for (; arc_it != arc_end; arc_it++)
	{
		std::vector<double> ff_values;
		arc_it->get_fitness_values(ff_values);
		std::vector<double>::iterator ff_it = ff_values.begin(), ff_end = ff_values.end();
		std::vector<double>::iterator min_it = minimums.begin(), min_end = minimums.end();
		std::vector<double>::iterator max_it = maximums.begin(), max_end = maximums.end();

		for (;ff_it != ff_end, min_it != min_end, max_it != max_end; ff_it++, min_it++,	max_it++)
		{
			if (*min_it > *ff_it) *min_it = *ff_it;
			if (*max_it < *ff_it) *max_it = *ff_it;
		}
	}
	// Ranges are computed accordingly with minimum and maximum values
	R.erase(R.begin(), R.end());
	std::vector<double>::iterator min_it = minimums.begin(), min_end = minimums.end();
	std::vector<double>::iterator max_it = maximums.begin(), max_end = maximums.end();
	for(; min_it != min_end, max_it != max_end; min_it++, max_it++)
		R.push_back(*max_it - *min_it);
}

/**
 * @brief Returns true if a new solution should be accepted.
 * 
 * @details
 * Returns true if a new solution should be accepted. The probability of accepting the solution is given by equation
 * (2) or (3) from [1].
 * 
 * @param delta_dominance the amount of dominance of the considered new solution
 * @param temperature current temperature
 * 
 * @return true if the solution should be accepted
 * @return false  if the solution should not be accepted
 */
template<typename amosa_point_t>
bool optimization_engine_t<amosa_point_t>::accept_new_solution(double delta_dominance, double temperature) const
{
	double acceptance_probability = 1 / (1 + exp(delta_dominance * temperature));

	std::default_random_engine rand_engine;
	std::bernoulli_distribution bernullian(acceptance_probability);
	return bernullian(rand_engine);
}

/**
 * @brief Returns true if a new solution should be accepted.
 * 
 * @details
 * Returns true if a new solution should be accepted.
 * 
 * @param delta_dominance the amount of dominance of the considered new solution
 * 
 * @return true if the solution should be accepted
 * @return false  if the solution should not be accepted
 */
template<typename amosa_point_t>
bool optimization_engine_t<amosa_point_t>::accept_new_solution(double delta_dominance) const
{
	double acceptance_probability = 1 / (1 + exp(-delta_dominance));
	std::default_random_engine rand_engine;
	std::bernoulli_distribution bernullian(acceptance_probability);
	return bernullian(rand_engine);
}

/**
 * @brief Asd a new candidate solution to the archive
 * 
 * @note It manages removal of dominated solutions and clustering.
 * 
 * @param candidate_solution Candidate solution to be added
 */
template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::add_candidate_solution(
	std::vector<amosa_point_t>& archive,
	const amosa_point_t& candidate_solution)
{
	remove_dominated_archived_solutions(candidate_solution, archive);
	archive.push_back(candidate_solution);
	archive_clustering(archive);
}

template<typename amosa_point_t>
void optimization_engine_t<amosa_point_t>::run_single_thread(
	typename std::vector<std::vector<amosa_point_t>>::iterator archive_it,
	double current_temperature)
{
	std::vector<amosa_point_t>& archive = *archive_it;
	amosa_point_t current; 
		
	// picking a random solution from the archive.
	std::random_device rd;
	std::default_random_engine dre(rd());
	current = archive[dre() % archive.size()];
	
	for (unsigned int i = 0; i < num_iteration; i++)
	{
		// Generation of a new perturbed candidate solution
		amosa_point_t candidate_next = current.perturbate();
		
		// deltas are pre-computer in order to improve parallel computing
		double delta_dom_avg_inc_current, delta_dom_min, delta_dom_avg;
		amounts_of_domination(archive, candidate_next, current, delta_dom_min, delta_dom_avg, delta_dom_avg_inc_current);
		
		/* Based on the dominance status between *current and *candidate_next, three scenarios may arise:
		 * 1) 	If the current candidate solution and k > 0 archived solutions dominate the newly generated
		 * 		solution, the new solution is selected as the current candidate solution with a probability
		 * 		given from (2) -- see the referenced paper [1].
		 * 
		 * 2)	The new solution dominates the current solution.
		 * 		Based on the domination status between the new solution and the archived solutions, three
		 * 		scenario may arise:
		 * 		i) 		the new solution is dominated by k > 0 different solutions from the	archive. In this
		 * 				case, the new solution is selected as the current candidate solution with a probability
		 * 				given by the minimum amount of domination; 
		 * 				NOTE: min_amount_of_dominantion() returns a domination amount different from the largest
		 * 				double number only if the new solution is not dominated by any archive solutions.
		 * 		ii)		the new solution is non-dominating w.r.t the other archived solutions; the new solution
		 * 				is selected as the current candidate solution; in addition, it has to be added to the
		 * 				archive; if the current candidate soluion is in the archive, it must be removed from it.
		 * 		iii)	the new solution dominates k > 0 archived solutions; dominated solutions have to be
		 * 				removed from the archive, the new solution is selected as the current candidate
		 * 				solution; in addition, it has to be added to the archive.
		 * 
		 * 	3)	The candidate and new solutions are non-dominating with each other. Based on the domination 
		 * 		status between the new solution and the archived solutions, three scenario may arise:
		 * 		i) 		the new solution is dominated by k > 0 different solutions from the	archive. In this
		 * 				case, the new solution is selected as the current candidate solution with a probability
		 * 				given by equation (3) from [1].
		 * 				NOTE: avg_amount_of_dominantion() returns a domination amount greather than zero only if
		 * 				the new solution is dominated by at least an archive solution
		 * 		ii)		the new solution is non-dominating w.r.t the other archived solutions; the new solution
		 * 				is selected as the current candidate solution; in addition, it has to be added to the
		 * 				archive.
		 * 		iii)	the new solution dominates k > 0 archived solutions; dominated solutions have to be
		 * 				removed from the archive, the new solution is selected as the current candidate
		 * 				solution; in addition, it has to be added to the archive.
		 */
		
		// Case 1
		if (dominates(current, candidate_next) && accept_new_solution(delta_dom_avg_inc_current, current_temperature))
			current = candidate_next;
		else
		{
			// Case 2 - i
			if (dominates(candidate_next, current) && (delta_dom_min != std::numeric_limits<double>::max()) && accept_new_solution(delta_dom_min))
				current = candidate_next;
			else
			{
				// Case 3 i
				if ((delta_dom_avg > 0) && accept_new_solution(delta_dom_avg, current_temperature))
					current = candidate_next;
				else
				{	
					// Case 2 ii, 2 iii, 3 ii and 3 iii
					add_candidate_solution(archive, candidate_next);
					current = candidate_next;
				}
			}
		}
	}
}

} /* amosa namespace */

/**
 * @}
 */

#endif
