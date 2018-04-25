#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

#include "globals.h"
using CppAD::AD;


//No protype for FG_eval class

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
	  // implemention of MPC
	  // `fg` a vector of the cost constraints,
	  //`vars` is a vector of variable values (state & actuators)

	  /**********************************
	   *
	   * COST Definition
	   **********************************/
	  // The cost is stored is the first element of `fg`.
	  // Any additions to the cost is added to `fg[0]`.
	  fg[0] = 0.0;

	  // Reference State Cost
	  // : Define the cost related the reference state

	  // The part of the cost based on the reference state.
	  for (size_t t = 0; t < N; t++) {
		  fg[0] += cte_w*CppAD::pow(vars[cte_start + t]-ref_cte, 2);
		  fg[0] += epsi_w*CppAD::pow(vars[epsi_start + t]-ref_epsi, 2);
		  fg[0] += v_w*CppAD::pow(vars[v_start + t] - ref_v, 2);
		  //penalize velocity for high cte
		  fg[0] += cte_v_w*CppAD::pow(vars[cte_start + t]*vars[v_start + t], 2);
	  }

	  // Minimize the use of actuators.
	  for (size_t t = 0; t < N - 1; t++) {
		  fg[0] += delta_w*CppAD::pow(vars[delta_start + t], 2);
		  fg[0] += a_w*CppAD::pow(vars[a_start + t], 2);
		  fg[0] += delta_v_w*CppAD::pow(vars[delta_start + t] * (vars[v_start + t]), 2);
		  //fg[0] += delta_v_w*CppAD::pow(vars[delta_start + t] * (vars[v_start + t]-ref_v), 2); //
	  }

	  // Minimize the value gap between sequential actuations.
	  for (size_t t = 0; t < N - 2; t++) {
		  fg[0] += delta_s_w*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
		  fg[0] += a_s_w*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	  }


	  /**********************************
	   *
	   * Ipopt Constraint Setup
	   **********************************/
	  // Setup Constraints
	  //
	  // In this section model constraints are setup.

	  // Initial constraints
	  //
	  // We add 1 to each of the starting indices due to cost being located at
	  // index 0 of `fg`.
	  // This bumps up the position of all the other values.
	  // Initial value of state variables shall be starting value
	  fg[1 + x_start] = vars[x_start];
	  fg[1 + y_start] = vars[y_start];
	  fg[1 + psi_start] = vars[psi_start];
	  fg[1 + v_start] = vars[v_start];
	  fg[1 + cte_start] = vars[cte_start];
	  fg[1 + epsi_start] = vars[epsi_start];

	  // The rest of the constraints
	  for (size_t t = 1; t < N; t++) {

		  //next state
		  AD<double> x1 = vars[x_start + t];
		  AD<double> y1 = vars[y_start + t];
		  AD<double> psi1 = vars[psi_start + t];
		  AD<double> v1 = vars[v_start + t];
		  AD<double> cte1 = vars[cte_start + t];
		  AD<double> epsi1 = vars[epsi_start + t];

		  //current state
		  AD<double> x0 = vars[x_start + t - 1];
		  AD<double> y0 = vars[y_start + t - 1];
		  AD<double> psi0 = vars[psi_start + t - 1];
		  AD<double> v0 = vars[v_start + t - 1];
		  AD<double> cte0 = vars[cte_start + t - 1];
		  AD<double> epsi0 = vars[epsi_start + t - 1];


		  // consider the actuation at current time.
		  AD<double> delta0 = vars[delta_start + t - 1];
		  AD<double> a0 = vars[a_start + t - 1];






		  //find desired y position and psi
		  AD<double> f0 =0.0;
		  AD<double> psides0=0.0;

		  //f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0*x0 +coeffs[3] * x0*x0*x0  ;
		  //f' -->slope
		  //psides0 = CppAD::atan(coeffs[1]+2.0*coeffs[2]*x0+3.0*coeffs[3]*x0*x0);


		  switch(degree) {
		  //degree 0
		  //degree 1, line
		  //f0 represents desired y position py(from polynomila eqn using x0)
		  //psides0 represents desired psi, arctan of slope of poly
		  case 1:
			  f0 = coeffs[0] + coeffs[1] * x0;
			  //f' -->slope
			  psides0 = CppAD::atan(coeffs[1]);
			  break;

		  //degree 2 , quadratic
		  case 2:
			  f0 = coeffs[0] + coeffs[1] * x0+coeffs[2] * x0*x0;
			  //f' -->slope
			  psides0 = CppAD::atan(coeffs[1]+2.0*coeffs[2]*x0);
			  break;

		  //degree 3
		  case 3:
			  f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0*x0 +coeffs[3] * x0*x0*x0  ;
			  //f' -->slope
			  psides0 = CppAD::atan(coeffs[1]+2.0*coeffs[2]*x0+3.0*coeffs[3]*x0*x0);
			  //cout <<"deg 3"<<endl;

			  break;
		  //degree 4
		  case 4:
			  f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0*x0 +coeffs[3] * x0*x0*x0 + coeffs[4] * x0*x0*x0*x0 ;
			  //f' -->slope
			  psides0 = CppAD::atan(coeffs[1]+2.0*coeffs[2]*x0+3.0*coeffs[3]*x0*x0 + 4.0*coeffs[4]*x0*x0*x0);


			  break;



		  default :
			  cerr << "Error: Polynomial degree has to be betwen 1-3: " << degree<<endl;
			  exit(EXIT_FAILURE);


		  }



		  // The idea here is to constraint this value to be 0.
		  // so model constraints are essentially, state param next-current=0
		  // NOTE: The use of `AD<double>` and use of `CppAD`!
		  // This is also CppAD can compute derivatives and pass
		  // these to the solver.
		  // Note: '-' in psi0 + (v0/ Lf) * -(delta0) * dt
		  // delta is positive we rotate counter-clockwise, or turn left.
		  //In the simulator however, a positive value implies a right turn
		  //and a negative value implies a left turn, so delta here is used as -delta
		  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
		  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
		  fg[1 + psi_start + t] = psi1 - (psi0 + (v0/ Lf) * (delta0) * dt);
		  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
		  fg[1 + cte_start + t] =
				  cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		  fg[1 + epsi_start + t] =
				  epsi1 - ((psi0 - psides0) + (v0/ Lf) * (delta0)  * dt);

	  }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	bool ok = true;
	//size_t i;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	/*****************
	 *
	 * set number of values to calc for MPC model
	 ******************/

	// Set the number of model variables (includes both states and inputs).
	// For example: If the state is a 4 element vector, the actuators is a 2
	// element vector and there are 10 timesteps. The number of variables is:
	//
	// 4 * 10 + 2 * 9
	//size_t n_vars = 0;
	// Set the number of constraints
	//size_t n_constraints = 0;

	// number of independent variables
	// N timesteps == N - 1 actuations
	//size_t n_vars = N * 6 + (N - 1) * 2;
	size_t n_vars = N * nb_state_vars + (N - 1) * nb_actuators;
	// Number of constraints
	//size_t n_constraints = N * 6;
	size_t n_constraints = N * nb_state_vars;


	/************************
	 *
	 * Initialize state variables
	 ***********************/
	//store state variables
	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];


	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	//vars contains value of state and actuator values

	Dvector vars(n_vars);
	for (size_t i = 0; i < n_vars; i++) {
		vars[i] = 0;
	}

	// Set the initial variable values
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;

	/**************************
	 *
	 * Set bounds on state var and model constraints
	 **************************/

	//set acceptable values(lower/upper bounds) for variables
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);
	// Set lower and upper limits for variables.
	// Set all non-actuators upper and lowerlimits
	// to the max negative and positive values.
	for (size_t i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}

	// The upper and lower limits of delta are set to -25 and 25
	// degrees (values in radians).
	// These values shall be experimented with.0.436332=2*PI/360 *25
	for (size_t i = delta_start; i < a_start; i++) {
		vars_lowerbound[i] = -0.436332*Lf;
		vars_upperbound[i] = 0.436332*Lf;
	}

	// Acceleration/decceleration upper and lower limits.
	// These values shall be experimented with, -1 brake, 1 full throttle
	for (size_t i = a_start; i < n_vars; i++) {
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}


	//set acceptable values(lower/upper bounds) for constraints
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	//init contstraints bounds to 0
	for (size_t i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}
	//first value of contraints shall be initial values of state vars
	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;

	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;

	/************************
	 *
	 * Ipopt Solve
	 ************************/

	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";
	//options += "Numeric max_cpu_time          1.0\n";
	//options += "Numeric max_cpu_time          5.0\n";
	//options += "Numeric max_cpu_time          10.0\n";

	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	/*
	 * Syntax
  	  *# include <cppad/ipopt/solve.hpp>
  	  *ipopt::solve(
      * options, xi, xl, xu, gl, gu, fg_eval, solution
  	  *)

  	  *Purpose
  	  *The function ipopt::solve solves nonlinear programming problems of the form

	 */


	CppAD::ipopt::solve<Dvector, FG_eval>(
			options,
			vars,
			vars_lowerbound,
			vars_upperbound,
			constraints_lowerbound,
			constraints_upperbound,
			fg_eval, solution);

	// Check some of the solution values (dbg)
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
	if(ok) {
		// Cost
		auto cost = solution.obj_value;
		std::cout << "Success: Cost: " << cost << std::endl;
	} else {
		std::cout << " Ipopt solver failed!!"<<std::endl;
	}


	/*******************************
	 * Return values
	 *******************************/
	// Return the first actuator values. The variables can be accessed with
	// `solution.x[i]`.
	//
	// {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
	// creates a 2 element double vector.
	vector<double> result;
	result.push_back(solution.x[delta_start]);
	result.push_back(solution.x[a_start]);

	//also return px and py values from MPC model for use in mpc_x_vals/mpc_y_vals
	for(size_t i=0;i<N;i++){
		result.push_back(solution.x[x_start+i]);
		result.push_back(solution.x[y_start+i]);
	}
	return result;
}
