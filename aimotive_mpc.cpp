/**
 * My Solution to the AImotive planning assignement 
 *
 * Copyright 2018 by Ziad ZAMZAMI <zamzami@isir.upmc.fr>
 *
 * This file is part of my Solution to the AImotive planning assignement.
 * 
 * You can redistribute it and/or modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation, either version 3 of the License.
 * 
 * It is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 * See theGNU General Public License for more details.
 *
 * @license GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>
 */

#include "aimotive_mpc.h"

#include <vector>
#include <string>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

/**
 * Implements a Model Predictive Controller (MPC) for a kinematic point mass model of a vehicle, 
 * while taking into consideration the limits on velocity , acceleration and jerk.
 * The current implementation of the MPC controller relies on 
 * the IPOPT solver (Interior Point OPTimizer) and the CppAD (C++ Algorithmic Differentiation) package
 * for solving the optimization problem and calculating the gradients, respectively. 
 */

// Timestep length and duration for the optimization time horizon.
size_t N = 10;
const double dt = 0.1;

// The optimization requires a single vector containing all problem's variables.
// Indexing N elements per state variable and N-1 elements for the actuation,
// considering the vehicle's Jerk as the high order actuation source in this case.
// For the intersection crossing problem we consider only the vehicle's trajectory in 1D.

// x_start is the vehicle's initial position.
// vel_start is the vehicle's initial velocity.
// acc_start is the vehicle's initial acceleration.
// jerk_start is the vehicle's initial jerk.
size_t x_start = 0;
size_t vel_start = x_start + N;
size_t acc_start = vel_start + N;
size_t jerk_start = acc_start + N - 1;
// size_t time_start = jerk_start+ N;

class FG_eval {
 public:
  // the default reference velocity is the maximum admissible velocity.
  double ref_vel = 20;

  // Using the explicit single paramter constructor
  // for passing the referrence velocity to the MPC solver.
  explicit FG_eval(double ref_vel) {this -> ref_vel = ref_vel;}

  // Define CPPAD vector type for storing the variables `vars`,
  // objective function and constraints `fg`.
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the objective and constraints,
    // where the first element `fg[0]` corresponds to the objective function,
    // and subsequent elements correspond the constraints.
    // `vars` is a vector of variable values.

    // Initialize the objective function.
    fg[0] = 0;

    // Add an objective to minimize the difference between
    // the vehicle's velocity and the reference velocity
    // over the planning time horizon.
    for (size_t i = 0; i < N; i++) {
      fg[0] += CppAD::pow(vars[vel_start + i] - ref_vel, 2);
    }

    // Add an objective to minimize the vehicle's acceleration
    // over the planning time horizon.
  //  for (size_t i = 0; i < N ; i++) {
  //   fg[0] += CppAD::pow(vars[acc_start + i], 2);
  //  }

    // Add an objective to minimize the vehicle's jerk
    // over the planning time horizon. [Currently for test purposes only]
  //  for (size_t i = 0; i < N - 1; i++) {
  //   fg[0] += CppAD::pow(vars[jerk_start + i], 2);
  //  }

    // Add an objective to minimize the difference between
    // the vehicle's position and a desired position
    // over the planning time horizon. [Currently for test purposes only]
  // for (size_t i = 0; i < N ; i++) {
  //   fg[0] += CppAD::pow(vars[x_start + i] - 35, 2);
  //  }


    // In this section we add various constraints
    // including state and model constraints.
    // Note that CppAD requires that all constraints
    // to be indexed successively in the `fg` vector.

    // Initial State Constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + vel_start] = vars[vel_start];
    fg[1 + acc_start] = vars[acc_start];
    //fg[1 + jerk_start] = vars[jerk_start];
    // fg[1 + time_start] = vars[time_start];

    // Describing the state variables at time t and at time t+1
    for (size_t t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> v1 = vars[vel_start + t];
      AD<double> v0 = vars[vel_start + t - 1];
      AD<double> a1 = vars[acc_start + t ];
      AD<double> a0 = vars[acc_start + t - 1];
     // Only consider jerk at time t.
      AD<double> jerk = vars[jerk_start + t - 1];
    // AD<double> t1 = vars[time_start + t];
    // AD<double> t0 = vars[time_start + t - 1];



      // Setup the following model constraints in the form of c- g(x) = 0

      // x[t+1] = x[t] + v[t]* dt + 1/2 * (dt)^2 * a0 + 1/3 * (dt)^3 * jerk
      // v[t+1] = v[t] + dt * a0 + 1/2 * (dt)^2 * jerk
      // a[t+1] = a[t] + dt * jerk

      fg[1 + x_start + t] = x1 - (x0 + v0 * dt + 0.5* CppAD::pow(dt, 2) * a0 + 0.166 * CppAD::pow(dt, 3) * jerk);
      fg[1 + vel_start + t] = v1 - (v0 + a0 * dt + 0.5* CppAD::pow(dt, 2) * jerk);
      fg[1 + acc_start + t] = a1 - (a0 + jerk * dt);

      // fg[1 + jerk_start + t] = t1 - (t0 +  dt);
    }
  }
};

// MPC class definition implementation.

AimotiveMpc::AimotiveMpc() {}
AimotiveMpc::~AimotiveMpc() {}

std::vector<double> AimotiveMpc::Solve(
              double x_init,
              double vel_init,
              double acc_init,
              double jerk_init,
              double ref_vel) {
  
  typedef CPPAD_TESTVECTOR(double) Dvector;


  // Number of model variables are
  // (position + velocity + acceleration) * N + Jerk (N-1)
  size_t n_vars = N * 3 + (N - 1) * 1;
  // Set the number of constraints
  size_t n_constraints = N * 3;

  // Initializing values of the independent variables to 0
  // beside the initial state.
  Dvector vars(n_vars);

  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Construct vectors for independant variables lower and upper limits.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set the initial variable values
  //vars[x_start] = x_init;
  //vars[vel_start] = vel_init;
  //vars[acc_start] = acc_init;
  //vars[jerk_start] = jerk_init;

  // vars[time_start] = 0;

  // Set all upper and lowerlimits
  // to the max negative and positive values.

  // Position upper and lower limits.
  for (size_t i = 0; i < vel_start; i++) {
    vars_lowerbound[i] = -40;
    vars_upperbound[i] = 40;
  }

  // Velocity upper and lower limits.
  for (size_t i = vel_start; i < acc_start; i++) {
    vars_lowerbound[i] = -20;
    vars_upperbound[i] = 20;
  }

  // Acceleration/decceleration upper and lower limits.
  for (size_t i = acc_start; i < jerk_start; i++) {
    vars_lowerbound[i] = -2;
    vars_upperbound[i] = 2;
  }

  // Jerk upper and lower limits.
  for (size_t i = jerk_start; i < n_vars; i++) {
    vars_lowerbound[i] = -0.4;
    vars_upperbound[i] = 0.4;
  }

 //   for ( i = time_start; i < n_vars; i++) {
 //   vars_lowerbound[i] = 0.0;
 //   vars_upperbound[i] = 10;
 // }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x_init;
  constraints_lowerbound[vel_start] = vel_init;
  constraints_lowerbound[acc_start] = acc_init;
  //constraints_lowerbound[jerk_start] = jerk_init;
  //constraints_lowerbound[time_start] = 0;



  constraints_upperbound[x_start] = x_init;
  constraints_upperbound[vel_start] = vel_init;
  constraints_upperbound[acc_start] = acc_init;
  //constraints_upperbound[jerk_start] = jerk_init;
  //constraints_upperbound[time_start] = 0;


  // object that computes objective and constraints
  FG_eval fg_eval(ref_vel);

  // options for IPOPT solver
  std::string options;
  // Uncomment next line for more IPOPT solver Info for debugging.
  // options += "Integer print_level  4\n";
  options += "Integer print_level  0\n";
  options += "String sb  yes\n";
  // Maximum number of iterations
  options += "Integer max_iter  1000\n";
  // Numeric tolerance
  options += "Numeric tol  1e-4\n";
  // Derivative testing
  options += "String derivative_test  second-order\n";
  // Setting sparse to true allows the solver to take advantage of sparse routines.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // Maximum time limit of for the solver.
  options += "Numeric max_cpu_time          0.5\n";

  // Solution vector.
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem MPC problem.
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check solution 
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  double Mpc_vel = solution.x[ vel_start + 1 ];
  double Mpc_acc = solution.x[ acc_start + 1 ];
  double Mpc_jerk = solution.x[ jerk_start + 1 ];

  std::vector<double> mpc_result { Mpc_vel, Mpc_acc, Mpc_jerk };

  return mpc_result;
  

  // The rest of the code is for debugging purposes only.
  /*
  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  Dvector Xresult;
  Dvector Vresult;
  Dvector Accresult;
  Dvector Jerkresult;
   Dvector timeresult; 

  result.push_back(solution.x[acc_start]);

   int solution_size = solution.x.size();

  std::cout<<"the size of solution vector is "<<solution_size<<"\n";

  for ( i = 0; i < N; i++) {
    Xresult.push_back(solution.x[x_start + i ]);
  }
    std::cout<<"The Xresult is : "<<Xresult<<"\n";

  for (size_t i = 0; i < N; i++) {
    Vresult.push_back(solution.x[vel_start + i ]);
  }
  std::cout << "The Vresult is : " << Vresult <<"\n";

  for (size_t i = 0; i < N; i++) {
    Accresult.push_back(solution.x[acc_start + i ]);
  }
  std::cout << "The Accresult is : " << Accresult <<"\n";

  for (size_t i = 0; i < N-1; i++) {
    Jerkresult.push_back(solution.x[jerk_start + i ]);
  }
    std::cout << "The Jerkresult is : " << Jerkresult << "\n";

  for ( i = 0; i < N; i++) {
    timeresult.push_back(solution.x[time_start + i ]);
  }
    std::cout<<"The Timeresult is : "<<timeresult<<"\n";

    std::cout<<"The Optimization Cost is : "<<cost<<"\n"; */
  
}


// For debugging only
/* int main (void){

std::cout<<"Please enter initial state\n";
  double x_init; double v_init; double acc_init; double jerk_init; 
  std::cin>>x_init>>v_init>>acc_init>>jerk_init;
  MyMpc mpc;
  const auto result = mpc.Solve(x_init, v_init, acc_init, jerk_init );
  std::cout<<"Main resut is given\n"; //<< result <<"\n";
  //std::cout<<result<<"\n";
  std::cout<<"Debugging\n";

} */
