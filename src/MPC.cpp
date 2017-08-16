#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 5;
double dt = 0.3;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 50 mph.
double ref_v = 40*1.609e3/3600;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // : implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // fg[0] is the cost
    fg[0] = 0;

    // First part of the cost is based on the reference state
    for (int t=0; t<N; ++t) {
      fg[0] += 1000*CppAD::pow(vars[cte_start+t], 2);      // penalise CTE
      fg[0] += 100*CppAD::pow(vars[epsi_start+t], 2);     // penalise misalignment
      fg[0] += 0.3*CppAD::pow(vars[v_start+t]-ref_v, 2);  // penalise speed error
    }

    // Minimize the use of actuators to avoid large commanding
    for (int t=0; t<N-1; t++) {
      fg[0] += 0.01*CppAD::pow(vars[delta_start+t], 2);    // penalise large wheel command
      fg[0] += 0.01*CppAD::pow(vars[a_start+t], 2);        // penalise large accelerations

    }

    // Minimise actuator change to avoid large back and forth commanding
    for (int t=0; t<N-2; t++) {
      fg[0] += 1*CppAD::pow(vars[delta_start+t+1]-vars[delta_start+t], 2);   // large wheel command change
      fg[0] += 0.01*CppAD::pow(vars[a_start+t+1]-vars[a_start+t], 2);           // large acceleration change
    }


    // Setup constraint (= model constraint)

    // Initial constraints
    //
    // +1 because fg[0] is the cost, everything else is offseted by 1
    
    // initial state
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // model of the car
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      const AD<double> x1 = vars[x_start + t];
      const AD<double> y1 = vars[y_start + t];
      const AD<double> psi1 = vars[psi_start + t];
      const AD<double> v1 = vars[v_start + t];
      const AD<double> cte1 = vars[cte_start + t];
      const AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      const AD<double> x0 = vars[x_start + t - 1];
      const AD<double> y0 = vars[y_start + t - 1];
      const AD<double> psi0 = vars[psi_start + t - 1];
      const AD<double> v0 = vars[v_start + t - 1];
      const AD<double> cte0 = vars[cte_start + t - 1];
      const AD<double> epsi0 = vars[epsi_start + t - 1];

      const AD<double> delta0 = vars[delta_start + t - 1];
      const AD<double> a0 = vars[a_start + t - 1];
      
      // equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
       
      // Equations for the errors
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      AD<double> f0 = coeffs[0] + coeffs[1]  * x0 + coeffs[2] * pow(x0,2) + coeffs[3] * pow(x0,3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 *coeffs[3]* pow(x0, 2));
      fg[1 + cte_start + t] =
              cte1 - ((f0 - y0) - (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
              epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
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
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // initial state in variables
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // Done: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  // Here we have 6 variables and 2 actuatores, an N steps:
  size_t n_vars = 6*N + 2*(N-1);
  // Done: Set the number of constraints
  size_t n_constraints = 6*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Initial value of the variables
  //vars[x_start] = x;
  //vars[y_start] = y;
  //vars[psi_start] = psi;
  //vars[v_start] = v;
  //vars[cte_start] = cte;
  //vars[epsi_start] = epsi;
  
  // Lower and upper limits vor x 
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.
  // All limits are set to max positive and negatives values
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -5 and 5
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    double max_delta = 25.;
    vars_lowerbound[i] = -max_delta/180.*M_PI;
    vars_upperbound[i] =  max_delta/180.*M_PI;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    double max_a = 1.;
    vars_lowerbound[i] = -max_a;
    vars_upperbound[i] =  max_a;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
 
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



  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
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

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // And the N predicted points also
  //
  vector<double> result;
//  result.push_back((solution.x[delta_start]+solution.x[delta_start+1])/2);
//  result.push_back((solution.x[a_start]+solution.x[a_start+1])/2);

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (int t=0; t<N-1; ++t) {
    result.push_back(solution.x[x_start+t+1]);
    result.push_back(solution.x[y_start+t+1]);
  }

  return result;
}
