#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Define return struct
struct Result {
	vector<double> steering;
	vector<double> acceleration;
	vector<double> X;
	vector<double> Y;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.

  Result Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

	// add previous steering & throttle actuators
	double delta_prev{ 0 };
	double a_prev{ 0.1 };
};

#endif /* MPC_H */
