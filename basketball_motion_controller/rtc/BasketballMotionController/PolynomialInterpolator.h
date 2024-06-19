#ifndef POLYNOMIAL_INTERPOLATOR_H
#define POLYNOMIAL_INTERPOLATOR_H

#include <Eigen/Dense>

struct BoundaryConditions {
  double val;
  double first_der;
  double second_der;
};

class PolynomialInterpolator {
public:  
  PolynomialInterpolator();
  Eigen::VectorXd calcCoefficients(const BoundaryConditions& start_boundary_condisions, const BoundaryConditions& end_boundary_condisions, double tf);
  
private:
  int degree;
  int num_conditions;
  double tf;
  BoundaryConditions start_boundary_conditions;
  BoundaryConditions end_boundary_conditions;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
    
};

#endif // POLYNOMIAL_INTERPOLATOR_H
