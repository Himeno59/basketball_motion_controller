#include "PolynomialInterpolator.h"

PolynomialInterpolator::PolynomialInterpolator()
{
  degree = 5; // 5次多項式で補間
  num_conditions = 3; // 位置、速度、加速度の条件

  A.resize(2*num_conditions, degree+1);
  b.resize(2*num_conditions);

  A.setZero();
  b.setZero();
}

Eigen::VectorXd PolynomialInterpolator::calcCoefficients
(const BoundaryConditions& start_boundary_condisions, const BoundaryConditions& end_boundary_condisions, double tf) {
  // 始点の条件
  for (int i = 0; i <= degree; ++i) {
    A(0, i) = pow(0.0, i);
    if (i > 0) A(1, i) = i * pow(0.0, i - 1);
    if (i > 1) A(2, i) = i * (i - 1) * pow(0.0, i - 2);
  }
  b(0) = start_boundary_conditions.val;
  b(1) = start_boundary_conditions.first_der;
  b(2) = start_boundary_conditions.second_der;

  // 終点の条件
  for (int i = 0; i <= degree; ++i) {
    A(3, i) = pow(tf, i);
    if (i > 0) A(4, i) = i * pow(tf, i - 1);
    if (i > 1) A(5, i) = i * (i - 1) * pow(tf, i - 2);
  }
  b(3) = end_boundary_conditions.val;
  b(4) = end_boundary_conditions.first_der;
  b(5) = end_boundary_conditions.second_der;

  Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(b);
  
  return coefficients;
}
