#include "PolynomialInterpolator.h"

#include "iostream"

PolynomialInterpolator::PolynomialInterpolator()
{
  // degree = 5; // 5次多項式で補間
  // num_conditions = 3; // 位置、速度、加速度の条件

  degree = 3; // 3次多項式で補間
  num_conditions = 2; // 位置、速度の条件

  A.resize(2*num_conditions, degree+1); // 初期条件と終端条件で2倍
  b.resize(2*num_conditions);

  A.setZero();
  b.setZero();
}

Eigen::VectorXd PolynomialInterpolator::calcCoefficients
(const BoundaryConditions& start_boundary_conditions, const BoundaryConditions& end_boundary_conditions, double tf) {
  // t0 = 0 としている
  A <<
    1.0, 0.0,   0.0,      0.0,
    0.0, 1.0,   0.0,      0.0,
    1.0,  tf, tf*tf, tf*tf*tf,
    0.0, 1.0,  2*tf,   3*tf*tf;
  
  b(0) = start_boundary_conditions.val;
  b(1) = start_boundary_conditions.first_der;
  b(2) = end_boundary_conditions.val;
  b(3) = end_boundary_conditions.first_der;

  Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(b);

  std::cout << "theta: " << coefficients << std::endl;
  
  return coefficients;
}
