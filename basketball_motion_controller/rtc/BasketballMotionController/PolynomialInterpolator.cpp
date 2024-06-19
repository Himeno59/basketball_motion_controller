#include "PolynomialInterpolator.h"

#include "iostream"

PolynomialInterpolator::PolynomialInterpolator()
{
  degree = 5; // 5次多項式で補間
  num_conditions = 3; // 位置、速度、加速度の条件

  A.resize(2*num_conditions, degree+1); // 初期条件と終端条件で2倍
  b.resize(2*num_conditions);

  A.setZero();
  b.setZero();
}

Eigen::VectorXd PolynomialInterpolator::calcCoefficients
(const BoundaryConditions& start_boundary_conditions, const BoundaryConditions& end_boundary_conditions, double tf) {
  
  double t_values[2] = {0.0, tf}; // t0 = 0 としている

  // A
  for (int i=0; i<2; i++) {
    double t = t_values[i];    
    // f(t)
    A(i*3, 0) = 1.0;
    for (int j = 1; j < 6; ++j) {
      A(i*3, j) = pow(t, j);
    }    
    // f'(t)
    A(i*3 + 1, 1) = 1.0;
    for (int j = 2; j < 6; ++j) {
      A(i*3 + 1, j) = j * pow(t, j - 1);
    }
    // f''(t)
    A(i*3 + 2, 2) = 2.0;
    for (int j = 3; j < 6; ++j) {
      A(i*3 + 2, j) = j * (j - 1) * pow(t, j - 2);
    }
  }

  // b
  b(0) = start_boundary_conditions.val;
  b(1) = start_boundary_conditions.first_der;
  b(2) = start_boundary_conditions.second_der;
  b(3) = end_boundary_conditions.val;
  b(4) = end_boundary_conditions.first_der;
  b(5) = end_boundary_conditions.second_der;

  Eigen::VectorXd coefficients = A.colPivHouseholderQr().solve(b);

  std::cout << "theta: " << coefficients << std::endl;
  
  return coefficients;
}
