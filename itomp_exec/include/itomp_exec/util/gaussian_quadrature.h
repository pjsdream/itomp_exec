#ifndef ITOMP_EXEC_GAUSSIAN_QUADRATURE_H
#define ITOMP_EXEC_GAUSSIAN_QUADRATURE_H


#include <ecl/geometry/polynomial.hpp>


namespace itomp_exec
{

double gaussianQuadratureQuadraticPolynomial(double t0, double t1, const ecl::QuadraticPolynomial& poly);

}


#endif // ITOMP_EXEC_GAUSSIAN_QUADRATURE_H
