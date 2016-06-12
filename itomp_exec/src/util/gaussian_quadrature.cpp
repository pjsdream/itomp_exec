#include <itomp_exec/util/gaussian_quadrature.h>

namespace itomp_exec
{

static double gaussian_quadrature2_weights_[3] =
{
    0.8888888888888888,
    0.5555555555555556,
    0.5555555555555556,
};

static double gaussian_quadrature2_abscissa_[3] =
{
    0.0000000000000000,
    -0.7745966692414834,
    0.7745966692414834,
};

double gaussianQuadratureQuadraticPolynomial(double t0, double t1, const ecl::QuadraticPolynomial& poly)
{
    const double mid = (t0 + t1) * 0.5;
    const double radius = (t1 - t0) * 0.5;
    
    return (t1-t0) * 0.5 * (
                  gaussian_quadrature2_weights_[0] * poly( mid + radius * gaussian_quadrature2_abscissa_[0] )
                + gaussian_quadrature2_weights_[1] * poly( mid + radius * gaussian_quadrature2_abscissa_[1] )
                + gaussian_quadrature2_weights_[2] * poly( mid + radius * gaussian_quadrature2_abscissa_[2] )
            );
}

double gaussianQuadratureWeight2(int i)
{
    return gaussian_quadrature2_weights_[i];
}

double gaussianQuadratureAbscissa2(int i)
{
    return gaussian_quadrature2_abscissa_[i];
}

}
