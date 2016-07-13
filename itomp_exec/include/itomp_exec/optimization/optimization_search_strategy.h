/*
 * Refer to dlib/optimization/optimization_stop_strategies.h
 * max_line_search_iterations was changed
 */

#ifndef ITOMP_EXEC_OPTIMIZATION_SEARCH_STRATEGY_H
#define ITOMP_EXEC_OPTIMIZATION_SEARCH_STRATEGY_H


#include <dlib/optimization.h>


namespace itomp_exec
{

// reduced max_line_search_iterations from 100 to 10
class bfgs_search_strategy
{
public:
    bfgs_search_strategy() : been_used(false), been_used_twice(false) {}

    double get_wolfe_rho (
    ) const { return 0.01; }

    double get_wolfe_sigma (
    ) const { return 0.9; }

    unsigned long get_max_line_search_iterations (
    ) const { return 10; }

    template <typename T>
    const dlib::matrix<double,0,1>& get_next_direction (
        const T& x,
        const double ,
        const T& funct_derivative
    )
    {
        if (been_used == false)
        {
            been_used = true;
            H = dlib::identity_matrix<double>(x.size());
        }
        else
        {
            // update H with the BFGS formula from (3.2.12) on page 55 of Fletcher
            delta = (x-prev_x);
            gamma = funct_derivative-prev_derivative;

            double dg = dot(delta,gamma);

            // Try to set the initial value of the H matrix to something reasonable if we are still
            // in the early stages of figuring out what it is.  This formula below is what is suggested
            // in the book Numerical Optimization by Nocedal and Wright in the chapter on Quasi-Newton methods.
            if (been_used_twice == false)
            {
                double gg = trans(gamma)*gamma;
                if (std::abs(gg) > std::numeric_limits<double>::epsilon())
                {
                    const double temp = dlib::put_in_range(0.01, 100, dg/gg);
                    H = dlib::diagm(dlib::uniform_matrix<double>(x.size(),1, temp));
                    been_used_twice = true;
                }
            }

            Hg = H*gamma;
            gH = dlib::trans(dlib::trans(gamma)*H);
            double gHg = dlib::trans(gamma)*H*gamma;
            if (gHg < std::numeric_limits<double>::infinity() && dg < std::numeric_limits<double>::infinity() &&
                dg != 0)
            {
                H += (1 + gHg/dg)*delta*dlib::trans(delta)/(dg) - (delta*dlib::trans(gH) + Hg*dlib::trans(delta))/(dg);
            }
            else
            {
                H = dlib::identity_matrix<double>(H.nr());
                been_used_twice = false;
            }
        }

        prev_x = x;
        prev_direction = -H*funct_derivative;
        prev_derivative = funct_derivative;
        return prev_direction;
    }

private:
    bool been_used;
    bool been_used_twice;
    dlib::matrix<double,0,1> prev_x;
    dlib::matrix<double,0,1> prev_derivative;
    dlib::matrix<double,0,1> prev_direction;
    dlib::matrix<double> H;
    dlib::matrix<double,0,1> delta, gamma, Hg, gH;
};

}


#endif // ITOMP_EXEC_OPTIMIZATION_SEARCH_STRATEGY_H
