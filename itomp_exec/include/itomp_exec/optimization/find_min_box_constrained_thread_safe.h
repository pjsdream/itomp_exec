/*
 * variant of a function 'dlib::find_min_box_constrained_thread_safe' in dlib/optimization/optimization.h
 * This is a pthread cancel-safe implementation by adding pthread_setcancelstate(...)
 */

#ifndef ITOMP_EXEC_FIND_MIN_BOX_CONSTRAINED_THREAD_SAFE_H
#define ITOMP_EXEC_FIND_MIN_BOX_CONSTRAINED_THREAD_SAFE_H


#include <dlib/optimization/optimization.h>
#include <pthread.h>


namespace itomp_exec
{

template <
    typename search_strategy_type,
    typename stop_strategy_type,
    typename funct,
    typename funct_der,
    typename T,
    typename EXP1,
    typename EXP2
    >
double find_min_box_constrained_thread_safe (
    search_strategy_type search_strategy,
    stop_strategy_type stop_strategy,
    const funct& f,
    const funct_der& der,
    T& x,
    const dlib::matrix_exp<EXP1>& x_lower,
    const dlib::matrix_exp<EXP2>& x_upper
)
{
    /*
        The implementation of this function is more or less based on the discussion in
        the paper Projected Newton-type Methods in Machine Learning by Mark Schmidt, et al.
    */

    // make sure the requires clause is not violated
    COMPILE_TIME_ASSERT(dlib::is_matrix<T>::value);
    // The starting point (i.e. x) must be a column vector.
    COMPILE_TIME_ASSERT(T::NC <= 1);

    DLIB_CASSERT (
        dlib::is_col_vector(x) && dlib::is_col_vector(x_lower) && dlib::is_col_vector(x_upper) &&
        x.size() == x_lower.size() && x.size() == x_upper.size(),
        "\tdouble find_min_box_constrained()"
        << "\n\t The inputs to this function must be equal length column vectors."
        << "\n\t is_col_vector(x):       " << dlib::is_col_vector(x)
        << "\n\t is_col_vector(x_upper): " << dlib::is_col_vector(x_upper)
        << "\n\t is_col_vector(x_upper): " << dlib::is_col_vector(x_upper)
        << "\n\t x.size():               " << x.size()
        << "\n\t x_lower.size():         " << x_lower.size()
        << "\n\t x_upper.size():         " << x_upper.size()
    );
    DLIB_ASSERT (
        dlib::min(x_upper-x_lower) > 0,
        "\tdouble find_min_box_constrained()"
        << "\n\t You have to supply proper box constraints to this function."
        << "\n\r min(x_upper-x_lower): " << dlib::min(x_upper-x_lower)
    );


    T g, s;
    double f_value = f(x);
    g = der(x);

    if (!dlib::is_finite(f_value))
        throw dlib::error("The objective function generated non-finite outputs");
    if (!dlib::is_finite(g))
        throw dlib::error("The objective function generated non-finite outputs");

    // gap_eps determines how close we have to get to a bound constraint before we
    // start basically dropping it from the optimization and consider it to be an
    // active constraint.
    const double gap_eps = 1e-8;

    double last_alpha = 1;
    while(stop_strategy.should_continue_search(x, f_value, g))
    {
        s = search_strategy.get_next_direction(x, f_value, zero_bounded_variables(gap_eps, g, x, g, x_lower, x_upper));
        s = gap_step_assign_bounded_variables(gap_eps, s, x, g, x_lower, x_upper);

        double alpha = backtracking_line_search(
                    make_line_search_function(clamp_function(f,x_lower,x_upper), x, s, f_value),
                    f_value,
                    dot(g,s), // compute gradient for the line search
                    last_alpha,
                    search_strategy.get_wolfe_rho(),
                    search_strategy.get_max_line_search_iterations());

        // Do a trust region style thing for alpha.  The idea is that if we take a
        // small step then we are likely to take another small step.  So we reuse the
        // alpha from the last iteration unless the line search didn't shrink alpha at
        // all, in that case, we start with a bigger alpha next time.
        if (alpha == last_alpha)
            last_alpha = std::min(last_alpha*10,1.0);
        else
            last_alpha = alpha;

        // Take the search step indicated by the above line search
        // Defer pthread cancellation request until updating x is done
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
        x = dlib::clamp(x + alpha*s, x_lower, x_upper);
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

        g = der(x);

        if (!dlib::is_finite(f_value))
            throw dlib::error("The objective function generated non-finite outputs");
        if (!dlib::is_finite(g))
            throw dlib::error("The objective function generated non-finite outputs");
    }

    return f_value;
}

}


#endif // ITOMP_EXEC_FIND_MIN_BOX_CONSTRAINED_THREAD_SAFE_H
