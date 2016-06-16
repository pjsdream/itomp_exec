/*
 * Refer to dlib/optimizatino/optimization_stop_strategies.h
 * Class name follows lower_camel_naming_rule as dlib does, but other coding style follows my convention.
 */

#ifndef ITOMP_EXEC_OPTIMIZATION_STOP_STRATEGY_H
#define ITOMP_EXEC_OPTIMIZATION_STOP_STRATEGY_H


#include <dlib/optimization.h>
#include <ros/ros.h>


namespace itomp_exec
{

class time_limit_stop_strategy
{
public:
    time_limit_stop_strategy(double time_limit)
        : verbose_(false)
        , cur_iter_(0)
        , time_limit_(time_limit)
        , start_time_(ros::WallTime::now())
    {
        DLIB_ASSERT (
            time_limit > 0,
            "\t time_limit_stop_strategy(time_limit)"
            << "\n\t time_limit can't be non-positive"
            << "\n\t min_delta: " << time_limit
        );
    }

    time_limit_stop_strategy& be_verbose()
    {
        verbose_ = true;
        return *this;
    }

    template <typename T>
    bool should_continue_search (
        const T& ,
        const double funct_value,
        const T&
    )
    {
        const double elapsed_time = (ros::WallTime::now() - start_time_).toSec();

        if (verbose_)
        {
            using namespace std;
            cout << "iteration: " << cur_iter_ << "   time: " << elapsed_time << "   objective: " << funct_value << endl;
        }

        ++cur_iter_;

        // check if elapsed time exceeds time limit
        if (elapsed_time >= time_limit_)
            return false;

        return true;
    }

private:
    bool verbose_;

    int cur_iter_;
    double time_limit_;
    ros::WallTime start_time_;
};

}

#endif // ITOMP_EXEC_OPTIMIZATION_STOP_STRATEGY_H
