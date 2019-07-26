#include"pch.h"
#include"levmar_diff_wrapper.h"

int OptimizeUsingLevenbergMarquadt(void(*lm)(double *, double *, int, int, void *), std::vector<double> &q,
    double &error, Ray_Intersect &r_it)
{
    double options[LM_OPTS_SZ] = {
        std::sqrt(LM_INIT_MU),     /* Initial dampening factor */
        std::sqrt(LM_STOP_THRESH), /* epsilon_1 */
        std::sqrt(LM_STOP_THRESH), /* epsilon_2 */
        std::sqrt(LM_STOP_THRESH), /* epsilon_3 */
        -1e-4                       /* Finite differences delta (pos.: forward, neg.: central) */
    };
    double info[LM_INFO_SZ];
    int num_iters = dlevmar_dif(lm, q.data(), nullptr, 2, 2, 1048576, options, info, nullptr, nullptr, &r_it);
    error = info[1];
    return num_iters;
}