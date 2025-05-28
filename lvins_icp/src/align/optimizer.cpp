#include "lvins_icp/align/optimizer.h"

namespace lvins::point_cloud_align {

Optimizer::Optimizer(size_t max_iterations, size_t max_inner_iterations, double init_lambda, double lambda_factor)
    : max_iterations_(max_iterations),
      max_inner_iterations_(max_inner_iterations),
      init_lambda_(init_lambda),
      lambda_factor_(lambda_factor) {}

size_t Optimizer::maxIterations() const {
    return max_iterations_;
}

size_t Optimizer::maxInnerIterations() const {
    return max_inner_iterations_;
}

double Optimizer::initLambda() const {
    return init_lambda_;
}

double Optimizer::lambdaFactor() const {
    return lambda_factor_;
}

std::string Optimizer::print() const {
    return LVINS_FORMAT("  Optimizer:\n"
                        "    max iterations = {}\n"
                        "    max inner iterations = {}\n"
                        "    initial lambda = {}\n"
                        "    lambda factor = {}",
                        max_iterations_, max_inner_iterations_, init_lambda_, lambda_factor_);
}

} // namespace lvins::point_cloud_align
