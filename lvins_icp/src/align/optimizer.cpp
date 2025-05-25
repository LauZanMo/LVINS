#include "lvins_icp/align/optimizer.h"

namespace lvins::point_cloud_align {

Optimizer::Optimizer(size_t max_iterations, size_t max_inner_iterations, Float init_lambda, Float lambda_factor)
    : max_iterations_(max_iterations),
      max_inner_iterations_(max_inner_iterations),
      init_lambda_(init_lambda),
      lambda_factor_(lambda_factor) {}

} // namespace lvins::point_cloud_align
