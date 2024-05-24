#include <graph_core/metrics/euclidean_metrics.h>

namespace graph
{
namespace core
{
bool EuclideanMetrics::pluginInit(const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger)
{
  return init(logger);
}
}
}

/**
 * @brief Register class to be loaded with cnr_class_loader
 */
#include <cnr_class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(graph::core::EuclideanMetrics, graph::core::MetricsBase)
