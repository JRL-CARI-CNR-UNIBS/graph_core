#pragma once
/**
 * @file compat/cnr_logger.h
 * @brief TEMPORARY compatibility layer for code written against cnr_logger.
 *
 * It maps the CNR_* macros, cnr_logger::TraceLoggerPtr and the cnr_logger
 * color functions to graph_core's own logger (graph_core/logger.h), so that
 * packages built on graph_core can be migrated gradually.
 *
 * Do not include it together with the real <cnr_logger/cnr_logger.h>: the
 * names would clash. It will be removed once the downstream packages use the
 * GC_* macros and graph::core::LoggerPtr directly.
 */

#include <graph_core/logger.h>

#ifdef CNR_INFO
#error "graph_core/compat/cnr_logger.h cannot be used together with the real cnr_logger headers"
#endif

#define CNR_TRACE(logger, stream) GC_TRACE(logger, stream)
#define CNR_DEBUG(logger, stream) GC_DEBUG(logger, stream)
#define CNR_INFO(logger, stream) GC_INFO(logger, stream)
#define CNR_WARN(logger, stream) GC_WARN(logger, stream)
#define CNR_ERROR(logger, stream) GC_ERROR(logger, stream)
#define CNR_FATAL(logger, stream) GC_FATAL(logger, stream)
#define CNR_TRACE_THROTTLE(logger, period, stream) GC_TRACE_THROTTLE(logger, period, stream)
#define CNR_DEBUG_THROTTLE(logger, period, stream) GC_DEBUG_THROTTLE(logger, period, stream)
#define CNR_INFO_THROTTLE(logger, period, stream) GC_INFO_THROTTLE(logger, period, stream)
#define CNR_WARN_THROTTLE(logger, period, stream) GC_WARN_THROTTLE(logger, period, stream)
#define CNR_ERROR_THROTTLE(logger, period, stream) GC_ERROR_THROTTLE(logger, period, stream)
#define CNR_FATAL_THROTTLE(logger, period, stream) GC_FATAL_THROTTLE(logger, period, stream)

namespace cnr_logger
{
using TraceLoggerPtr = graph::core::LoggerPtr;

using graph::core::color::BLACK;
using graph::core::color::BLUE;
using graph::core::color::BOLDBLACK;
using graph::core::color::BOLDBLUE;
using graph::core::color::BOLDCYAN;
using graph::core::color::BOLDGREEN;
using graph::core::color::BOLDMAGENTA;
using graph::core::color::BOLDRED;
using graph::core::color::BOLDWHITE;
using graph::core::color::BOLDYELLOW;
using graph::core::color::CYAN;
using graph::core::color::GREEN;
using graph::core::color::MAGENTA;
using graph::core::color::RED;
using graph::core::color::RESET;
using graph::core::color::WHITE;
using graph::core::color::YELLOW;
}  // namespace cnr_logger
