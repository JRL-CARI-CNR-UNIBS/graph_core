#pragma once

#include <graph_core/logger.h>
#include <graph_core/params.h>

#include <iostream>
#include <memory>
#include <string>

namespace graph_core_tests
{
/**
 * @brief Creates the logger of a test. The log level is taken from the first
 * command-line argument (trace, debug, info, warn, error, fatal); default: debug.
 * @return the logger, or nullptr if the level is not valid.
 */
inline graph::core::LoggerPtr makeLogger(const std::string& name, int argc, char** argv)
{
  graph::core::LogLevel level = graph::core::LogLevel::Debug;
  if (argc > 1)
  {
    auto parsed = graph::core::logLevelFromString(argv[1]);
    if (not parsed)
    {
      std::cerr << "Invalid log level '" << argv[1] << "'. Use: trace, debug, info, warn, error, fatal" << std::endl;
      return nullptr;
    }
    level = *parsed;
  }
  return std::make_shared<graph::core::ConsoleLogger>(name, level);
}

/**
 * @brief Loads the parameters under `ns` from a YAML file.
 */
inline graph::core::ParamMap loadParams(const std::string& file, const std::string& ns)
{
  return graph::core::paramsFromYAML(YAML::LoadFile(file)[ns]);
}
}  // namespace graph_core_tests
