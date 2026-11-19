#pragma once

#include <graph_core/logger.h>
#include <graph_core/params.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
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
  YAML::Node yaml = YAML::LoadFile(file)[ns];
  if (not yaml.IsMap())
    throw std::invalid_argument("loadParams: '" + ns + "' is not a map in " + file);
  return graph::core::paramsFromYAML(yaml);
}

/**
 * @brief Folder where the tests write their outputs (created if missing):
 * <system temp dir>/graph_core_tests.
 */
inline std::string outputFolder()
{
  const std::filesystem::path folder = std::filesystem::temp_directory_path() / "graph_core_tests";
  std::filesystem::create_directories(folder);
  return folder.string();
}
}  // namespace graph_core_tests
