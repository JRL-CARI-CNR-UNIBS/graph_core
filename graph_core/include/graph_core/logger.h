#pragma once
/*
Copyright (c) 2024, Manuel Beschi and Cesare Tonola, JRL-CARI CNR-STIIMA/UNIBS,
manuel.beschi@unibs.it, c.tonola001@unibs.it All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file logger.h
 * @brief Logging interface used by graph_core.
 *
 * graph_core does not depend on any logging framework. Classes receive a
 * graph::core::LoggerPtr and write through the GC_* macros. The application
 * decides where messages go by passing its own Logger implementation (e.g. an
 * adapter to rclcpp::Logger, living in the ROS package that uses graph_core).
 * ConsoleLogger is a ready-to-use implementation writing to std::cout/std::cerr.
 */

#include <cctype>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

namespace graph
{
namespace core
{
enum class LogLevel
{
  Trace = 0,
  Debug,
  Info,
  Warn,
  Error,
  Fatal
};

/**
 * @brief Returns the upper-case name of a level ("TRACE", "DEBUG", ...).
 */
inline const char* toString(LogLevel level)
{
  switch (level)
  {
    case LogLevel::Trace:
      return "TRACE";
    case LogLevel::Debug:
      return "DEBUG";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Warn:
      return "WARN";
    case LogLevel::Error:
      return "ERROR";
    case LogLevel::Fatal:
      return "FATAL";
  }
  return "UNKNOWN";
}

/**
 * @brief Parses a level name (case-insensitive: trace, debug, info, warn,
 * warning, error, fatal).
 * @return the level, or std::nullopt if the name is not valid.
 */
inline std::optional<LogLevel> logLevelFromString(std::string name)
{
  for (char& c : name)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  if (name == "trace")
    return LogLevel::Trace;
  if (name == "debug")
    return LogLevel::Debug;
  if (name == "info")
    return LogLevel::Info;
  if (name == "warn" || name == "warning")
    return LogLevel::Warn;
  if (name == "error")
    return LogLevel::Error;
  if (name == "fatal")
    return LogLevel::Fatal;
  return std::nullopt;
}

/**
 * @brief Abstract logger.
 *
 * Implementations must be thread safe: the same logger can be shared by
 * objects used from different threads.
 */
class Logger
{
public:
  virtual ~Logger() = default;

  /**
   * @brief Tells if messages of the given level are written. The GC_* macros
   * call it before building the message, so disabled levels cost almost nothing.
   */
  virtual bool enabled(LogLevel level) const = 0;

  /**
   * @brief Writes a message.
   * @param level message level.
   * @param func name of the function that produced the message.
   * @param line source line that produced the message.
   * @param msg the message.
   */
  virtual void log(LogLevel level, const char* func, int line, const std::string& msg) = 0;
};
typedef std::shared_ptr<Logger> LoggerPtr;

/**
 * @brief Logger writing to std::cout (TRACE, DEBUG, INFO) and std::cerr (WARN,
 * ERROR, FATAL), with format "[LEVEL] [name] func, line: msg".
 */
class ConsoleLogger : public Logger
{
protected:
  std::string name_;
  LogLevel min_level_;
  mutable std::mutex mtx_;

public:
  /**
   * @param name name printed in each message.
   * @param min_level messages below this level are discarded.
   */
  explicit ConsoleLogger(const std::string& name = "graph_core", LogLevel min_level = LogLevel::Info)
    : name_(name), min_level_(min_level)
  {
  }

  void setLevel(LogLevel min_level)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    min_level_ = min_level;
  }

  LogLevel getLevel() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return min_level_;
  }

  bool enabled(LogLevel level) const override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return level >= min_level_;
  }

  void log(LogLevel level, const char* func, int line, const std::string& msg) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    std::ostream& os = (level >= LogLevel::Warn) ? std::cerr : std::cout;
    os << "[" << toString(level) << "] [" << name_ << "] " << func << ", " << line << ": " << msg << std::endl;
  }
};

/**
 * @brief ANSI color codes, to be streamed into log messages.
 */
namespace color
{
inline std::string RESET()
{
  return "\033[0m";
}
inline std::string BLACK()
{
  return "\033[30m";
}
inline std::string RED()
{
  return "\033[31m";
}
inline std::string GREEN()
{
  return "\033[32m";
}
inline std::string YELLOW()
{
  return "\033[33m";
}
inline std::string BLUE()
{
  return "\033[34m";
}
inline std::string MAGENTA()
{
  return "\033[35m";
}
inline std::string CYAN()
{
  return "\033[36m";
}
inline std::string WHITE()
{
  return "\033[37m";
}
inline std::string BOLDBLACK()
{
  return "\033[1m\033[30m";
}
inline std::string BOLDRED()
{
  return "\033[1m\033[31m";
}
inline std::string BOLDGREEN()
{
  return "\033[1m\033[32m";
}
inline std::string BOLDYELLOW()
{
  return "\033[1m\033[33m";
}
inline std::string BOLDBLUE()
{
  return "\033[1m\033[34m";
}
inline std::string BOLDMAGENTA()
{
  return "\033[1m\033[35m";
}
inline std::string BOLDCYAN()
{
  return "\033[1m\033[36m";
}
inline std::string BOLDWHITE()
{
  return "\033[1m\033[37m";
}
}  // namespace color

}  // namespace core
}  // namespace graph

/**
 * @brief Writes `stream` (anything that can follow `std::ostream <<`) with the
 * given level. Nothing happens if the logger is null or the level is disabled;
 * in that case `stream` is not evaluated.
 */
#define GC_LOG(logger, level, stream)                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    const auto& gc_logger_ = (logger);                                                                                 \
    if (gc_logger_ && gc_logger_->enabled(level))                                                                      \
    {                                                                                                                  \
      std::ostringstream gc_oss_;                                                                                      \
      gc_oss_ << stream;                                                                                               \
      gc_logger_->log(level, __func__, __LINE__, gc_oss_.str());                                                       \
    }                                                                                                                  \
  } while (0)

#define GC_TRACE(logger, stream) GC_LOG(logger, graph::core::LogLevel::Trace, stream)
#define GC_DEBUG(logger, stream) GC_LOG(logger, graph::core::LogLevel::Debug, stream)
#define GC_INFO(logger, stream) GC_LOG(logger, graph::core::LogLevel::Info, stream)
#define GC_WARN(logger, stream) GC_LOG(logger, graph::core::LogLevel::Warn, stream)
#define GC_ERROR(logger, stream) GC_LOG(logger, graph::core::LogLevel::Error, stream)
/// Only logs, like CNR_FATAL: it does not abort the program.
#define GC_FATAL(logger, stream) GC_LOG(logger, graph::core::LogLevel::Fatal, stream)
