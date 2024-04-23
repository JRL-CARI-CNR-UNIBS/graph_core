#pragma once
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

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


#include <Eigen/Core>
#include <memory>
#include <vector>
#include <cnr_param/cnr_param.h>
#include <cnr_logger/cnr_logger.h>
#include <chrono>
#include <any>
#include <yaml-cpp/yaml.h>

#define Stringize( L )     #L
#define MakeString( M, L ) M(L)
#define $Line MakeString( Stringize, __LINE__ )
#define Reminder __FILE__ "(" $Line ") : Reminder: "


namespace graph
{
namespace core
{
class Connection;
class Node;
class Path;
class PathOptimizerBase;

typedef std::shared_ptr<Connection> ConnectionPtr;
typedef std::shared_ptr<Node> NodePtr;
typedef std::weak_ptr<Connection> ConnectionWeakPtr;
typedef std::weak_ptr<Node> NodeWeakPtr;

static const double TOLERANCE = 1e-06;

/**
 * @brief Retrieves a parameter of type T from a given namespace and parameter name.
 *
 * This function attempts to retrieve a parameter specified by `param_name` within a namespace `param_ns`.
 * The function first checks if the parameter exists under the specified namespace, and if so, attempts to retrieve it.
 * If any step fails, it logs an error message using the provided logger and returns false.
 *
 * @tparam T The data type of the parameter to retrieve. This type should be compatible with the types
 * supported by the `cnr::param::get` function.
 * @param logger A shared pointer to a `cnr_logger::TraceLogger` used for logging errors when the parameter
 * cannot be loaded or is not available.
 * @param ns The namespace under which the parameter is categorized.
 * @param param_name The name of the parameter to retrieve.
 * @param[out] param Reference to a variable where the retrieved parameter value will be stored if successful.
 *
 * @return bool Returns true if the parameter is successfully retrieved and stored in `param`, otherwise false
 * if the parameter does not exist or cannot be retrieved.
 *
 * @note This function uses the `cnr::param` namespace functions `has` and `get` to check for the existence
 * and retrieve the parameter, respectively. Error messages include the full parameter name for clarity.
 */
template<typename T>
inline bool get_param(const cnr_logger::TraceLoggerPtr& logger, const std::string param_ns, const std::string param_name, T& param)
{
  std::string what, full_param_name = param_ns+param_name;
  if(cnr::param::has(full_param_name, what))
  {
    if(not cnr::param::get(full_param_name, param, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n"<<what);
      throw std::invalid_argument("Cannot load " + full_param_name + " parameter.");
    }
  }
  else
  {
    CNR_WARN(logger, full_param_name + " parameter not available.\n"<<what);
    return false;
  }
  return true;
}

template<typename T>
inline bool get_param(const cnr_logger::TraceLoggerPtr& logger, const std::string param_ns, const std::string param_name, T& param, const T& default_value)
{
  std::string what, full_param_name = param_ns+param_name;
  if(cnr::param::has(full_param_name, what))
  {
    if(not cnr::param::get(full_param_name, param, what))
    {
      CNR_ERROR(logger, "Cannot load " << full_param_name + " parameter.\n"<<what);
      throw std::invalid_argument("Cannot load " + full_param_name + " parameter.");
    }
  }
  else
  {
    param = default_value;
    CNR_WARN(logger, full_param_name<<" parameter not available.\n"<<what<<"\nUsing "<<param_name<<": = "<<default_value);
    return false;
  }
  return true;
}

} //end namespace core
} // end namespace graph
