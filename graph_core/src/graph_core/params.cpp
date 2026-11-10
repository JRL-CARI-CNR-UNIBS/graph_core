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

#include <graph_core/params.h>

#include <optional>

namespace graph
{
namespace core
{
namespace
{
std::optional<int64_t> asInt(const YAML::Node& node)
{
  int64_t value;
  if (YAML::convert<int64_t>::decode(node, value))
    return value;
  return std::nullopt;
}

std::optional<double> asDouble(const YAML::Node& node)
{
  double value;
  if (YAML::convert<double>::decode(node, value))
    return value;
  return std::nullopt;
}

std::optional<bool> asBool(const YAML::Node& node)
{
  // Only true/false: yaml-cpp would also turn y/n/yes/no/on/off into bools.
  const std::string& s = node.Scalar();
  if (s == "true" || s == "True" || s == "TRUE")
    return true;
  if (s == "false" || s == "False" || s == "FALSE")
    return false;
  return std::nullopt;
}

ParamValue scalarValue(const YAML::Node& node)
{
  if (auto v = asInt(node))
    return *v;
  if (auto v = asDouble(node))
    return *v;
  if (auto v = asBool(node))
    return *v;
  return node.Scalar();
}

std::optional<ParamValue> sequenceValue(const YAML::Node& node)
{
  bool all_int = true;
  bool all_number = true;
  for (const YAML::Node& item : node)
  {
    if (not item.IsScalar())
      return std::nullopt;
    if (not asInt(item))
      all_int = false;
    if (not asDouble(item))
      all_number = false;
  }

  if (all_int)
  {
    std::vector<int64_t> v;
    for (const YAML::Node& item : node)
      v.push_back(*asInt(item));
    return v;
  }
  if (all_number)
  {
    std::vector<double> v;
    for (const YAML::Node& item : node)
      v.push_back(*asDouble(item));
    return v;
  }
  std::vector<std::string> v;
  for (const YAML::Node& item : node)
    v.push_back(item.Scalar());
  return v;
}

void fill(const YAML::Node& yaml, const std::string& prefix, const std::string& separator, ParamMap& params)
{
  for (const auto& kv : yaml)
  {
    std::string key = prefix + kv.first.as<std::string>();
    const YAML::Node& value = kv.second;

    if (value.IsMap())
      fill(value, key + separator, separator, params);
    else if (value.IsScalar())
      params[key] = scalarValue(value);
    else if (value.IsSequence())
    {
      if (auto v = sequenceValue(value))
        params[key] = *v;
    }
  }
}
}  // namespace

ParamMap paramsFromYAML(const YAML::Node& yaml, const std::string& separator)
{
  if (not yaml.IsMap())
    throw std::invalid_argument("paramsFromYAML: the YAML node is not a map");

  ParamMap params;
  fill(yaml, "", separator, params);
  return params;
}

ParamMap subParams(const ParamMap& params, const std::string& prefix, const std::string& separator)
{
  const std::string full_prefix = prefix + separator;
  ParamMap sub;
  for (auto it = params.lower_bound(full_prefix); it != params.end(); ++it)
  {
    if (it->first.compare(0, full_prefix.size(), full_prefix) != 0)
      break;
    sub[it->first.substr(full_prefix.size())] = it->second;
  }
  return sub;
}

}  // namespace core
}  // namespace graph
