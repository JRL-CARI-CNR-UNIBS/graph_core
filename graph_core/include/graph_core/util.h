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

#include <graph_core/logger.h>
#include <graph_core/params.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include <any>
#include <cassert>
#include <chrono>
#include <memory>
#include <type_traits>
#include <vector>

#define Stringize(L) #L
#define MakeString(M, L) M(L)
#define $Line MakeString(Stringize, __LINE__)
#define Reminder __FILE__ "(" $Line ") : Reminder: "

namespace graph
{
namespace core
{
using namespace std::literals::chrono_literals;

using graph_time = std::chrono::system_clock;
using graph_time_point = graph_time::time_point;
using graph_duration = std::chrono::duration<double>;

/**
 * @brief Converts the duration between two time points to seconds.
 *
 * This function calculates the duration between the provided end and start time
 * points, and returns the duration in seconds.
 *
 * @param end The end time point.
 * @param start The start time point.
 * @return The duration in seconds as a double.
 */
inline double toSeconds(const graph_time_point& end, const graph_time_point& start)
{
  return graph_duration(end - start).count();
}

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
 * @brief Overload fucntion to print out a vector.
 *
 * It outputs the elements of the vector in a
 * formatted string enclosed in square brackets and separated by commas.
 * For example, a vector with elements 1, 2, 3 would be output as "[1, 2, 3]".

 * @param os The output stream to which the content is to be written. This is
 typically
 *           std::cout, but can be any stream derived from std::ostream.
 * @param v The vector whose contents are to be output to the stream. The
 elements of the
 *          vector are output in the order they appear in the vector.
 * @return Returns a reference to the output stream `os` after the vector
 content
 *         has been written to it. This allows for chaining of output
 operations.
 */
template <typename T>
inline std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
  os << "[";
  for (size_t i = 0; i < v.size(); ++i)
  {
    if (i != 0)
      os << " ";
    os << v[i];
  }
  os << "]";
  return os;
}

}  // end namespace core
}  // end namespace graph
