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
#include <graph_core/log.h>

namespace ros_log_color
{
std::ostream& operator<<(std::ostream& os, PRINT_COLOR c)
{
  switch(c)
  {
  case BLACK       : os << "\033[30m"  ; break;
  case RED         : os << "\033[31m"  ; break;
  case GREEN       : os << "\033[32m"  ; break;
  case YELLOW      : os << "\033[33m"  ; break;
  case BLUE        : os << "\033[34m"  ; break;
  case MAGENTA     : os << "\033[35m"  ; break;
  case CYAN        : os << "\033[36m"  ; break;
  case WHITE       : os << "\033[37m"  ; break;
  case BOLDBLACK   : os << "\033[1;30m"; break;
  case BOLDRED     : os << "\033[1;31m"; break;
  case BOLDGREEN   : os << "\033[1;32m"; break;
  case BOLDYELLOW  : os << "\033[1;33m"; break;
  case BOLDBLUE    : os << "\033[1;34m"; break;
  case BOLDMAGENTA : os << "\033[1;35m"; break;
  case BOLDCYAN    : os << "\033[1;36m"; break;
  case BOLDWHITE   : os << "\033[1;37m"; break;
  case ENDCOLOR    : os << "\033[0m";    break;
  default          : os << "\033[37m";
  }
  return os;
}
}
