// Copyright 2016 - 2017 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <string>

#ifndef SRC_UTIL_COLORIZE_H_
#define SRC_UTIL_COLORIZE_H_

namespace colorize {

  std::string ColorRed(const std::string& str);

  std::string ColorBlue(const std::string& str);

  std::string ColorGreen(const std::string& str);

  std::string ColorCyan(const std::string& str);

  std::string ColorWhite(const std::string& str);

  std::string ColorPurple(const std::string& str);

}  // namespace colorize
#endif  // SRC_UTIL_COLORIZE_H_
