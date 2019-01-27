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

#include "util/colorize.h"

#include <string>

using std::string;

namespace colorize {

static const char RESET[] = "\033[0;0m";
static const char RED[] = "\033[1;31m";
static const char GREEN[] = "\033[1;32m";
static const char BLUE[] = "\033[1;34m";
static const char PURPLE[] = "\033[1;35m";
static const char CYAN[] = "\033[1;36m";
static const char WHITE[] = "\033[1;37m";

string ColorRed(const string& str) {
  return RED + str + RESET;
}

string ColorGreen(const string& str) {
  return GREEN + str + RESET;
}

string ColorBlue(const string& str) {
  return BLUE + str + RESET;
}

string ColorCyan(const string& str) {
  return CYAN + str + RESET;
}
string ColorPurple(const string& str) {
  return PURPLE + str + RESET;
}
string ColorWhite(const string& str) {
  return WHITE + str + RESET;
}


}  // namespace colorize
