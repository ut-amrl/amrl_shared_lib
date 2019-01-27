// Copyright 2017 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// Random number library.
//
//========================================================================
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
//========================================================================
#include <random>
#include <thread>
#include "util/timer.h"

#ifndef SRC_UTIL_RANDOM_H_
#define SRC_UTIL_RANDOM_H_

// Constructs a thread safe RNG.
namespace minutebotrandom {
class Random {
 public:
  Random();
  explicit Random(const size_t noise);
  ~Random();

  // Generate random numbers between 0 and 1, inclusive.
  double UniformRandom();

  // Generate random numbers between a and b, inclusive.
  double UniformRandom(double a, double b);

  // Generate random numbers between min, inclusive, and max, exclusive.
  int RandomInt(size_t min, size_t max);

  // Return a random value drawn from a Normal distribution.
  float Gaussian(const float mean, const float stddev);

  unsigned int GetSeedData() const;

 private:
  unsigned int thread_rng_seed;
};
}  // namespace minutebotrandom
#endif  // SRC_UTIL_RANDOM_H_
