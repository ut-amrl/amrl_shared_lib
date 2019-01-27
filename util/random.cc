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
#include "util/random.h"
#include "math/math_util.h"

using std::thread;
using math_util::Sq;

namespace minutebotrandom {

Random::Random()
    : thread_rng_seed(std::hash<std::thread::id>()(std::this_thread::get_id()) +
                      static_cast<int>(GetMonotonicTime() * 10000000.0) +
                      reinterpret_cast<size_t>(this)) {}

Random::Random(const size_t noise) : thread_rng_seed(noise) {}

Random::~Random() {}

int Random::RandomInt(size_t min, size_t max) {
  return (rand_r(&thread_rng_seed) % max) + min;
}

double Random::UniformRandom(double a, double b) {
  return (b - a) * UniformRandom() + a;
}

double Random::UniformRandom() {
  return rand_r(&thread_rng_seed) / static_cast<double>(RAND_MAX);
}

float Random::Gaussian(const float mean, const float stddev) {
  // Uses Box-Muller transform to turn a pair of uniform random
  // numbers into a pair of gaussian random numbers
  float u1 = static_cast<float>(rand_r(&thread_rng_seed)) /
             static_cast<float>(RAND_MAX);
  float u2 = static_cast<float>(rand_r(&thread_rng_seed)) /
             static_cast<float>(RAND_MAX);
  float z1 = Sq(-2.0 * std::log(u1)) * std::sin(2.0 * M_PI * u2);
  return z1 * stddev + mean;
}

unsigned int Random::GetSeedData() const { return thread_rng_seed; }

}  // namespace minutebotrandom
