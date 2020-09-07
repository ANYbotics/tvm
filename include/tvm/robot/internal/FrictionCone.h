/* Copyright 2017-2018 CNRS-AIST JRL and CNRS-UM LIRMM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <tvm/api.h>

#include <Eigen/Core>

#include <vector>

namespace tvm
{

namespace robot
{

namespace internal
{

/** Linearized friction cone generator.
 *
 * Compute the vectors that linearize the friction cone using the
 * generatrix.
 */
class TVM_DLLAPI FrictionCone
{
public:
  /** Default constructor */
  FrictionCone() {}

  /** Compute the friction cone linearization
   *
   * \param frame Friction cone frame. The friction cone is defined along the frame normal axis
   *
   * \param nrGen Number of vectors generating the cone
   *
   * \param mu Coefficient of friction
   *
   * \param dir Cone direction
   *
   */
  FrictionCone(const Eigen::Matrix3d & frame, unsigned int nrGen, double mu, double direction = 1.0);

  std::vector<Eigen::Vector3d> generators;
};

} // namespace internal

} // namespace robot

} // namespace tvm
