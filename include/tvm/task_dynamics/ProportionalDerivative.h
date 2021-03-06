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

#include <tvm/task_dynamics/abstract/TaskDynamics.h>

namespace tvm
{

namespace task_dynamics
{

  /** Compute \ddot{e}* = -kv*dot{f}-kp*f (dynamic order)
  *
  * FIXME have a version with diagonal or sdp gain matrices
  */
  class TVM_DLLAPI ProportionalDerivative : public abstract::TaskDynamics
  {
  public:
    class TVM_DLLAPI Impl: public abstract::TaskDynamicsImpl
    {
    public:
      Impl(FunctionPtr f, constraint::Type t, const Eigen::VectorXd& rhs, double kp, double kv);
      void updateValue() override;

      /** return (kp, kv) */
      std::pair<double, double> gains() const;
      void gains(double kp, double kv);
      /** Critically damped version: kv = 2*sqrt(kp) */
      void gains(double kp);

    private:
      double kp_;
      double kv_;
    };

    /** General constructor*/
    ProportionalDerivative(double kp, double kv);

    /** Critically damped version*/
    ProportionalDerivative(double kp);

  protected:
    std::unique_ptr<abstract::TaskDynamicsImpl> impl_(FunctionPtr f, constraint::Type t, const Eigen::VectorXd& rhs) const override;

  private:
    double kp_;
    double kv_;
  };

  /** Alias for convenience */
  using PD = ProportionalDerivative;
}  // namespace task_dynamics

}  // namespace tvm
