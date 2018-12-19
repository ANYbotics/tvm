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

#include <tvm/task_dynamics/Proportional.h>

#include <tvm/function/abstract/Function.h>

namespace tvm
{

namespace task_dynamics
{

  Proportional::Proportional(double kp)
    : kp_(kp)
  {
  }

  std::unique_ptr<abstract::TaskDynamicsImpl> Proportional::impl_(FunctionPtr f, constraint::Type t, const Eigen::VectorXd& rhs) const
  {
    return std::unique_ptr<abstract::TaskDynamicsImpl>(new Impl(f, t, rhs, kp_));
  }

  Proportional::Impl::Impl(FunctionPtr f, constraint::Type t, const Eigen::VectorXd& rhs, double kp)
    : TaskDynamicsImpl(Order::One, f, t, rhs)
    , kp_(kp)
  {
  }

  void Proportional::Impl::updateValue()
  {
    value_ = -kp_ * (function().value() - rhs());
  }

  void Proportional::Impl::gain(double kp)
  {
    kp_ = kp;
  }

  double Proportional::Impl::gain() const
  {
    return kp_;
  }

}  // namespace task_dynamics

}  // namespace tvm
