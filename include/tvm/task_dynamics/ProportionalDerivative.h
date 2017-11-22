#pragma once

/* Copyright 2017 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is part of TVM.
 *
 * TVM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * TVM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with TVM.  If not, see <http://www.gnu.org/licenses/>.
 */

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
      /** Critically damped version: kv = sqrt(2*kv) */
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
