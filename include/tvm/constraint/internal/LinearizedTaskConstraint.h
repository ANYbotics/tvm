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

#include <tvm/Task.h>
#include <tvm/constraint/abstract/LinearConstraint.h>
#include <tvm/utils/ProtoTask.h>

namespace tvm
{

namespace constraint
{

namespace internal
{

  /** Given a task \f$(e, op, rhs, e^*)\f$, this class derives the constraint
    * \f$d^k e/dt^k\ op\  e^*(e,de/dt,...de^{k-1}/dt^{k-1}, rhs [,g])\f$, where e is an
    * error function, op is ==, >= or <= and \f$e^*\f$ is a desired error dynamics. 
    * k is specified by \f$e^*\f$ and (optional) g is any other quantities.
    *
    * EQUAL (E) \ GREATER_THAN (L) \ LOWER_THAN (U) cases. Dotted dependencies
    * correspond by default to the second order dynamics case (k=2), unless
    * specified otherwise by the task dynamics used.
    * \dot
    * digraph "update graph" {
    *   rankdir="LR";
    *   subgraph cluster1 {
    *     label="f"
    *     {
    *       rank=same; node [shape=diamond];
    *       fValue [label="Value"]; 
    *       fJacobian [label="Jacobian"]; 
    *       fVelocity [label="Velocity",style=dotted];
    *       fNormalAcceleration [label="NormalAcceleration",style=dotted];
    *     }
    *   }
    *   subgraph cluster2 {
    *     label="td"
    *     {
    *       tdValue [label="Value", shape=Mdiamond];
    *       tdUpdate [label="UpdateValue"];
    *     }
    *   }
    *   {
    *     rank=same;
    *     uValue [label=Value];
    *     updateRHS;
    *   }
    *   {
    *     rank = same; node [shape=hexagon];
    *     Value; Jacobian; 
    *     E [label="E \\ L \\ U"];
    *   }
    *   {
    *     rank = same; node [style=invis, label=""];
    *     outValue; outJacobian; outE;
    *   }
    *   x_i [shape=box]
    *   fValue -> tdUpdate
    *   fVelocity -> tdUpdate [style=dotted]
    *   tdUpdate -> tdValue
    *   tdValue -> updateRHS
    *   updateRHS -> E
    *   fJacobian -> Jacobian
    *   fJacobian -> uValue
    *   fNormalAcceleration -> updateRHS [style=dotted]
    *   Value -> outValue [label="value()"];
    *   Jacobian -> outJacobian [label="jacobian(x_i)"];
    *   E -> outE [label="e() \\ l() \\ u()"];
    *   x_i -> uValue [label="value()"];
    *   uValue -> Value;
    * }
    * \enddot
    *
    * DOUBLE_SIDED case. Dotted dependencies correspond by default to the second
    * order dynamics case (k=2), unless specified otherwise by the task dynamics
    * used.
    * \dot
    * digraph "update graph" {
    *   rankdir="LR";
    *   subgraph cluster1 {
    *     label="f"
    *     {
    *       rank=same; node [shape=diamond];
    *       fValue [label="Value"];
    *       fJacobian [label="Jacobian"];
    *       fVelocity [label="Velocity",style=dotted];
    *       fNormalAcceleration [label="NormalAcceleration",style=dotted];
    *     }
    *   }
    *   subgraph cluster2 {
    *     label="td"
    *     {
    *       td1Value [label="Value", shape=Mdiamond];
    *       td1Update [label="UpdateValue"];
    *     }
    *   }
    *   subgraph cluster3 {
    *     label="td2"
    *     {
    *       td2Value [label="Value", shape=Mdiamond];
    *       td2Update [label="UpdateValue"];
    *     }
    *   }
    *   {
    *     rank=same;
    *     uValue [label=Value];
    *     updateRHS;
    *     updateRHS2;
    *   }
    *   {
    *     rank = same; node [shape=hexagon];
    *     Value; Jacobian; L; U
    *   }
    *   {
    *     rank = same; node [style=invis, label=""];
    *     outValue; outJacobian; outL; outU
    *   }
    *   x_i [shape=box]
    *   fValue -> td1Update
    *   fVelocity -> td1Update [style=dotted]
    *   td1Update -> td1Value
    *   td1Value -> updateRHS
    *   fValue -> td2Update
    *   fVelocity -> td2Update [style=dotted]
    *   td2Update -> td2Value
    *   td2Value -> updateRHS2
    *   updateRHS -> L
    *   updateRHS2 -> U
    *   fJacobian -> Jacobian
    *   fJacobian -> uValue
    *   fNormalAcceleration -> updateRHS [style=dotted]
    *   fNormalAcceleration -> updateRHS2 [style=dotted]
    *   Value -> outValue [label="value()"];
    *   Jacobian -> outJacobian [label="jacobian(x_i)"];
    *   L -> outL [label="l()"];
    *   U -> outU [label="u()"];
    *   x_i -> uValue [label="value()"];
    *   uValue -> Value;
    * }
    * \enddot
    *
    * FIXME Consider the case where the TaskDynamics has its own variables?
    */
  class TVM_DLLAPI LinearizedTaskConstraint : public abstract::LinearConstraint
  {
  public:
    SET_UPDATES(LinearizedTaskConstraint, UpdateRHS, UpdateRHS2)

    /** Constructor from a task*/
    LinearizedTaskConstraint(const Task& task);

    /** Constructor from a ProtoTask and a TaskDynamics*/
    template<constraint::Type T>
    LinearizedTaskConstraint(const utils::ProtoTask<T>& pt, const task_dynamics::abstract::TaskDynamics& td);

    void updateLKin();
    void updateLDyn();
    void updateUKin();
    void updateUDyn();
    void updateEKin();
    void updateEDyn();
    void updateU2Kin();
    void updateU2Dyn();

    const tvm::internal::MatrixWithProperties& jacobian(const Variable& x) const override;

  private:
    FunctionPtr f_;
    TaskDynamicsPtr td_;
    TaskDynamicsPtr td2_; // for double sided constraints only;
  };


  template<constraint::Type T>
  LinearizedTaskConstraint::LinearizedTaskConstraint(const utils::ProtoTask<T>& pt, const task_dynamics::abstract::TaskDynamics& td)
    : LinearizedTaskConstraint(Task(pt, td))
  {
  }

}  // namespace internal

}  // namespace constraint

}  // namespace tvm
