/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2012-2019, CNRS-UM LIRMM, CNRS-AIST JRL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <gtest/gtest.h>

#include "SolverTestFunctions.h"

#include <tvm/ControlProblem.h>
#include <tvm/LinearizedControlProblem.h>
#include <tvm/Variable.h>
#include <tvm/constraint/abstract/Constraint.h>
#include <tvm/hint/Substitution.h>
#include <tvm/function/abstract/LinearFunction.h>
#include <tvm/function/IdentityFunction.h>
#include <tvm/graph/CallGraph.h>
#include <tvm/scheme/WeightedLeastSquares.h>
#include <tvm/solver/defaultLeastSquareSolver.h>
#include <tvm/task_dynamics/None.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>
#include <tvm/task_dynamics/VelocityDamper.h>

using namespace tvm;
using namespace Eigen;

TEST(SolverTest, basicProblem) {  // NOLINT

}

TEST(SolverTest, substitution) {  // NOLINT
  Space s1(2);
  int dim = 3;
  Space s2(dim);

  VectorXd ddx0;
  VectorXd ddq0;
  {
    VariablePtr x = s1.createVariable("x");
    VariablePtr dx = dot(x);
    x->value(Vector2d(0.5, 0.5));
    dx->value(Vector2d::Zero());

    VariablePtr q = s2.createVariable("q");
    VariablePtr dq = dot(q);
    q->value(Vector3d(0.4, -0.6, 0.9));
    dq->value(Vector3d::Zero());

    auto sf = std::make_shared<SphereFunction>(x, Vector2d(0, 0), 1);
    auto rf = std::make_shared<Simple2dRobotEE>(q, Vector2d(2, 0), Vector3d(1, 1, 1));
    auto idx = std::make_shared<function::IdentityFunction>(x);
    auto df = std::make_shared<Difference>(rf, idx);
    auto idq = std::make_shared<function::IdentityFunction>(dot(q,2));

    VectorXd v(2); v << 0, 0;
    Vector3d b = Vector3d::Constant(1.5);

    double dt = 1e-1;
    LinearizedControlProblem lpb;
    auto t1 = lpb.add(sf == 0., task_dynamics::PD(2), { requirements::PriorityLevel(0) });
    auto t2 = lpb.add(df == v, task_dynamics::PD(2), { requirements::PriorityLevel(0) });
    auto t3 = lpb.add(-b <= q <= b, task_dynamics::VelocityDamper(dt, { 1., 0.01, 0, 1 }), { requirements::PriorityLevel(0) });
    auto t4 = lpb.add(idq == 0., task_dynamics::None(), { requirements::PriorityLevel(1) });

    scheme::WeightedLeastSquares solver(solver::DefaultLSSolverFactory{});
    solver.solve(lpb);
    ddx0 = dot(x, 2)->value();
    ddq0 = dot(q, 2)->value();
  }

  VectorXd ddxs;
  VectorXd ddqs;
  {
    VariablePtr x = s1.createVariable("x");
    VariablePtr dx = dot(x);
    x->value(Vector2d(0.5, 0.5));
    dx->value(Vector2d::Zero());

    VariablePtr q = s2.createVariable("q");
    VariablePtr dq = dot(q);
    q->value(Vector3d(0.4, -0.6, 0.9));
    dq->value(Vector3d::Zero());

    auto sf = std::make_shared<SphereFunction>(x, Vector2d(0, 0), 1);
    auto rf = std::make_shared<Simple2dRobotEE>(q, Vector2d(2, 0), Vector3d(1, 1, 1));
    auto idx = std::make_shared<function::IdentityFunction>(x);
    auto df = std::make_shared<Difference>(rf, idx);
    auto idq = std::make_shared<function::IdentityFunction>(dot(q, 2));

    VectorXd v(2); v << 0, 0;
    Vector3d b = Vector3d::Constant(1.5);

    double dt = 1e-1;
    LinearizedControlProblem lpb;
    auto t1 = lpb.add(sf == 0., task_dynamics::PD(2), { requirements::PriorityLevel(0) });
    auto t2 = lpb.add(df == v, task_dynamics::PD(2), { requirements::PriorityLevel(0) });
    auto t3 = lpb.add(-b <= q <= b, task_dynamics::VelocityDamper(dt, { 1., 0.01, 0, 1 }), { requirements::PriorityLevel(0) });
    auto t4 = lpb.add(idq == 0., task_dynamics::None(), { requirements::PriorityLevel(1) });

    lpb.add(hint::Substitution(lpb.constraint(t2.get()), dot(x, 2)));

    scheme::WeightedLeastSquares solver(solver::DefaultLSSolverOptions{});
    solver.solve(lpb);
    ddxs = dot(x, 2)->value();
    ddqs = dot(q, 2)->value();
  }

  EXPECT_TRUE(ddx0.isApprox(ddxs, 1e-5));
  EXPECT_TRUE(ddq0.isApprox(ddqs, 1e-5));
}
