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

#include <tvm/Variable.h>
#include <tvm/constraint/internal/LinearizedTaskConstraint.h>
#include <tvm/task_dynamics/Proportional.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>
#include <tvm/utils/graph.h>
#include <tvm/utils/ProtoTask.h>


using namespace Eigen;
using namespace tvm;
using LTC = tvm::constraint::internal::LinearizedTaskConstraint;

TEST(LinearizedTaskConstraintTest, construnction) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  auto dx = dot(x);
  auto ddx = dot(dx);
  auto f = std::make_shared<SphereFunction>(x, Vector3d(1, 0, 0), 2);

  task_dynamics::P td1(2);
  task_dynamics::PD td2(2, 1);

  LTC l1(f == 0., td1);
  ASSERT_EQ(l1.size(), 1);
  ASSERT_EQ(l1.type(), constraint::Type::EQUAL);
  ASSERT_EQ(l1.rhs(), constraint::RHS::AS_GIVEN);
  ASSERT_EQ(l1.variables()[0], dx);
  ASSERT_TRUE(l1.linearIn(*dx));

  LTC l2(f <= 0., td2);
  ASSERT_EQ(l2.size(), 1);
  ASSERT_EQ(l2.type(), constraint::Type::LOWER_THAN);
  ASSERT_EQ(l2.rhs(), constraint::RHS::AS_GIVEN);
  ASSERT_EQ(l2.variables()[0], ddx);
  ASSERT_TRUE(l2.linearIn(*ddx));

  LTC l3(-1. <= f, td2);
  ASSERT_EQ(l3.type(), constraint::Type::GREATER_THAN);

  LTC l4(-1. <= f <= 1., td2);
  ASSERT_EQ(l4.type(), constraint::Type::DOUBLE_SIDED);
}

TEST(LinearizedTaskConstraintTest, valueTest) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  auto dx = dot(x);
  auto ddx = dot(dx);

  x << 1, 2, 3;
  dx << -1, -2, -3;

  auto f = std::make_shared<SphereFunction>(x, Vector3d(1, 0, 0), 2);
  task_dynamics::P td1(2);
  task_dynamics::PD td2(2, 1);

  auto l1 = std::make_shared<LTC>(f == 0., td1);
  auto l2 = std::make_shared<LTC>(f <= 0., td2);

  auto E = LTC::Output::E;
  auto U = LTC::Output::U;
  auto J = LTC::Output::Jacobian;
  auto graph1 = utils::generateUpdateGraph(l1, E, J);
  graph1->execute();
  EXPECT_EQ(l1->e()[0], -2*9);
  EXPECT_TRUE(l1->jacobian(*dx).isApprox(Vector3d(0,4,6).transpose()));

  auto graph2 = utils::generateUpdateGraph(l2, U, J);
  graph2->execute();
  EXPECT_EQ(l2->u()[0], -2*9 - (-26) - 28); /*-kp*f - kv*df/dt - d2f/dxdt dx/dt*/
  EXPECT_TRUE(l2->jacobian(*ddx).isApprox(Vector3d(0, 4, 6).transpose()));
}