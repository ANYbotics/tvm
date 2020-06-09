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
#include <tvm/task_dynamics/Constant.h>
#include <tvm/task_dynamics/None.h>
#include <tvm/task_dynamics/Proportional.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>
#include <tvm/task_dynamics/VelocityDamper.h>
#include <tvm/function/BasicLinearFunction.h>
#include <tvm/function/IdentityFunction.h>
#include <tvm/utils/graph.h>

#include <iostream>

using namespace Eigen;
using namespace tvm;

TEST(TaskDynamicsTest, validConstruction) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  x << Vector3d::Zero();
  auto f = std::make_shared<function::IdentityFunction>(x);
  task_dynamics::Constant td;
  EXPECT_NO_THROW(td.impl(f, constraint::Type::EQUAL, Vector3d(1, 0, 0)));
  EXPECT_ANY_THROW(td.impl(f, constraint::Type::EQUAL, Vector2d(1, 0)));
}

TEST(TaskDynamicsTest, constant) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  x << Vector3d::Zero();
  auto f = std::make_shared<function::IdentityFunction>(x);
  task_dynamics::Constant td;
  auto tdi = td.impl(f,constraint::Type::EQUAL, Vector3d(1,0,0));

  tdi->updateValue();

  EXPECT_EQ(tdi->order(), task_dynamics::Order::Zero);
  EXPECT_TRUE(tdi->value().isApprox(Vector3d(1, 0, 0)));
  EXPECT_TRUE(tdi->checkType<task_dynamics::Constant>());
  EXPECT_FALSE(tdi->checkType<task_dynamics::None>());
}

TEST(TaskDynamicsTest, none) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  x << 1,2,3;
  MatrixXd A = MatrixXd::Random(2, 3);
  Vector2d b(1, 2);
  auto f = std::make_shared<function::BasicLinearFunction>(A, x, b);
  task_dynamics::None td;
  auto tdi = td.impl(f, constraint::Type::EQUAL, Vector2d(1, 0));

  f->updateValue();
  tdi->updateValue();

  EXPECT_EQ(tdi->order(), task_dynamics::Order::Zero);
  EXPECT_TRUE(tdi->value().isApprox(Vector2d(0,-2)));
  EXPECT_TRUE(tdi->checkType<task_dynamics::None>());
  EXPECT_FALSE(tdi->checkType<task_dynamics::Constant>());
}

TEST(TaskDynamicsTest, proportional) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  x << 1, 2, 3;
  auto f = std::make_shared<SphereFunction>(x, Vector3d(1, 0, -3), 2);

  double kp = 2;
  task_dynamics::P td(kp);
  VectorXd rhs(1); rhs[0] = -1;
  auto tdi = td.impl(f, constraint::Type::EQUAL, rhs);

  f->updateValue();
  tdi->updateValue();

  EXPECT_EQ(tdi->order(), task_dynamics::Order::One);
  EXPECT_EQ(tdi->value()[0], -kp*(36 - rhs[0]));  // -kp*||(1,2,3) - (1,0,-3)||^2 - 2^2 - rhs
  EXPECT_TRUE(tdi->checkType<task_dynamics::P>());
  EXPECT_FALSE(tdi->checkType<task_dynamics::Constant>());
}

TEST(TaskDynamicsTest, proportionalDerivative) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  VariablePtr dx = dot(x);
  x << 1, 2, 3;
  dx << 1, 1, 1;
  auto f = std::make_shared<SphereFunction>(x, Vector3d(1, 0, -3), 2);

  double kp = 2;
  double kv = 3;
  task_dynamics::PD td(kp,kv);
  VectorXd rhs(1); rhs[0] = -1;
  auto tdi = td.impl(f, constraint::Type::EQUAL, rhs);

  f->updateValue();
  f->updateVelocityAndNormalAcc();
  tdi->updateValue();

  EXPECT_EQ(tdi->order(), task_dynamics::Order::Two);
  EXPECT_EQ(tdi->value()[0], -kp*(36 - rhs[0])-kv*16);
  EXPECT_TRUE(tdi->checkType<task_dynamics::PD>());
  EXPECT_FALSE(tdi->checkType<task_dynamics::Constant>());
}

TEST(TaskDynamicsTest, velocityDamper) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  VariablePtr dx = dot(x);
  auto f = std::make_shared<function::IdentityFunction>(x);

  double di = 3;
  double ds = 1;
  double xsi = 2;
  double dt = 0.1;

  //test validity
  EXPECT_ANY_THROW(task_dynamics::VelocityDamper({ 0.5, ds, xsi }));
  EXPECT_ANY_THROW(task_dynamics::VelocityDamper(dt, { 0.5, ds, xsi }, 1000));
  EXPECT_ANY_THROW(task_dynamics::VelocityDamper({ di, ds, -1 }));
  EXPECT_ANY_THROW(task_dynamics::VelocityDamper(dt, { di, ds, -1 }, 1000));
  EXPECT_ANY_THROW(task_dynamics::VelocityDamper(0, { di, ds, xsi }, 1000));
  EXPECT_ANY_THROW(task_dynamics::VelocityDamper(-0.1, { di, ds, xsi }, 1000));

  //test kinematics
  task_dynamics::VelocityDamper td1({ di, ds, xsi });
  {
    x << 1, 2, 4;
    auto tdl = td1.impl(f, constraint::Type::GREATER_THAN, Vector3d::Zero());

    f->updateValue();
    tdl->updateValue();

    EXPECT_EQ(tdl->order(), task_dynamics::Order::One);
    EXPECT_EQ(tdl->value()[0], 0);
    EXPECT_EQ(tdl->value()[1], -1);
    EXPECT_EQ(tdl->value()[2], -constant::big_number);
    EXPECT_TRUE(tdl->checkType<task_dynamics::VelocityDamper>());
    EXPECT_FALSE(tdl->checkType<task_dynamics::Constant>());


    x << -1, -2, -4;
    auto tdu = td1.impl(f, constraint::Type::LOWER_THAN, Vector3d::Zero());
    f->updateValue();
    tdu->updateValue();
    EXPECT_EQ(tdu->value()[0], 0);
    EXPECT_EQ(tdu->value()[1], 1);
    EXPECT_EQ(tdu->value()[2], constant::big_number);
  }


  //test dynamics
  double big = 1000;
  task_dynamics::VelocityDamper td2(dt, { di, ds, xsi }, big);
  {
    x << 1, 2, 4;
    dx << 1, 1, 1;
    auto tdl = td2.impl(f, constraint::Type::GREATER_THAN, Vector3d::Zero());
    
    f->updateValue();
    f->updateVelocity();
    tdl->updateValue();

    EXPECT_EQ(tdl->order(), task_dynamics::Order::Two);
    EXPECT_TRUE(tdl->value().isApprox(Vector3d(-10, -20, -1000)));
    EXPECT_TRUE(tdl->checkType<task_dynamics::VelocityDamper>());
    EXPECT_FALSE(tdl->checkType<task_dynamics::Constant>());

    x << -1, -2, -4;
    dx << -1, -1, -1;
    auto tdu = td2.impl(f, constraint::Type::LOWER_THAN, Vector3d::Zero());

    f->updateValue();
    f->updateVelocity();
    tdu->updateValue();
    EXPECT_TRUE(tdu->value().isApprox(Vector3d(10, 20, 1000)));
  }

}

TEST(TaskDynamicsTest, automaticXsi) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  VariablePtr dx = dot(x);
  auto f = std::make_shared<function::IdentityFunction>(x);

  double di = 3;
  double ds = 1;
  double xsiOff = 1;
  double dt = 0.1;

  //test kinematics
  double big = 100;
  task_dynamics::VelocityDamper td1({ di, ds, 0, xsiOff }, big);
  {
    x << 5, 4, 2;
    dx << -0.5, -0.5, -0.5;
    TaskDynamicsPtr tdl = td1.impl(f, constraint::Type::GREATER_THAN, Vector3d::Zero());

    auto Value = task_dynamics::abstract::TaskDynamicsImpl::Output::Value;
    auto gl = utils::generateUpdateGraph(tdl, Value);

    gl->execute();
    EXPECT_TRUE(tdl->value().isApprox(Vector3d(-big, -big, -1)));

    x << 4.5, 3.5, 1.5;
    gl->execute();
    EXPECT_TRUE(tdl->value().isApprox(Vector3d(-big, -big, -0.5)));

    x << 4, 3, 1;
    gl->execute();
    EXPECT_TRUE(tdl->value().isApprox(Vector3d(-big, -1.5, 0)));

    //we check that two consecutive updates with the same variable values give the same results.
    gl->execute();
    EXPECT_TRUE(tdl->value().isApprox(Vector3d(-big, -1.5, 0)));

    dx << -0.5, -0.5, 0;
    x << 3.5, 2.5, 1;
    gl->execute();
    EXPECT_TRUE(tdl->value().isApprox(Vector3d(-big, -1.125, 0)));

    x << 3, 2, 1;
    gl->execute();
    EXPECT_TRUE(tdl->value().isApprox(Vector3d(-1.5, -0.75, 0)));

    x << -5, -4, -2;
    dx << 0.5, 0.5, 0.5;
    TaskDynamicsPtr tdu = td1.impl(f, constraint::Type::LOWER_THAN, Vector3d::Zero());

    auto gu = utils::generateUpdateGraph(tdu, Value);

    gu->execute();
    EXPECT_TRUE(tdu->value().isApprox(Vector3d(big, big, 1)));

    x << -4.5, -3.5, -1.5;
    gu->execute();
    EXPECT_TRUE(tdu->value().isApprox(Vector3d(big, big, 0.5)));

    x << -4, -3, -1;
    gu->execute();
    EXPECT_TRUE(tdu->value().isApprox(Vector3d(big, 1.5, 0)));

    dx << 0.5, 0.5, 0;
    x << -3.5, -2.5, -1;
    gu->execute();
    EXPECT_TRUE(tdu->value().isApprox(Vector3d(big, 1.125, 0)));

    x << -3, -2, -1;
    gu->execute();
    EXPECT_TRUE(tdu->value().isApprox(Vector3d(1.5, 0.75, 0)));
  }
}