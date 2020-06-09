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
#include <tvm/Task.h>
#include <tvm/Variable.h>
#include <tvm/function/BasicLinearFunction.h>
#include <tvm/function/IdentityFunction.h>
#include <tvm/task_dynamics/Proportional.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>
#include <tvm/utils/checkFunction.h>
#include <tvm/utils/graph.h>
#include <tvm/utils/ProtoTask.h>
#include <tvm/utils/UpdatelessFunction.h>

#include <iostream>

using namespace Eigen;
using namespace tvm;
using namespace tvm::utils;


template<typename T>
void checkSimpleBoundOnFunction(const T& bound)
{
  Space s1(2);
  VariablePtr x = s1.createVariable("x");
  auto f = std::make_shared<function::IdentityFunction>(x);
  task_dynamics::P td(2);

  {
    Task t(f == bound, td);
    EXPECT_EQ(t.type(), constraint::Type::EQUAL);
    EXPECT_EQ(t.function(), f);
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(bound == f, td);
    EXPECT_EQ(t.type(), constraint::Type::EQUAL);
    EXPECT_EQ(t.function(), f);
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(f >= bound, td);
    EXPECT_EQ(t.type(), constraint::Type::GREATER_THAN);
    EXPECT_EQ(t.function(), f);
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(bound <= f, td);
    EXPECT_EQ(t.type(), constraint::Type::GREATER_THAN);
    EXPECT_EQ(t.function(), f);
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(f <= bound, td);
    EXPECT_EQ(t.type(), constraint::Type::LOWER_THAN);
    EXPECT_EQ(t.function(), f);
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(bound >= f, td);
    EXPECT_EQ(t.type(), constraint::Type::LOWER_THAN);
    EXPECT_EQ(t.function(), f);
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }
}


template<typename L, typename U>
void checkDoubleBoundsOnFunction(const L& l, const U& u)
{
  Space s1(2);
  VariablePtr x = s1.createVariable("x");
  auto f = std::make_shared<function::IdentityFunction>(x);
  task_dynamics::P td(2);

  {
    Task t(l <= f <= u, td);
    EXPECT_EQ(t.type(), constraint::Type::DOUBLE_SIDED);
    EXPECT_EQ(t.function(), f);
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_TRUE(t.secondBoundTaskDynamics());
  }

  {
    Task t(l >= f >= u, td);
    EXPECT_EQ(t.type(), constraint::Type::DOUBLE_SIDED);
    EXPECT_EQ(t.function(), f);
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_TRUE(t.secondBoundTaskDynamics());
  }
}

template<typename T>
void checkSimpleBoundOnVariable(const T& bound)
{
  Space s1(2);
  VariablePtr x = s1.createVariable("x");
  task_dynamics::P td(2);

  {
    Task t(x == bound, td);
    EXPECT_EQ(t.type(), constraint::Type::EQUAL);
    EXPECT_TRUE(t.function()->jacobian(*x).properties().isIdentity());
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(bound == x, td);
    EXPECT_EQ(t.type(), constraint::Type::EQUAL);
    EXPECT_TRUE(t.function()->jacobian(*x).properties().isIdentity());
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(x >= bound, td);
    EXPECT_EQ(t.type(), constraint::Type::GREATER_THAN);
    EXPECT_TRUE(t.function()->jacobian(*x).properties().isIdentity());
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(bound <= x, td);
    EXPECT_EQ(t.type(), constraint::Type::GREATER_THAN);
    EXPECT_TRUE(t.function()->jacobian(*x).properties().isIdentity());
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(x <= bound, td);
    EXPECT_EQ(t.type(), constraint::Type::LOWER_THAN);
    EXPECT_TRUE(t.function()->jacobian(*x).properties().isIdentity());
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }

  {
    Task t(bound >= x, td);
    EXPECT_EQ(t.type(), constraint::Type::LOWER_THAN);
    EXPECT_TRUE(t.function()->jacobian(*x).properties().isIdentity());
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_FALSE(t.secondBoundTaskDynamics());
  }
}


template<typename L, typename U>
void checkDoubleBoundsOnVariable(const L& l, const U& u)
{
  Space s1(2);
  VariablePtr x = s1.createVariable("x");
  task_dynamics::P td(2);

  {
    Task t(l <= x <= u, td);
    EXPECT_EQ(t.type(), constraint::Type::DOUBLE_SIDED);
    EXPECT_TRUE(t.function()->jacobian(*x).properties().isIdentity());
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_TRUE(t.secondBoundTaskDynamics());
  }

  {
    Task t(l >= x >= u, td);
    EXPECT_EQ(t.type(), constraint::Type::DOUBLE_SIDED);
    EXPECT_TRUE(t.function()->jacobian(*x).properties().isIdentity());
    EXPECT_NO_THROW(t.taskDynamics<task_dynamics::P>());
    EXPECT_ANY_THROW(t.taskDynamics<task_dynamics::PD>());
    EXPECT_TRUE(t.secondBoundTaskDynamics());
  }
}

TEST(UtilsTest, protoTask) {  // NOLINT
  // scalar-defined bounds
  checkSimpleBoundOnFunction(0.);
  checkDoubleBoundsOnFunction(-1., 3.);
  checkSimpleBoundOnVariable(0.);
  checkDoubleBoundsOnVariable(-1., 3.);

  // vector-defined bounds
  VectorXd l = -Vector2d::Ones();
  VectorXd u = Vector2d::Ones();
  checkSimpleBoundOnFunction(l);
  checkDoubleBoundsOnFunction(l, u);
  checkSimpleBoundOnVariable(l);
  checkDoubleBoundsOnVariable(l, u);

  // mix
  checkDoubleBoundsOnFunction(-1., u);
  checkDoubleBoundsOnFunction(l, 3.);
  checkDoubleBoundsOnVariable(-1., u);
  checkDoubleBoundsOnVariable(l, 3.);

  // expression
  Matrix2d A = Matrix2d::Identity();
  checkSimpleBoundOnFunction(-l);
  checkSimpleBoundOnFunction(A*l - u);
  checkSimpleBoundOnVariable(-l);
  checkSimpleBoundOnVariable(A*l - u);
}

TEST(UtilsTest, updatelessFunction) {  // NOLINT
  //value
  VariablePtr x = Space(2).createVariable("x");
  VariablePtr y = Space(4).createVariable("y");
  VariablePtr z = Space(3).createVariable("z");
  MatrixXd Ax = MatrixXd::Random(3, 2);
  MatrixXd Ay = MatrixXd::Random(3, 4);
  MatrixXd Az = MatrixXd::Random(3, 3);
  auto f = std::shared_ptr<function::BasicLinearFunction>( 
    new function::BasicLinearFunction( { Ax, Ay, Az }, {x, y, z} ));

  UpdatelessFunction uf(f);

  VectorXd xr = VectorXd::Random(2);
  VectorXd yr = VectorXd::Random(4);
  VectorXd zr = VectorXd::Random(3);

  VectorXd v = uf.value(xr, yr, zr);
  EXPECT_TRUE(v.isApprox(Ax * xr + Ay * yr + Az * zr));
  EXPECT_TRUE(x->value().isApprox(xr));
  EXPECT_TRUE(y->value().isApprox(yr));
  EXPECT_TRUE(z->value().isApprox(zr));

  VectorXd xm(2); xm << 1, 2;
  VectorXd ym(4); ym << 3, 4, 5, 6;
  VectorXd zm(3); zm << 7, 8, 9;

  using l = std::initializer_list<double>;
  v = uf.value(l{ 1,2 }, l{ 3,4,5,6 }, l{ 7., 8., 9. });
  EXPECT_TRUE(v.isApprox(Ax * xm + Ay * ym + Az * zm));

  v = uf.value(l{ 1,2 }, yr, l{ 7., 8., 9. });
  EXPECT_TRUE(v.isApprox(Ax * xm + Ay * yr + Az * zm));

  v = uf.value(*x, xr);
  EXPECT_TRUE(v.isApprox(Ax * xr + Ay * yr + Az * zm));

  v = uf.value(*z, l{ 1,2,3 }, *y, l{ 3,4,5,6 }, *z, l{ 7,8,9 }, *x, xr);
  EXPECT_TRUE(v.isApprox(Ax * xr + Ay * ym + Az * zm));

  VectorXd concatr(9);
  concatr << xr, yr, zr;
  v = uf.value(concatr);
  EXPECT_TRUE(v.isApprox(Ax * xr + Ay * yr + Az * zr));

  v = uf.value(l{ 1,2,3,4,5,6,7,8,9 });
  EXPECT_TRUE(v.isApprox(Ax * xm + Ay * ym + Az * zm));

  //errors
  // not enough args
  EXPECT_ANY_THROW(uf.value(xr, yr));
  EXPECT_ANY_THROW(uf.value(l{ 1,2 }, l{3,4,5,6}));
  //uf.value(*x); //does not compile (and that's normal)
  // too many args
  EXPECT_ANY_THROW(uf.value(xr, yr, zr, xm, ym, zm));
  EXPECT_ANY_THROW(uf.value(l{ 1,2 }, l{ 3,4,5,6 }, zr, l{ 7,8,9 }));
  //uf.value(*x, xr, yr); //does not compile (and that's normal)
  // wrong size
  EXPECT_ANY_THROW(uf.value(xr, zr, yr));
  EXPECT_ANY_THROW(uf.value(l{ 1,2,3 }, l{ 3,4,5,6 }, l{ 7., 8., 9. }));
  EXPECT_ANY_THROW(uf.value(*x, zr));


  //jacobian
  auto s1 = std::make_shared<SphereFunction>(x, Vector2d(0, 0), 1);
  auto s2 = std::make_shared<SphereFunction>(z, Vector3d(0, 0, 1), 1);
  auto df = std::make_shared<Difference>(s1, s2);
  UpdatelessFunction udf(df);

  MatrixXd J = udf.jacobian(*x, xr, zr);
  EXPECT_TRUE(J.isApprox(2 * xr.transpose()));
  J = udf.jacobian(*z, xr, zr);
  EXPECT_TRUE(J.isApprox(-2 * (zr - Vector3d(0, 0, 1)).transpose()));
  J = udf.jacobian(*x, l{ 1,2 }, l{ 7,8,9 });
  EXPECT_TRUE(J.isApprox(2 * xm.transpose()));
  J = udf.jacobian(*x, *x, l{ 1,2 });
  EXPECT_TRUE(J.isApprox(2 * xm.transpose()));
  J = udf.jacobian(*x, l{ 1,2,7,8,9 });
  EXPECT_TRUE(J.isApprox(2 * xm.transpose()));

  //velocity
  VectorXd dxr = VectorXd::Random(2);
  VectorXd dyr = VectorXd::Random(4);
  VectorXd dzr = VectorXd::Random(3);
  
  VectorXd dv = uf.velocity(xr, dxr, yr, dyr, zr, dzr);
  EXPECT_TRUE(dv.isApprox(Ax * dxr + Ay * dyr + Az * dzr));
  EXPECT_TRUE(dot(x)->value().isApprox(dxr));
  EXPECT_TRUE(dot(y)->value().isApprox(dyr));
  EXPECT_TRUE(dot(z)->value().isApprox(dzr));

  VectorXd dxm(2); dxm << -1, -2;
  VectorXd dym(4); dym << -3, -4, -5, -6;
  VectorXd dzm(3); dzm << -7, -8, -9;

  dv = uf.velocity(l{ 1,2 }, l{ -1,-2 }, l{ 3,4,5,6 }, l{ -3,-4,-5,-6 }, l{ 7,8,9 }, l{ -7, -8, -9 });
  EXPECT_TRUE(dv.isApprox(Ax * dxm + Ay * dym + Az * dzm));

  dv = uf.velocity(xr, l{ -1,-2 }, l{ 3,4,5,6 }, dyr, zr, l{ -7,-8,-9 });
  EXPECT_TRUE(dv.isApprox(Ax * dxm + Ay * dyr + Az * dzm));

  dv = uf.velocity(*x, xr, dxr);
  EXPECT_TRUE(dv.isApprox(Ax * dxr + Ay * dyr + Az * dzm));

  dv = uf.velocity(*z, l{1,2,3}, l{ -1,-2,-3 }, *y, yr, l{ -3,-4,-5,-6 }, *z, zr, l{ -7,-8,-9 }, *x, xr, dxr);
  EXPECT_TRUE(dv.isApprox(Ax * dxr + Ay * dym + Az * dzm));

  VectorXd concatdr(9);
  concatdr << dxr, dyr, dzr;
  dv = uf.velocity(concatr, concatdr);
  EXPECT_TRUE(dv.isApprox(Ax * dxr + Ay * dyr + Az * dzr));

  dv = uf.velocity(l{ 1,2,3,4,5,6,7,8,9 }, concatdr);
  EXPECT_TRUE(dv.isApprox(Ax * dxr + Ay * dyr + Az * dzr));

  dv = uf.velocity(concatr, l{ -1,-2,-3,-4,-5,-6,-7,-8,-9 });
  EXPECT_TRUE(dv.isApprox(Ax * dxm + Ay * dym + Az * dzm));

  dv = uf.velocity(l{ 1,2,3,4,5,6,7,8,9 }, l{ -1,-2,-3,-4,-5,-6,-7,-8,-9 });
  EXPECT_TRUE(dv.isApprox(Ax * dxm + Ay * dym + Az * dzm));

  //errors
  // not enough args
  EXPECT_ANY_THROW(uf.velocity(xr, dxr, yr, dyr));
  //uf.velocity(xr, dxr, yr, dyr, zr); //does not compile (and that's normal)
  EXPECT_ANY_THROW(uf.velocity(l{ 1,2 }, l{-1,-2}, l{ 3,4,5,6 }, l{ -3,-4,-5,-6 }));
  //uf.value(*x); //does not compile (and that's normal)
  // too many args
  EXPECT_ANY_THROW(uf.velocity(xr, dxr, yr, dyr, zr, dzr, xm, dxm, ym, dym, zm, dzm));
  EXPECT_ANY_THROW(uf.velocity(l{ 1,2 }, dxr, l{ 3,4,5,6 }, l{ -3,-4,-5,-6 }, zr, l{ -7,-8,-9 }, l{ 1,2 }, l{ -1,-2 }, l{ 3,4,5,6 }, l{ -3,-4,-5,-6 }));
  //uf.value(*x, xr, yr); //does not compile (and that's normal)
  // wrong size
  EXPECT_ANY_THROW(uf.velocity(xr, dxr, yr, dxr, zr, dzr));
  EXPECT_ANY_THROW(uf.velocity(l{ 1,2,3 }, dxr, l{ 3,4,5,6 }, dyr, l{ 7., 8., 9. }, dzr));
  EXPECT_ANY_THROW(uf.velocity(*x, xr, dzr));

  // normal acceleration
  VectorXd na = udf.normalAcceleration(xr, dxr, zr, dzr);
  EXPECT_TRUE(na.isApprox(2*(dxr.transpose()*dxr - dzr.transpose()*dzr)));
  na = udf.normalAcceleration(l{ 1,2 }, l{ -1,-2 }, l{ 7,8,9 }, l{ -7,-8,-9 });
  EXPECT_TRUE(na.isApprox(2 * (dxm.transpose()*dxm - dzm.transpose()*dzm)));
  na = udf.normalAcceleration(l{ 1,2 }, dxr, zr, l{ -7,-8,-9 });
  EXPECT_TRUE(na.isApprox(2 * (dxr.transpose()*dxr - dzm.transpose()*dzm)));
  na = udf.normalAcceleration(*z, l{ 7,8,9 }, dzr, *x, l{ 1,2 }, l{ -1,-2 });
  EXPECT_TRUE(na.isApprox(2 * (dxm.transpose()*dxm - dzr.transpose()*dzr)));
  na = udf.normalAcceleration(l{ 1,2,7,8,9 }, l{ -1,-2,-7,-8,-9 });
  EXPECT_TRUE(na.isApprox(2 * (dxm .transpose()*dxm - dzm.transpose()*dzm)));

  EXPECT_ANY_THROW(udf.normalAcceleration(*z, l{ 7,8.9 }, dzr, *x, l{ 1,2 }, l{ -1,-2 }));

  // JDot
  MatrixXd Jd = uf.JDot(*x, xr, dxr, yr, dyr, zr, dzr);
  EXPECT_TRUE(Jd.isZero());
  Jd = uf.JDot(*x, concatr, concatdr);
  EXPECT_TRUE(Jd.isZero());
  EXPECT_ANY_THROW(udf.JDot(*x, xr, dzr));
}

TEST(UtilsTest, checks) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");

  auto f = std::make_shared<SphereFunction>(x, Vector3d(1, 0, 0), 3);
  auto brokenf = std::make_shared<BrokenSphereFunction>(x, Vector3d(1, 0, 0), 3);
  EXPECT_TRUE(checkJacobian(f));
  brokenf->breakJacobian(true);
  EXPECT_FALSE(checkJacobian(brokenf, {1e-7, 1e-6,false}));
  brokenf->breakJacobian(false);

  EXPECT_TRUE(checkVelocity(f));
  brokenf->breakVelocity(true);
  EXPECT_FALSE(checkVelocity(brokenf, { 1e-7, 1e-6,false }));
  brokenf->breakVelocity(false);

  EXPECT_TRUE(checkNormalAcceleration(f));
  brokenf->breakNormalAcceleration(true);
  EXPECT_FALSE(checkNormalAcceleration(brokenf, { 1e-7, 1e-6,false }));
  brokenf->breakNormalAcceleration(false);
}

TEST(UtilsTest, graphGeneration) {  // NOLINT
  VariablePtr x = Space(3).createVariable("x");
  auto dx = dot(x);
  x << Vector3d::Random();
  dx << Vector3d::Random();

  auto f1 = std::make_shared<SphereFunction>(x, Vector3d(1, 0, 0), 3);
  auto f2 = std::make_shared<SphereFunction>(x, Vector3d(0, 1, 0), 3);
  auto f3 = std::make_shared<SphereFunction>(x, Vector3d(0, 1, 0), 3);
  auto f4 = std::make_shared<SphereFunction>(x, Vector3d(1, 1, 1), 3);
  auto f12 = std::make_shared<Difference>(f1, f2);
  auto f34 = std::make_shared<Difference>(f3, f4);
  auto f = std::make_shared<Difference>(f12, f34);

  //update by hand
  for (const auto& fi : std::vector<std::shared_ptr<SphereFunction>>{f1, f2, f3, f4})
  {
    fi->updateValue();
    fi->updateJacobian();
    fi->updateVelocityAndNormalAcc();
  }
  for (const auto& fi : std::vector<std::shared_ptr<Difference>>{ f12, f34, f })
  {
    fi->updateValue();
    fi->updateJacobian();
    fi->updateVelocity();
    fi->updateJDot();
  }

  //store values
  auto v1 = f1->value();
  auto v2 = f2->value();
  auto v3 = f3->value();
  auto v4 = f4->value();
  auto v12 = f12->value();
  auto v34 = f34->value();
  auto v = f->value();
  auto J1 = f1->jacobian(*x);
  auto J2 = f2->jacobian(*x);
  auto J3 = f3->jacobian(*x);
  auto J4 = f4->jacobian(*x);
  auto J12 = f12->jacobian(*x);
  auto J34 = f34->jacobian(*x);
  auto J = f->jacobian(*x);
  auto dv1 = f1->velocity();
  auto dv2 = f2->velocity();
  auto dv3 = f3->velocity();
  auto dv4 = f4->velocity();
  auto dv12 = f12->velocity();
  auto dv34 = f34->velocity();
  auto dv = f->velocity();
  auto na1 = f1->normalAcceleration();
  auto na2 = f2->normalAcceleration();
  auto na3 = f3->normalAcceleration();
  auto na4 = f4->normalAcceleration();
  auto na12 = f12->normalAcceleration();
  auto na34 = f34->normalAcceleration();
  auto na = f->normalAcceleration();

  //create graphs
  auto Value = function::abstract::Function::Output::Value;
  auto Jacobian = function::abstract::Function::Output::Jacobian;
  auto Velocity = function::abstract::Function::Output::Velocity;
  auto NormaAcceleration = function::abstract::Function::Output::NormalAcceleration;
  auto g = utils::generateUpdateGraph(f12, Velocity, f34, Value, Jacobian, f, Value);

  //change variable values
  x << Vector3d::Random();
  x << Vector3d::Random();
  dx << Vector3d::Random();

  //update
  g->execute();

  //checks: values should be the same if not updated, and different otherwise
  EXPECT_NE(f1->value()[0], v1[0]);
  EXPECT_NE(f2->value()[0], v2[0]);
  EXPECT_NE(f3->value()[0], v3[0]);
  EXPECT_NE(f4->value()[0], v4[0]);
  EXPECT_NE(f12->value()[0], v12[0]);
  EXPECT_NE(f34->value()[0], v34[0]);
  EXPECT_NE(f->value()[0], v[0]);
  EXPECT_NE(f1->velocity()[0], dv1[0]);
  EXPECT_NE(f2->velocity()[0], dv2[0]);
  EXPECT_EQ(f3->velocity()[0], dv3[0]);
  EXPECT_EQ(f4->velocity()[0], dv4[0]);
  EXPECT_NE(f12->velocity()[0], dv12[0]);
  EXPECT_EQ(f34->velocity()[0], dv34[0]);
  EXPECT_EQ(f->velocity()[0], dv[0]);
  EXPECT_EQ(f1->jacobian(*x)(0), J1(0));
  EXPECT_EQ(f2->jacobian(*x)(0), J2(0));
  EXPECT_NE(f3->jacobian(*x)(0), J3(0));
  EXPECT_NE(f4->jacobian(*x)(0), J4(0));
  EXPECT_EQ(f12->jacobian(*x)(0), J12(0));
  EXPECT_DOUBLE_EQ(f34->jacobian(*x)(0), J34(0)); //for the difference of sphere functions, the jacobian is independent of x
  EXPECT_EQ(f->jacobian(*x)(0), J(0));
  EXPECT_NE(f1->normalAcceleration()[0], na1[0]);  //for SphereFunction, the normal acceleration is updated when the velocity is updated
  EXPECT_NE(f2->normalAcceleration()[0], na2[0]);  //for SphereFunction, the normal acceleration is updated when the velocity is updated
  EXPECT_EQ(f3->normalAcceleration()[0], na3[0]);
  EXPECT_EQ(f4->normalAcceleration()[0], na4[0]);
  EXPECT_EQ(f12->normalAcceleration()[0], na12[0]);
  EXPECT_EQ(f34->normalAcceleration()[0], na34[0]);
  EXPECT_EQ(f->normalAcceleration()[0], na[0]);
}
