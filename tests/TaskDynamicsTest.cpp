#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define DOCTEST_CONFIG_SUPER_FAST_ASSERTS
#include "doctest/doctest.h"

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

TEST_CASE("Valid construction")
{
  VariablePtr x = Space(3).createVariable("x");
  x << Vector3d::Zero();
  auto f = std::make_shared<function::IdentityFunction>(x);
  task_dynamics::Constant td;
  CHECK_NOTHROW(td.impl(f, constraint::Type::EQUAL, Vector3d(1, 0, 0)));
  CHECK_THROWS(td.impl(f, constraint::Type::EQUAL, Vector2d(1, 0)));
}

TEST_CASE("Test Constant")
{
  VariablePtr x = Space(3).createVariable("x");
  x << Vector3d::Zero();
  auto f = std::make_shared<function::IdentityFunction>(x);
  task_dynamics::Constant td;
  auto tdi = td.impl(f,constraint::Type::EQUAL, Vector3d(1,0,0));

  tdi->updateValue();

  FAST_CHECK_EQ(tdi->order(), task_dynamics::Order::Zero);
  FAST_CHECK_UNARY(tdi->value().isApprox(Vector3d(1, 0, 0)));
  FAST_CHECK_UNARY(tdi->checkType<task_dynamics::Constant>());
  FAST_CHECK_UNARY_FALSE(tdi->checkType<task_dynamics::None>());
}

TEST_CASE("Test None")
{
  VariablePtr x = Space(3).createVariable("x");
  x << 1,2,3;
  MatrixXd A = MatrixXd::Random(2, 3);
  Vector2d b(1, 2);
  auto f = std::make_shared<function::BasicLinearFunction>(A, x, b);
  task_dynamics::None td;
  auto tdi = td.impl(f, constraint::Type::EQUAL, Vector2d(1, 0));

  f->updateValue();
  tdi->updateValue();

  FAST_CHECK_EQ(tdi->order(), task_dynamics::Order::Zero);
  FAST_CHECK_UNARY(tdi->value().isApprox(Vector2d(0,-2)));
  FAST_CHECK_UNARY(tdi->checkType<task_dynamics::None>());
  FAST_CHECK_UNARY_FALSE(tdi->checkType<task_dynamics::Constant>());
}

TEST_CASE("Test Proportional")
{
  VariablePtr x = Space(3).createVariable("x");
  x << 1, 2, 3;
  auto f = std::make_shared<SphereFunction>(x, Vector3d(1, 0, -3), 2);

  double kp = 2;
  task_dynamics::P td(kp);
  VectorXd rhs(1); rhs[0] = -1;
  auto tdi = td.impl(f, constraint::Type::EQUAL, rhs);

  f->updateValue();
  tdi->updateValue();

  FAST_CHECK_EQ(tdi->order(), task_dynamics::Order::One);
  FAST_CHECK_EQ(tdi->value()[0], -kp*(36 - rhs[0]));  // -kp*||(1,2,3) - (1,0,-3)||^2 - 2^2 - rhs
  FAST_CHECK_UNARY(tdi->checkType<task_dynamics::P>());
  FAST_CHECK_UNARY_FALSE(tdi->checkType<task_dynamics::Constant>());
}

TEST_CASE("Test Proportional Derivative")
{
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

  FAST_CHECK_EQ(tdi->order(), task_dynamics::Order::Two);
  FAST_CHECK_EQ(tdi->value()[0], -kp*(36 - rhs[0])-kv*16);
  FAST_CHECK_UNARY(tdi->checkType<task_dynamics::PD>());
  FAST_CHECK_UNARY_FALSE(tdi->checkType<task_dynamics::Constant>());
}

TEST_CASE("Test Velocity Damper")
{
  VariablePtr x = Space(3).createVariable("x");
  VariablePtr dx = dot(x);
  auto f = std::make_shared<function::IdentityFunction>(x);

  double di = 3;
  double ds = 1;
  double xsi = 2;
  double dt = 0.1;

  //test validity
  CHECK_THROWS(task_dynamics::VelocityDamper(false, 0.5, ds, xsi));
  CHECK_THROWS(task_dynamics::VelocityDamper(dt, false, 0.5, ds, xsi, 1000));
  CHECK_THROWS(task_dynamics::VelocityDamper(false, di, ds, -1));
  CHECK_THROWS(task_dynamics::VelocityDamper(dt, false, di, ds, -1, 1000));
  CHECK_THROWS(task_dynamics::VelocityDamper(0, false, di, ds, xsi, 1000));
  CHECK_THROWS(task_dynamics::VelocityDamper(-0.1, false, di, ds, xsi, 1000));

  //test kinematics
  task_dynamics::VelocityDamper td1(false, di, ds, xsi);
  {
    x << 1, 2, 4;
    auto tdl = td1.impl(f, constraint::Type::GREATER_THAN, Vector3d::Zero());

    f->updateValue();
    tdl->updateValue();

    FAST_CHECK_EQ(tdl->order(), task_dynamics::Order::One);
    FAST_CHECK_EQ(tdl->value()[0], 0);
    FAST_CHECK_EQ(tdl->value()[1], -1);
    FAST_CHECK_EQ(tdl->value()[2], -constant::big_number);
    FAST_CHECK_UNARY(tdl->checkType<task_dynamics::VelocityDamper>());
    FAST_CHECK_UNARY_FALSE(tdl->checkType<task_dynamics::Constant>());


    x << -1, -2, -4;
    auto tdu = td1.impl(f, constraint::Type::LOWER_THAN, Vector3d::Zero());
    f->updateValue();
    tdu->updateValue();
    FAST_CHECK_EQ(tdu->value()[0], 0);
    FAST_CHECK_EQ(tdu->value()[1], 1);
    FAST_CHECK_EQ(tdu->value()[2], constant::big_number);
  }


  //test dynamics
  double big = 1000;
  task_dynamics::VelocityDamper td2(dt, false, di, ds, xsi, big);
  {
    x << 1, 2, 4;
    dx << 1, 1, 1;
    auto tdl = td2.impl(f, constraint::Type::GREATER_THAN, Vector3d::Zero());
    
    f->updateValue();
    f->updateVelocity();
    tdl->updateValue();

    FAST_CHECK_EQ(tdl->order(), task_dynamics::Order::Two);
    FAST_CHECK_UNARY(tdl->value().isApprox(Vector3d(-10, -20, -1000)));
    FAST_CHECK_UNARY(tdl->checkType<task_dynamics::VelocityDamper>());
    FAST_CHECK_UNARY_FALSE(tdl->checkType<task_dynamics::Constant>());

    x << -1, -2, -4;
    dx << -1, -1, -1;
    auto tdu = td2.impl(f, constraint::Type::LOWER_THAN, Vector3d::Zero());

    f->updateValue();
    f->updateVelocity();
    tdu->updateValue();
    FAST_CHECK_UNARY(tdu->value().isApprox(Vector3d(10, 20, 1000)));
  }

}

TEST_CASE("Test automatic xsi")
{
  VariablePtr x = Space(3).createVariable("x");
  VariablePtr dx = dot(x);
  auto f = std::make_shared<function::IdentityFunction>(x);

  double di = 3;
  double ds = 1;
  double xsi = 1;
  double dt = 0.1;

  //test kinematics
  double big = 100;
  task_dynamics::VelocityDamper td1(true, di, ds, xsi, big);
  {
    x << 5, 4, 2;
    dx << -0.5, -0.5, -0.5;
    TaskDynamicsPtr tdl = td1.impl(f, constraint::Type::GREATER_THAN, Vector3d::Zero());

    auto Value = task_dynamics::abstract::TaskDynamicsImpl::Output::Value;
    auto gl = utils::generateUpdateGraph(tdl, Value);

    gl->execute();
    FAST_CHECK_UNARY(tdl->value().isApprox(Vector3d(-big, -big, -1)));

    x << 4.5, 3.5, 1.5;
    gl->execute();
    FAST_CHECK_UNARY(tdl->value().isApprox(Vector3d(-big, -big, -0.5)));

    x << 4, 3, 1;
    gl->execute();
    FAST_CHECK_UNARY(tdl->value().isApprox(Vector3d(-big, -1.5, 0)));

    //we check that two consecutive updates with the same variable values give the same results.
    gl->execute();
    FAST_CHECK_UNARY(tdl->value().isApprox(Vector3d(-big, -1.5, 0)));

    dx << -0.5, -0.5, 0;
    x << 3.5, 2.5, 1;
    gl->execute();
    FAST_CHECK_UNARY(tdl->value().isApprox(Vector3d(-big, -1.125, 0)));

    x << 3, 2, 1;
    gl->execute();
    FAST_CHECK_UNARY(tdl->value().isApprox(Vector3d(-1.5, -0.75, 0)));

    x << -5, -4, -2;
    dx << 0.5, 0.5, 0.5;
    TaskDynamicsPtr tdu = td1.impl(f, constraint::Type::LOWER_THAN, Vector3d::Zero());

    auto gu = utils::generateUpdateGraph(tdu, Value);

    gu->execute();
    FAST_CHECK_UNARY(tdu->value().isApprox(Vector3d(big, big, 1)));

    x << -4.5, -3.5, -1.5;
    gu->execute();
    FAST_CHECK_UNARY(tdu->value().isApprox(Vector3d(big, big, 0.5)));

    x << -4, -3, -1;
    gu->execute();
    FAST_CHECK_UNARY(tdu->value().isApprox(Vector3d(big, 1.5, 0)));

    dx << 0.5, 0.5, 0;
    x << -3.5, -2.5, -1;
    gu->execute();
    FAST_CHECK_UNARY(tdu->value().isApprox(Vector3d(big, 1.125, 0)));

    x << -3, -2, -1;
    gu->execute();
    FAST_CHECK_UNARY(tdu->value().isApprox(Vector3d(1.5, 0.75, 0)));
  }
}