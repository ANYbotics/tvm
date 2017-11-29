#include <tvm/Variable.h>
#include <tvm/VariableVector.h>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define DOCTEST_CONFIG_SUPER_FAST_ASSERTS
#include "doctest/doctest.h"

using namespace tvm;

TEST_CASE("Test Variable creation")
{
  VariablePtr u = Space(3).createVariable("u");
  FAST_CHECK_EQ(u->size(), 3);
  FAST_CHECK_EQ(dot(u)->size(), 3);
  FAST_CHECK_EQ(dot(u, 4)->size(), 3);
  FAST_CHECK_UNARY(u->space().isEuclidean());
  FAST_CHECK_EQ(u->space().size(), 3);
  FAST_CHECK_EQ(u->space().rSize(), 3);
  FAST_CHECK_EQ(u->space().tSize(), 3);

  VariablePtr v = Space(3, 4, 3).createVariable("v");
  FAST_CHECK_EQ(v->size(), 4);
  FAST_CHECK_EQ(dot(v)->size(), 3);
  FAST_CHECK_EQ(dot(v,4)->size(), 3);
  FAST_CHECK_UNARY(!v->space().isEuclidean());
  FAST_CHECK_EQ(v->space().size(), 3);
  FAST_CHECK_EQ(v->space().rSize(), 4);
  FAST_CHECK_EQ(v->space().tSize(), 3);
}

TEST_CASE("Test Variable value")
{
  VariablePtr v = Space(3).createVariable("v");
  Eigen::Vector3d val = Eigen::Vector3d::Random();
  v->value(val);

  FAST_CHECK_UNARY(v->value().isApprox(val));
  CHECK_THROWS(v->value(Eigen::VectorXd(5)));
}

TEST_CASE("Test Variable Derivatives")
{
  VariablePtr v = Space(3).createVariable("v");
  FAST_CHECK_EQ(v->derivativeNumber(), 0);
  FAST_CHECK_UNARY(v->isBasePrimitive());
  FAST_CHECK_UNARY(!v->basePrimitive());
  CHECK_THROWS(v->primitive());

  auto dv = dot(v);
  FAST_CHECK_EQ(dv->space(), v->space());
  FAST_CHECK_EQ(dv->derivativeNumber(), 1);
  FAST_CHECK_UNARY(!dv->isBasePrimitive());
  FAST_CHECK_UNARY(dv->basePrimitive());
  FAST_CHECK_EQ(dv->basePrimitive(), v);
  FAST_CHECK_EQ(dv->primitive(), v);
  CHECK_THROWS(dv->primitive<3>());

  auto dv3 = dot(v, 3);
  FAST_CHECK_EQ(dv3->space(), v->space());
  FAST_CHECK_EQ(dv3->derivativeNumber(), 3);
  FAST_CHECK_UNARY(!dv3->isBasePrimitive());
  FAST_CHECK_UNARY(dv3->basePrimitive());
  FAST_CHECK_EQ(dv3->primitive<3>(), v);
  FAST_CHECK_EQ(dv3->basePrimitive(), v);
  FAST_CHECK_EQ(dot(dv, 2), dv3);
  FAST_CHECK_EQ(dot(dv), dv3->primitive());
}

TEST_CASE("Test Variable Name")
{
  VariablePtr v = Space(3).createVariable("v");
  FAST_CHECK_EQ(v->name(), "v");
  auto dv = dot(v);
  FAST_CHECK_EQ(dv->name(), "d v / dt");
  auto dv3 = dot(dv, 2);
  FAST_CHECK_EQ(dv3->name(), "d3 v / dt3");
  auto dv5 = dot(v, 5);
  FAST_CHECK_EQ(dv5->name(), "d5 v / dt5");
  auto dv4 = dot(dv, 3);
  FAST_CHECK_EQ(dv4->name(), "d4 v / dt4");
}

TEST_CASE("Test VariableVector creation")
{
  VariablePtr v1 = Space(3).createVariable("v1");
  VariablePtr v2 = Space(4).createVariable("v2");
  VariablePtr v3 = Space(2).createVariable("v3");
  VariablePtr v4 = Space(3).createVariable("v4");

  VariableVector vv1;
  vv1.add(v2);
  vv1.add(v3);
  vv1.add(v1);

  FAST_CHECK_EQ(vv1.numberOfVariables(), 3);
  FAST_CHECK_EQ(vv1.size(), 9);
  FAST_CHECK_EQ(vv1[0], v2);
  FAST_CHECK_EQ(vv1[1], v3);
  FAST_CHECK_EQ(vv1[2], v1);
  FAST_CHECK_UNARY(vv1.contains(*v1));
  FAST_CHECK_UNARY(vv1.contains(*v2));
  FAST_CHECK_UNARY(vv1.contains(*v3));
  FAST_CHECK_UNARY(!vv1.contains(*v4));

  vv1.remove(*v3);
  FAST_CHECK_EQ(vv1.numberOfVariables(), 2);
  FAST_CHECK_EQ(vv1.size(), 7);
  FAST_CHECK_EQ(vv1[0], v2);
  FAST_CHECK_EQ(vv1[1], v1);
  FAST_CHECK_UNARY(vv1.contains(*v1));
  FAST_CHECK_UNARY(vv1.contains(*v2));
  FAST_CHECK_UNARY(!vv1.contains(*v3));
  FAST_CHECK_UNARY(!vv1.contains(*v4));

  VariableVector vv2({ v1, v2, v3 });
  FAST_CHECK_EQ(vv2.numberOfVariables(), 3);
  FAST_CHECK_EQ(vv2.size(), 9);
  FAST_CHECK_EQ(vv2[0], v1);
  FAST_CHECK_EQ(vv2[1], v2);
  FAST_CHECK_EQ(vv2[2], v3);

  CHECK_THROWS(vv2.add(v1));
  CHECK_NOTHROW(vv2.add(v1, true));
  CHECK_THROWS(vv2.remove(*v4));
  CHECK_NOTHROW(vv2.remove(*v4, true));

  std::vector<VariablePtr> vec = { v1, v2 };
  std::vector<VariablePtr> vec2 = { v1, v3, v4 };
  VariableVector vv3(vec);
  FAST_CHECK_EQ(vv3.numberOfVariables(), 2);
  FAST_CHECK_EQ(vv3.size(), 7);
  FAST_CHECK_EQ(vv3[0], v1);
  FAST_CHECK_EQ(vv3[1], v2);
  CHECK_THROWS(vv3.add(vec2));
  CHECK_NOTHROW(vv3.add(vec2, true));
  FAST_CHECK_EQ(vv3.numberOfVariables(), 4);
  FAST_CHECK_EQ(vv3.size(), 12);
  FAST_CHECK_EQ(vv3[0], v1);
  FAST_CHECK_EQ(vv3[1], v2);
  FAST_CHECK_EQ(vv3[2], v3);
  FAST_CHECK_EQ(vv3[3], v4);
  FAST_CHECK_UNARY(vv3.contains(*v1));
  FAST_CHECK_UNARY(vv3.contains(*v2));
  FAST_CHECK_UNARY(vv3.contains(*v3));
  FAST_CHECK_UNARY(vv3.contains(*v4));
}