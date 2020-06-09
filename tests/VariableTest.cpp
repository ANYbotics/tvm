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

#include <tvm/Range.h>
#include <tvm/Variable.h>
#include <tvm/VariableVector.h>

using namespace tvm;

TEST(VariableTest, variableCreation) {  // NOLINT
  VariablePtr u = Space(3).createVariable("u");
  EXPECT_EQ(u->size(), 3);
  EXPECT_EQ(dot(u)->size(), 3);
  EXPECT_EQ(dot(u, 4)->size(), 3);
  EXPECT_TRUE(u->space().isEuclidean());
  EXPECT_EQ(u->space().size(), 3);
  EXPECT_EQ(u->space().rSize(), 3);
  EXPECT_EQ(u->space().tSize(), 3);
  EXPECT_TRUE(u->isEuclidean());
  EXPECT_TRUE(dot(u)->isEuclidean());

  VariablePtr v = Space(3, 4, 3).createVariable("v");
  EXPECT_EQ(v->size(), 4);
  EXPECT_EQ(dot(v)->size(), 3);
  EXPECT_EQ(dot(v,4)->size(), 3);
  EXPECT_TRUE(!v->space().isEuclidean());
  EXPECT_EQ(v->space().size(), 3);
  EXPECT_EQ(v->space().rSize(), 4);
  EXPECT_EQ(v->space().tSize(), 3);
  EXPECT_FALSE(v->isEuclidean());
  EXPECT_TRUE(dot(v)->isEuclidean());

  VariablePtr w = v->duplicate("w");
  EXPECT_NE(v, w);
  EXPECT_EQ(v->space(), w->space());
  EXPECT_FALSE(w->isEuclidean());
  EXPECT_TRUE(dot(w)->isEuclidean());
}

TEST(VariableTest, variableValue) {  // NOLINT
  {
    VariablePtr v = Space(3).createVariable("v");
    Eigen::Vector3d val = Eigen::Vector3d::Random();
    v->value(val);

    EXPECT_TRUE(v->value().isApprox(val));
    EXPECT_ANY_THROW(v->value(Eigen::VectorXd(5)));
  }
  {
    VariablePtr v = Space(3).createVariable("v");
    Eigen::Vector3d val(1,2,3);
    v << val;
    EXPECT_TRUE(v->value().isApprox(val));
  }
  {
    VariablePtr v = Space(3).createVariable("v");
    Eigen::VectorXd val(5); val << 1, 2, 3, 4, 5;
    v << val.head(3);
    EXPECT_TRUE(v->value().isApprox(Eigen::Vector3d(1, 2, 3)));
  }
  {
    VariablePtr v = Space(3).createVariable("v");
    v << 1, 2, 3;
    EXPECT_TRUE(v->value().isApprox(Eigen::Vector3d(1,2,3)));
  }
  {
    VariablePtr v = Space(3).createVariable("v");
    Eigen::VectorXd val(5); val << 1, 2, 3, 4, 5;
    v << val.tail(2), 6;
    EXPECT_TRUE(v->value().isApprox(Eigen::Vector3d(4, 5, 6)));
  }
}

TEST(VariableTest, variableDerivatives) {  // NOLINT
  VariablePtr v = Space(3).createVariable("v");
  EXPECT_EQ(v->derivativeNumber(), 0);
  EXPECT_TRUE(v->isBasePrimitive());
  EXPECT_EQ(v->basePrimitive(), v);
  EXPECT_ANY_THROW(v->primitive());

  auto dv = dot(v);
  EXPECT_EQ(dv->space(), v->space());
  EXPECT_EQ(dv->derivativeNumber(), 1);
  EXPECT_TRUE(!dv->isBasePrimitive());
  EXPECT_EQ(dv->basePrimitive(), v);
  EXPECT_EQ(dv->primitive(), v);
  EXPECT_ANY_THROW(dv->primitive<3>());

  auto dv3 = dot(v, 3);
  EXPECT_EQ(dv3->space(), v->space());
  EXPECT_EQ(dv3->derivativeNumber(), 3);
  EXPECT_TRUE(!dv3->isBasePrimitive());
  EXPECT_EQ(dv3->primitive<3>(), v);
  EXPECT_EQ(dv3->basePrimitive(), v);
  EXPECT_EQ(dot(dv, 2), dv3);
  EXPECT_EQ(dot(dv), dv3->primitive());

  VariablePtr u = Space(4).createVariable("u");
  EXPECT_TRUE(dv->isDerivativeOf(*v));
  EXPECT_FALSE(v->isDerivativeOf(*dv));
  EXPECT_FALSE(dv->isDerivativeOf(*u));
  EXPECT_FALSE(dv->isDerivativeOf(*dv));
  EXPECT_TRUE(v->isPrimitiveOf(*dv));
  EXPECT_FALSE(dv->isPrimitiveOf(*v));
  EXPECT_FALSE(dv->isPrimitiveOf(*u));
  EXPECT_FALSE(dv->isPrimitiveOf(*dv));
  EXPECT_TRUE(dv3->isDerivativeOf(*v));
  EXPECT_TRUE(dv3->isDerivativeOf(*dv));
}

TEST(VariableTest, variableName) {  // NOLINT
  VariablePtr v = Space(3).createVariable("v");
  EXPECT_EQ(v->name(), "v");
  auto dv = dot(v);
  EXPECT_EQ(dv->name(), "d v / dt");
  auto dv3 = dot(dv, 2);
  EXPECT_EQ(dv3->name(), "d3 v / dt3");
  auto dv5 = dot(v, 5);
  EXPECT_EQ(dv5->name(), "d5 v / dt5");
  auto dv4 = dot(dv, 3);
  EXPECT_EQ(dv4->name(), "d4 v / dt4");
  auto du3 = dv3->duplicate();
  EXPECT_EQ(du3->name(), "d3 v' / dt3");
  auto dw3 = dv3->duplicate("w");
  EXPECT_EQ(dw3->name(), "d3 w / dt3");
}

TEST(VariableTest, variableVectorCreation) {  // NOLINT
  VariablePtr v1 = Space(3).createVariable("v1");
  VariablePtr v2 = Space(4).createVariable("v2");
  VariablePtr v3 = Space(2).createVariable("v3");
  VariablePtr v4 = Space(3).createVariable("v4");

  VariableVector vv1;
  vv1.add(v2);
  vv1.add(v3);
  vv1.add(v1);

  int i1, i2, i3, i4;
  EXPECT_EQ(vv1.numberOfVariables(), 3);
  EXPECT_EQ(vv1.totalSize(), 9);
  EXPECT_EQ(vv1[0], v2);
  EXPECT_EQ(vv1[1], v3);
  EXPECT_EQ(vv1[2], v1);
  EXPECT_TRUE(vv1.contains(*v1));
  EXPECT_TRUE(vv1.contains(*v2));
  EXPECT_TRUE(vv1.contains(*v3));
  EXPECT_TRUE(!vv1.contains(*v4));
  EXPECT_EQ(vv1.indexOf(*v1), 2);
  EXPECT_EQ(vv1.indexOf(*v2), 0);
  EXPECT_EQ(vv1.indexOf(*v3), 1);
  EXPECT_EQ(vv1.indexOf(*v4), -1);

  vv1.remove(*v3);
  EXPECT_EQ(vv1.numberOfVariables(), 2);
  EXPECT_EQ(vv1.totalSize(), 7);
  EXPECT_EQ(vv1[0], v2);
  EXPECT_EQ(vv1[1], v1);
  EXPECT_TRUE(vv1.contains(*v1));
  EXPECT_TRUE(vv1.contains(*v2));
  EXPECT_TRUE(!vv1.contains(*v3));
  EXPECT_TRUE(!vv1.contains(*v4));
  EXPECT_EQ(vv1.indexOf(*v1), 1);
  EXPECT_EQ(vv1.indexOf(*v2), 0);
  EXPECT_EQ(vv1.indexOf(*v3), -1);
  EXPECT_EQ(vv1.indexOf(*v4), -1);

  VariableVector vv2({ v1, v2, v3 });
  EXPECT_EQ(vv2.numberOfVariables(), 3);
  EXPECT_EQ(vv2.totalSize(), 9);
  EXPECT_EQ(vv2[0], v1);
  EXPECT_EQ(vv2[1], v2);
  EXPECT_EQ(vv2[2], v3);

  EXPECT_TRUE(vv2.add(v1) == false);
  EXPECT_TRUE(vv2.remove(*v4) == false);

  std::vector<VariablePtr> vec = { v1, v2 };
  std::vector<VariablePtr> vec2 = { v1, v3, v4 };
  VariableVector vv3(vec);
  for(const auto & v : vec) { vv3.add(v); }
  EXPECT_EQ(vv3.numberOfVariables(), 2);
  EXPECT_EQ(vv3.totalSize(), 7);
  EXPECT_EQ(vv3[0], v1);
  EXPECT_EQ(vv3[1], v2);
  for(const auto & v : vec2) { vv3.add(v); }
  EXPECT_EQ(vv3.numberOfVariables(), 4);
  EXPECT_EQ(vv3.totalSize(), 12);
  EXPECT_EQ(vv3[0], v1);
  EXPECT_EQ(vv3[1], v2);
  EXPECT_EQ(vv3[2], v3);
  EXPECT_EQ(vv3[3], v4);
  EXPECT_TRUE(vv3.contains(*v1));
  EXPECT_TRUE(vv3.contains(*v2));
  EXPECT_TRUE(vv3.contains(*v3));
  EXPECT_TRUE(vv3.contains(*v4));
}

TEST(VariableTest, mapping) {  // NOLINT
  VariablePtr v1 = Space(3).createVariable("v1");
  VariablePtr v2 = Space(4).createVariable("v2");
  VariablePtr v3 = Space(2).createVariable("v3");
  VariablePtr v4 = Space(3).createVariable("v4");

  VariableVector vv1;
  int s = vv1.stamp();
  vv1.add(v1);
  vv1.add(v2);
  vv1.add(v3);

  VariableVector vv2;
  vv2.add(v3);
  vv2.add(v2);
  vv2.add(v1);

  EXPECT_EQ(vv1.stamp(), s + 3);
  EXPECT_EQ(vv2.stamp(), s + 7);

  EXPECT_EQ(v1->getMappingIn(vv1), Range( 0, 3 ));
  EXPECT_EQ(v2->getMappingIn(vv1), Range( 3, 4 ));
  EXPECT_EQ(v3->getMappingIn(vv1), Range( 7, 2 ));
  EXPECT_ANY_THROW(v4->getMappingIn(vv1));
  EXPECT_EQ(v1->getMappingIn(vv2), Range( 6, 3 ));
  EXPECT_EQ(v2->getMappingIn(vv2), Range( 2, 4 ));
  EXPECT_EQ(v3->getMappingIn(vv2), Range( 0, 2 ));
  EXPECT_ANY_THROW(v4->getMappingIn(vv2));

  EXPECT_EQ(vv1.stamp(), s + 3);
  EXPECT_EQ(vv2.stamp(), s + 7);

  vv1.add(v4);
  EXPECT_EQ(vv1.stamp(), s + 8);
  EXPECT_EQ(v1->getMappingIn(vv1), Range( 0, 3 ));
  EXPECT_EQ(v2->getMappingIn(vv1), Range( 3, 4 ));
  EXPECT_EQ(v3->getMappingIn(vv1), Range( 7, 2 ));
  EXPECT_EQ(v4->getMappingIn(vv1), Range( 9, 3 ));
  vv1.remove(*v2);
  EXPECT_EQ(vv1.stamp(), s + 9);
  EXPECT_EQ(v1->getMappingIn(vv1), Range( 0, 3 ));
  EXPECT_ANY_THROW(v2->getMappingIn(vv1));
  EXPECT_EQ(v3->getMappingIn(vv1), Range( 3, 2 ));
  EXPECT_EQ(v4->getMappingIn(vv1), Range( 5, 3 ));
  EXPECT_EQ(v1->getMappingIn(vv2), Range( 6, 3 ));
  EXPECT_EQ(v2->getMappingIn(vv2), Range( 2, 4 ));
  EXPECT_EQ(v3->getMappingIn(vv2), Range( 0, 2 ));
  EXPECT_ANY_THROW(v4->getMappingIn(vv2));
}

TEST(VariableTest, variableVectorDerivation) {  // NOLINT
  VariablePtr v1 = Space(3).createVariable("v1");
  VariablePtr v2 = Space(4).createVariable("v2");
  VariablePtr v3 = Space(2).createVariable("v3");

  VariableVector vv;
  vv.add(v1);
  vv.add(v2);
  vv.add(v3);

  auto dvv = dot(vv);
  EXPECT_EQ(dvv[0], dot(v1));
  EXPECT_EQ(dvv[1], dot(v2));
  EXPECT_EQ(dvv[2], dot(v3));

  auto dvv3 = dot(vv, 3);
  EXPECT_EQ(dvv3[0], dot(v1, 3));
  EXPECT_EQ(dvv3[1], dot(v2, 3));
  EXPECT_EQ(dvv3[2], dot(v3, 3));

  auto dvv3b = dot(dvv, 2);
  EXPECT_EQ(dvv3b[0], dot(v1, 3));
  EXPECT_EQ(dvv3b[1], dot(v2, 3));
  EXPECT_EQ(dvv3b[2], dot(v3, 3));
}
