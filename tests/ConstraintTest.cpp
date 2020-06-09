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

#include <tvm/Variable.h>
#include <tvm/constraint/BasicLinearConstraint.h>
#include <tvm/exception/exceptions.h>

TEST(ConstraintTest, constraint) {  // NOLINT
  Eigen::MatrixXd A1 = Eigen::MatrixXd::Random(4, 5);
  Eigen::MatrixXd A2 = Eigen::MatrixXd::Random(4, 2);
  Eigen::VectorXd b = Eigen::VectorXd::Random(4);

  tvm::VariablePtr x1 = tvm::Space(5).createVariable("x1");
  tvm::VariablePtr x2 = tvm::Space(2).createVariable("x2");

  x1->value(Eigen::VectorXd::Random(5));
  x2->value(Eigen::VectorXd::Random(2));

  //A1x >= 0
  tvm::constraint::BasicLinearConstraint C1(A1, x1, tvm::constraint::Type::GREATER_THAN);
  C1.updateValue();

  ASSERT_TRUE(C1.value().isApprox(A1*x1->value()));

  // [A1 A2] [x1' x2']' <= b
  tvm::constraint::BasicLinearConstraint C2({ A1,A2 }, { x1,x2 }, b, tvm::constraint::Type::LOWER_THAN);
  C2.updateValue();
  tvm::constraint::BasicLinearConstraint C3({ A2,A1 }, { x2,x1 }, b, tvm::constraint::Type::LOWER_THAN);
  C3.updateValue();

  ASSERT_TRUE(C2.value().isApprox(A1*x1->value() + A2*x2->value()));
  ASSERT_TRUE(C2.value().isApprox(C3.value()));
  ASSERT_TRUE(C2.u().isApprox(b));
}
