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

#include <tvm/requirements/SolvingRequirements.h>

#include <iostream>

using namespace tvm::requirements;
using namespace Eigen;

bool checkRequirements(const SolvingRequirements& sr,
                       bool defaultPriority, int priority,
                       bool defaultWeight, double weight,
                       bool defaultAWeight, const VectorXd& aweight,
                       bool defaultEval, ViolationEvaluationType type)
{
  bool b = (sr.priorityLevel().isDefault() == defaultPriority);
  b = b && (sr.priorityLevel().value() == priority);
  b = b && (sr.weight().isDefault() == defaultWeight);
  b = b && (sr.weight().value() == weight);
  b = b && (sr.anisotropicWeight().isDefault() == defaultAWeight);
  b = b && (sr.anisotropicWeight().value() == aweight);
  b = b && (sr.violationEvaluation().isDefault() == defaultEval);
  b = b && (sr.violationEvaluation().value() == type);
  return b;
}

TEST(SolvingRequirementsTest, solvingRequirements) {  // NOLINT
  SolvingRequirements s0;
  EXPECT_TRUE(checkRequirements(s0, true, 0, true, 1, true, VectorXd(), true, ViolationEvaluationType::L2));

  SolvingRequirements s1(PriorityLevel(2), Weight(3));
  EXPECT_TRUE(checkRequirements(s1, false, 2, false, 3, true, VectorXd(), true, ViolationEvaluationType::L2));

  SolvingRequirements s2(PriorityLevel(2), Weight(1));
  EXPECT_TRUE(checkRequirements(s2, false, 2, false, 1, true, VectorXd(), true, ViolationEvaluationType::L2));

  SolvingRequirements s3(Weight(3), PriorityLevel(2));
  EXPECT_TRUE(checkRequirements(s3, false, 2, false, 3, true, VectorXd(), true, ViolationEvaluationType::L2));

  SolvingRequirements s4(AnisotropicWeight((VectorXd(3) << 3,4,5).finished()), Weight(3), ViolationEvaluation(ViolationEvaluationType::L1), PriorityLevel(2));
  EXPECT_TRUE(checkRequirements(s4, false, 2, false, 3, false, (VectorXd(3) << 3, 4, 5).finished(), false, ViolationEvaluationType::L1));

  EXPECT_THROW(SolvingRequirements s(PriorityLevel(-1)), std::runtime_error);

  EXPECT_THROW(SolvingRequirements s(Weight(-1)), std::runtime_error);

  EXPECT_THROW(SolvingRequirements s(AnisotropicWeight(Vector3d(-1,2,3))), std::runtime_error);
}
