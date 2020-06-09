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

#include <tvm/internal/MatrixProperties.h>
#include <tvm/internal/MatrixWithProperties.h>

using namespace tvm::internal;

TEST(MatrixPropertiesTest, shapeProperties) {  // NOLINT
  MatrixProperties p0;
  EXPECT_EQ(p0.shape(), MatrixProperties::GENERAL);
  EXPECT_EQ(p0.positiveness(), MatrixProperties::NA);
  EXPECT_FALSE(p0.isConstant());
  EXPECT_FALSE(p0.isDiagonal());
  EXPECT_FALSE(p0.isIdentity());
  EXPECT_FALSE(p0.isInvertible());
  EXPECT_FALSE(p0.isMinusIdentity());
  EXPECT_FALSE(p0.isMultipleOfIdentity());
  EXPECT_FALSE(p0.isNegativeDefinite());
  EXPECT_FALSE(p0.isNegativeSemidefinite());
  EXPECT_FALSE(p0.isNonZeroIndefinite());
  EXPECT_FALSE(p0.isPositiveDefinite());
  EXPECT_FALSE(p0.isPositiveSemiDefinite());
  EXPECT_FALSE(p0.isSymmetric());
  EXPECT_FALSE(p0.isIndefinite());
  EXPECT_FALSE(p0.isZero());
  EXPECT_FALSE(p0.isTriangular());
  EXPECT_FALSE(p0.isLowerTriangular());
  EXPECT_FALSE(p0.isUpperTriangular());

  MatrixProperties p1(MatrixProperties::LOWER_TRIANGULAR);
  EXPECT_EQ(p1.shape(), MatrixProperties::LOWER_TRIANGULAR);
  EXPECT_EQ(p1.positiveness(), MatrixProperties::NA);
  EXPECT_FALSE(p1.isConstant());
  EXPECT_FALSE(p1.isDiagonal());
  EXPECT_FALSE(p1.isIdentity());
  EXPECT_FALSE(p1.isInvertible());
  EXPECT_FALSE(p1.isMinusIdentity());
  EXPECT_FALSE(p1.isMultipleOfIdentity());
  EXPECT_FALSE(p1.isNegativeDefinite());
  EXPECT_FALSE(p1.isNegativeSemidefinite());
  EXPECT_FALSE(p1.isNonZeroIndefinite());
  EXPECT_FALSE(p1.isPositiveDefinite());
  EXPECT_FALSE(p1.isPositiveSemiDefinite());
  EXPECT_FALSE(p1.isSymmetric());
  EXPECT_FALSE(p1.isIndefinite());
  EXPECT_FALSE(p1.isZero());
  EXPECT_TRUE(p1.isTriangular());
  EXPECT_TRUE(p1.isLowerTriangular());
  EXPECT_FALSE(p1.isUpperTriangular());

  MatrixProperties p2(MatrixProperties::UPPER_TRIANGULAR);
  EXPECT_EQ(p2.shape(), MatrixProperties::UPPER_TRIANGULAR);
  EXPECT_EQ(p2.positiveness(), MatrixProperties::NA);
  EXPECT_FALSE(p2.isConstant());
  EXPECT_FALSE(p2.isDiagonal());
  EXPECT_FALSE(p2.isIdentity());
  EXPECT_FALSE(p2.isInvertible());
  EXPECT_FALSE(p2.isMinusIdentity());
  EXPECT_FALSE(p2.isMultipleOfIdentity());
  EXPECT_FALSE(p2.isNegativeDefinite());
  EXPECT_FALSE(p2.isNegativeSemidefinite());
  EXPECT_FALSE(p2.isNonZeroIndefinite());
  EXPECT_FALSE(p2.isPositiveDefinite());
  EXPECT_FALSE(p2.isPositiveSemiDefinite());
  EXPECT_FALSE(p2.isSymmetric());
  EXPECT_FALSE(p2.isIndefinite());
  EXPECT_FALSE(p2.isZero());
  EXPECT_TRUE(p2.isTriangular());
  EXPECT_FALSE(p2.isLowerTriangular());
  EXPECT_TRUE(p2.isUpperTriangular());

  MatrixProperties p3(MatrixProperties::DIAGONAL);
  EXPECT_EQ(p3.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p3.positiveness(), MatrixProperties::INDEFINITE);
  EXPECT_FALSE(p3.isConstant());
  EXPECT_TRUE(p3.isDiagonal());
  EXPECT_FALSE(p3.isIdentity());
  EXPECT_FALSE(p3.isInvertible());
  EXPECT_FALSE(p3.isMinusIdentity());
  EXPECT_FALSE(p3.isMultipleOfIdentity());
  EXPECT_FALSE(p3.isNegativeDefinite());
  EXPECT_FALSE(p3.isNegativeSemidefinite());
  EXPECT_FALSE(p3.isNonZeroIndefinite());
  EXPECT_FALSE(p3.isPositiveDefinite());
  EXPECT_FALSE(p3.isPositiveSemiDefinite());
  EXPECT_TRUE(p3.isSymmetric());
  EXPECT_TRUE(p3.isIndefinite());
  EXPECT_FALSE(p3.isZero());
  EXPECT_TRUE(p3.isTriangular());
  EXPECT_TRUE(p3.isLowerTriangular());
  EXPECT_TRUE(p3.isUpperTriangular());

  MatrixProperties p4(MatrixProperties::MULTIPLE_OF_IDENTITY);
  EXPECT_EQ(p4.shape(), MatrixProperties::MULTIPLE_OF_IDENTITY);
  EXPECT_EQ(p4.positiveness(), MatrixProperties::INDEFINITE);
  EXPECT_FALSE(p4.isConstant());
  EXPECT_TRUE(p4.isDiagonal());
  EXPECT_FALSE(p4.isIdentity());
  EXPECT_FALSE(p4.isInvertible());
  EXPECT_FALSE(p4.isMinusIdentity());
  EXPECT_TRUE(p4.isMultipleOfIdentity());
  EXPECT_FALSE(p4.isNegativeDefinite());
  EXPECT_FALSE(p4.isNegativeSemidefinite());
  EXPECT_FALSE(p4.isNonZeroIndefinite());
  EXPECT_FALSE(p4.isPositiveDefinite());
  EXPECT_FALSE(p4.isPositiveSemiDefinite());
  EXPECT_TRUE(p4.isSymmetric());
  EXPECT_TRUE(p4.isIndefinite());
  EXPECT_FALSE(p4.isZero());
  EXPECT_TRUE(p4.isTriangular());
  EXPECT_TRUE(p4.isLowerTriangular());
  EXPECT_TRUE(p4.isUpperTriangular());

  MatrixProperties p5(MatrixProperties::IDENTITY);
  EXPECT_EQ(p5.shape(), MatrixProperties::IDENTITY);
  EXPECT_EQ(p5.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_TRUE(p5.isConstant());
  EXPECT_TRUE(p5.isDiagonal());
  EXPECT_TRUE(p5.isIdentity());
  EXPECT_TRUE(p5.isInvertible());
  EXPECT_FALSE(p5.isMinusIdentity());
  EXPECT_TRUE(p5.isMultipleOfIdentity());
  EXPECT_FALSE(p5.isNegativeDefinite());
  EXPECT_FALSE(p5.isNegativeSemidefinite());
  EXPECT_TRUE(p5.isNonZeroIndefinite());
  EXPECT_TRUE(p5.isPositiveDefinite());
  EXPECT_TRUE(p5.isPositiveSemiDefinite());
  EXPECT_TRUE(p5.isSymmetric());
  EXPECT_TRUE(p5.isIndefinite());
  EXPECT_FALSE(p5.isZero());
  EXPECT_TRUE(p5.isTriangular());
  EXPECT_TRUE(p5.isLowerTriangular());
  EXPECT_TRUE(p5.isUpperTriangular());

  MatrixProperties p6(MatrixProperties::MINUS_IDENTITY);
  EXPECT_EQ(p6.shape(), MatrixProperties::MINUS_IDENTITY);
  EXPECT_EQ(p6.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_TRUE(p6.isConstant());
  EXPECT_TRUE(p6.isDiagonal());
  EXPECT_FALSE(p6.isIdentity());
  EXPECT_TRUE(p6.isInvertible());
  EXPECT_TRUE(p6.isMinusIdentity());
  EXPECT_TRUE(p6.isMultipleOfIdentity());
  EXPECT_TRUE(p6.isNegativeDefinite());
  EXPECT_TRUE(p6.isNegativeSemidefinite());
  EXPECT_TRUE(p6.isNonZeroIndefinite());
  EXPECT_FALSE(p6.isPositiveDefinite());
  EXPECT_FALSE(p6.isPositiveSemiDefinite());
  EXPECT_TRUE(p6.isSymmetric());
  EXPECT_TRUE(p6.isIndefinite());
  EXPECT_FALSE(p6.isZero());
  EXPECT_TRUE(p6.isTriangular());
  EXPECT_TRUE(p6.isLowerTriangular());
  EXPECT_TRUE(p6.isUpperTriangular());

  MatrixProperties p7(MatrixProperties::ZERO);
  EXPECT_EQ(p7.shape(), MatrixProperties::ZERO);
  EXPECT_EQ(p7.positiveness(), MatrixProperties::INDEFINITE);
  EXPECT_TRUE(p7.isConstant());
  EXPECT_TRUE(p7.isDiagonal());
  EXPECT_FALSE(p7.isIdentity());
  EXPECT_FALSE(p7.isInvertible());
  EXPECT_FALSE(p7.isMinusIdentity());
  EXPECT_TRUE(p7.isMultipleOfIdentity());
  EXPECT_FALSE(p7.isNegativeDefinite());
  EXPECT_TRUE(p7.isNegativeSemidefinite());
  EXPECT_FALSE(p7.isNonZeroIndefinite());
  EXPECT_FALSE(p7.isPositiveDefinite());
  EXPECT_TRUE(p7.isPositiveSemiDefinite());
  EXPECT_TRUE(p7.isSymmetric());
  EXPECT_TRUE(p7.isIndefinite());
  EXPECT_TRUE(p7.isZero());
  EXPECT_TRUE(p7.isTriangular());
  EXPECT_TRUE(p7.isLowerTriangular());
  EXPECT_TRUE(p7.isUpperTriangular());
}

TEST(MatrixPropertiesTest, propertiesDeductions) {  // NOLINT
  MatrixProperties p01(MatrixProperties::GENERAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_EQ(p01.shape(), MatrixProperties::GENERAL);
  EXPECT_EQ(p01.positiveness(), MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_FALSE(p01.isConstant());
  EXPECT_FALSE(p01.isDiagonal());
  EXPECT_FALSE(p01.isIdentity());
  EXPECT_FALSE(p01.isInvertible());
  EXPECT_FALSE(p01.isMinusIdentity());
  EXPECT_FALSE(p01.isMultipleOfIdentity());
  EXPECT_FALSE(p01.isNegativeDefinite());
  EXPECT_FALSE(p01.isNegativeSemidefinite());
  EXPECT_FALSE(p01.isNonZeroIndefinite());
  EXPECT_FALSE(p01.isPositiveDefinite());
  EXPECT_TRUE(p01.isPositiveSemiDefinite());
  EXPECT_TRUE(p01.isSymmetric());
  EXPECT_TRUE(p01.isIndefinite());
  EXPECT_FALSE(p01.isZero());
  EXPECT_FALSE(p01.isTriangular());
  EXPECT_FALSE(p01.isLowerTriangular());
  EXPECT_FALSE(p01.isUpperTriangular());

  MatrixProperties p02(MatrixProperties::GENERAL, MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_EQ(p02.shape(), MatrixProperties::GENERAL);
  EXPECT_EQ(p02.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_FALSE(p02.isConstant());
  EXPECT_FALSE(p02.isDiagonal());
  EXPECT_FALSE(p02.isIdentity());
  EXPECT_TRUE(p02.isInvertible());
  EXPECT_FALSE(p02.isMinusIdentity());
  EXPECT_FALSE(p02.isMultipleOfIdentity());
  EXPECT_FALSE(p02.isNegativeDefinite());
  EXPECT_FALSE(p02.isNegativeSemidefinite());
  EXPECT_TRUE(p02.isNonZeroIndefinite());
  EXPECT_TRUE(p02.isPositiveDefinite());
  EXPECT_TRUE(p02.isPositiveSemiDefinite());
  EXPECT_TRUE(p02.isSymmetric());
  EXPECT_TRUE(p02.isIndefinite());
  EXPECT_FALSE(p02.isZero());
  EXPECT_FALSE(p02.isTriangular());
  EXPECT_FALSE(p02.isLowerTriangular());
  EXPECT_FALSE(p02.isUpperTriangular());

  MatrixProperties p03(MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_EQ(p03.shape(), MatrixProperties::GENERAL);
  EXPECT_EQ(p03.positiveness(), MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_FALSE(p03.isConstant());
  EXPECT_FALSE(p03.isDiagonal());
  EXPECT_FALSE(p03.isIdentity());
  EXPECT_FALSE(p03.isInvertible());
  EXPECT_FALSE(p03.isMinusIdentity());
  EXPECT_FALSE(p03.isMultipleOfIdentity());
  EXPECT_FALSE(p03.isNegativeDefinite());
  EXPECT_TRUE(p03.isNegativeSemidefinite());
  EXPECT_FALSE(p03.isNonZeroIndefinite());
  EXPECT_FALSE(p03.isPositiveDefinite());
  EXPECT_FALSE(p03.isPositiveSemiDefinite());
  EXPECT_TRUE(p03.isSymmetric());
  EXPECT_TRUE(p03.isIndefinite());
  EXPECT_FALSE(p03.isZero());
  EXPECT_FALSE(p03.isTriangular());
  EXPECT_FALSE(p03.isLowerTriangular());
  EXPECT_FALSE(p03.isUpperTriangular());

  MatrixProperties p04(MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_EQ(p04.shape(), MatrixProperties::GENERAL);
  EXPECT_EQ(p04.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_FALSE(p04.isConstant());
  EXPECT_FALSE(p04.isDiagonal());
  EXPECT_FALSE(p04.isIdentity());
  EXPECT_TRUE(p04.isInvertible());
  EXPECT_FALSE(p04.isMinusIdentity());
  EXPECT_FALSE(p04.isMultipleOfIdentity());
  EXPECT_TRUE(p04.isNegativeDefinite());
  EXPECT_TRUE(p04.isNegativeSemidefinite());
  EXPECT_TRUE(p04.isNonZeroIndefinite());
  EXPECT_FALSE(p04.isPositiveDefinite());
  EXPECT_FALSE(p04.isPositiveSemiDefinite());
  EXPECT_TRUE(p04.isSymmetric());
  EXPECT_TRUE(p04.isIndefinite());
  EXPECT_FALSE(p04.isZero());
  EXPECT_FALSE(p04.isTriangular());
  EXPECT_FALSE(p04.isLowerTriangular());
  EXPECT_FALSE(p04.isUpperTriangular());

  MatrixProperties p05(MatrixProperties::GENERAL, MatrixProperties::INDEFINITE);
  EXPECT_EQ(p05.shape(), MatrixProperties::GENERAL);
  EXPECT_EQ(p05.positiveness(), MatrixProperties::INDEFINITE);
  EXPECT_FALSE(p05.isConstant());
  EXPECT_FALSE(p05.isDiagonal());
  EXPECT_FALSE(p05.isIdentity());
  EXPECT_FALSE(p05.isInvertible());
  EXPECT_FALSE(p05.isMinusIdentity());
  EXPECT_FALSE(p05.isMultipleOfIdentity());
  EXPECT_FALSE(p05.isNegativeDefinite());
  EXPECT_FALSE(p05.isNegativeSemidefinite());
  EXPECT_FALSE(p05.isNonZeroIndefinite());
  EXPECT_FALSE(p05.isPositiveDefinite());
  EXPECT_FALSE(p05.isPositiveSemiDefinite());
  EXPECT_TRUE(p05.isSymmetric());
  EXPECT_TRUE(p05.isIndefinite());
  EXPECT_FALSE(p05.isZero());
  EXPECT_FALSE(p05.isTriangular());
  EXPECT_FALSE(p05.isLowerTriangular());
  EXPECT_FALSE(p05.isUpperTriangular());

  MatrixProperties p06(MatrixProperties::GENERAL, MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_EQ(p06.shape(), MatrixProperties::GENERAL);
  EXPECT_EQ(p06.positiveness(), MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_FALSE(p06.isConstant());
  EXPECT_FALSE(p06.isDiagonal());
  EXPECT_FALSE(p06.isIdentity());
  EXPECT_TRUE(p06.isInvertible());
  EXPECT_FALSE(p06.isMinusIdentity());
  EXPECT_FALSE(p06.isMultipleOfIdentity());
  EXPECT_FALSE(p06.isNegativeDefinite());
  EXPECT_FALSE(p06.isNegativeSemidefinite());
  EXPECT_TRUE(p06.isNonZeroIndefinite());
  EXPECT_FALSE(p06.isPositiveDefinite());
  EXPECT_FALSE(p06.isPositiveSemiDefinite());
  EXPECT_TRUE(p06.isSymmetric());
  EXPECT_TRUE(p06.isIndefinite());
  EXPECT_FALSE(p06.isZero());
  EXPECT_FALSE(p06.isTriangular());
  EXPECT_FALSE(p06.isLowerTriangular());
  EXPECT_FALSE(p06.isUpperTriangular());

  MatrixProperties p11(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_EQ(p11.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p11.positiveness(), MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_FALSE(p11.isConstant());
  EXPECT_TRUE(p11.isDiagonal());
  EXPECT_FALSE(p11.isIdentity());
  EXPECT_FALSE(p11.isInvertible());
  EXPECT_FALSE(p11.isMinusIdentity());
  EXPECT_FALSE(p11.isMultipleOfIdentity());
  EXPECT_FALSE(p11.isNegativeDefinite());
  EXPECT_FALSE(p11.isNegativeSemidefinite());
  EXPECT_FALSE(p11.isNonZeroIndefinite());
  EXPECT_FALSE(p11.isPositiveDefinite());
  EXPECT_TRUE(p11.isPositiveSemiDefinite());
  EXPECT_TRUE(p11.isSymmetric());
  EXPECT_TRUE(p11.isIndefinite());
  EXPECT_FALSE(p11.isZero());
  EXPECT_TRUE(p11.isTriangular());
  EXPECT_TRUE(p11.isLowerTriangular());
  EXPECT_TRUE(p11.isUpperTriangular());

  MatrixProperties p12(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_EQ(p12.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p12.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_FALSE(p12.isConstant());
  EXPECT_TRUE(p12.isDiagonal());
  EXPECT_FALSE(p12.isIdentity());
  EXPECT_TRUE(p12.isInvertible());
  EXPECT_FALSE(p12.isMinusIdentity());
  EXPECT_FALSE(p12.isMultipleOfIdentity());
  EXPECT_FALSE(p12.isNegativeDefinite());
  EXPECT_FALSE(p12.isNegativeSemidefinite());
  EXPECT_TRUE(p12.isNonZeroIndefinite());
  EXPECT_TRUE(p12.isPositiveDefinite());
  EXPECT_TRUE(p12.isPositiveSemiDefinite());
  EXPECT_TRUE(p12.isSymmetric());
  EXPECT_TRUE(p12.isIndefinite());
  EXPECT_FALSE(p12.isZero());
  EXPECT_TRUE(p12.isTriangular());
  EXPECT_TRUE(p12.isLowerTriangular());
  EXPECT_TRUE(p12.isUpperTriangular());

  MatrixProperties p13(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_EQ(p13.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p13.positiveness(), MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_FALSE(p13.isConstant());
  EXPECT_TRUE(p13.isDiagonal());
  EXPECT_FALSE(p13.isIdentity());
  EXPECT_FALSE(p13.isInvertible());
  EXPECT_FALSE(p13.isMinusIdentity());
  EXPECT_FALSE(p13.isMultipleOfIdentity());
  EXPECT_FALSE(p13.isNegativeDefinite());
  EXPECT_TRUE(p13.isNegativeSemidefinite());
  EXPECT_FALSE(p13.isNonZeroIndefinite());
  EXPECT_FALSE(p13.isPositiveDefinite());
  EXPECT_FALSE(p13.isPositiveSemiDefinite());
  EXPECT_TRUE(p13.isSymmetric());
  EXPECT_TRUE(p13.isIndefinite());
  EXPECT_FALSE(p13.isZero());
  EXPECT_TRUE(p13.isTriangular());
  EXPECT_TRUE(p13.isLowerTriangular());
  EXPECT_TRUE(p13.isUpperTriangular());

  MatrixProperties p14(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_EQ(p14.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p14.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_FALSE(p14.isConstant());
  EXPECT_TRUE(p14.isDiagonal());
  EXPECT_FALSE(p14.isIdentity());
  EXPECT_TRUE(p14.isInvertible());
  EXPECT_FALSE(p14.isMinusIdentity());
  EXPECT_FALSE(p14.isMultipleOfIdentity());
  EXPECT_TRUE(p14.isNegativeDefinite());
  EXPECT_TRUE(p14.isNegativeSemidefinite());
  EXPECT_TRUE(p14.isNonZeroIndefinite());
  EXPECT_FALSE(p14.isPositiveDefinite());
  EXPECT_FALSE(p14.isPositiveSemiDefinite());
  EXPECT_TRUE(p14.isSymmetric());
  EXPECT_TRUE(p14.isIndefinite());
  EXPECT_FALSE(p14.isZero());
  EXPECT_TRUE(p14.isTriangular());
  EXPECT_TRUE(p14.isLowerTriangular());
  EXPECT_TRUE(p14.isUpperTriangular());

  MatrixProperties p15(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::INDEFINITE);
  EXPECT_EQ(p15.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p15.positiveness(), MatrixProperties::INDEFINITE);
  EXPECT_FALSE(p15.isConstant());
  EXPECT_TRUE(p15.isDiagonal());
  EXPECT_FALSE(p15.isIdentity());
  EXPECT_FALSE(p15.isInvertible());
  EXPECT_FALSE(p15.isMinusIdentity());
  EXPECT_FALSE(p15.isMultipleOfIdentity());
  EXPECT_FALSE(p15.isNegativeDefinite());
  EXPECT_FALSE(p15.isNegativeSemidefinite());
  EXPECT_FALSE(p15.isNonZeroIndefinite());
  EXPECT_FALSE(p15.isPositiveDefinite());
  EXPECT_FALSE(p15.isPositiveSemiDefinite());
  EXPECT_TRUE(p15.isSymmetric());
  EXPECT_TRUE(p15.isIndefinite());
  EXPECT_FALSE(p15.isZero());
  EXPECT_TRUE(p15.isTriangular());
  EXPECT_TRUE(p15.isLowerTriangular());
  EXPECT_TRUE(p15.isUpperTriangular());

  MatrixProperties p16(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_EQ(p16.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p16.positiveness(), MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_FALSE(p16.isConstant());
  EXPECT_TRUE(p16.isDiagonal());
  EXPECT_FALSE(p16.isIdentity());
  EXPECT_TRUE(p16.isInvertible());
  EXPECT_FALSE(p16.isMinusIdentity());
  EXPECT_FALSE(p16.isMultipleOfIdentity());
  EXPECT_FALSE(p16.isNegativeDefinite());
  EXPECT_FALSE(p16.isNegativeSemidefinite());
  EXPECT_TRUE(p16.isNonZeroIndefinite());
  EXPECT_FALSE(p16.isPositiveDefinite());
  EXPECT_FALSE(p16.isPositiveSemiDefinite());
  EXPECT_TRUE(p16.isSymmetric());
  EXPECT_TRUE(p16.isIndefinite());
  EXPECT_FALSE(p16.isZero());
  EXPECT_TRUE(p16.isTriangular());
  EXPECT_TRUE(p16.isLowerTriangular());
  EXPECT_TRUE(p16.isUpperTriangular());


  MatrixProperties p21(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_EQ(p21.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p21.positiveness(), MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_FALSE(p21.isConstant());
  EXPECT_TRUE(p21.isDiagonal());
  EXPECT_FALSE(p21.isIdentity());
  EXPECT_FALSE(p21.isInvertible());
  EXPECT_FALSE(p21.isMinusIdentity());
  EXPECT_FALSE(p21.isMultipleOfIdentity());
  EXPECT_FALSE(p21.isNegativeDefinite());
  EXPECT_FALSE(p21.isNegativeSemidefinite());
  EXPECT_FALSE(p21.isNonZeroIndefinite());
  EXPECT_FALSE(p21.isPositiveDefinite());
  EXPECT_TRUE(p21.isPositiveSemiDefinite());
  EXPECT_TRUE(p21.isSymmetric());
  EXPECT_TRUE(p21.isIndefinite());
  EXPECT_FALSE(p21.isZero());
  EXPECT_TRUE(p21.isTriangular());
  EXPECT_TRUE(p21.isLowerTriangular());
  EXPECT_TRUE(p21.isUpperTriangular());

  MatrixProperties p22(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_EQ(p22.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p22.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_FALSE(p22.isConstant());
  EXPECT_TRUE(p22.isDiagonal());
  EXPECT_FALSE(p22.isIdentity());
  EXPECT_TRUE(p22.isInvertible());
  EXPECT_FALSE(p22.isMinusIdentity());
  EXPECT_FALSE(p22.isMultipleOfIdentity());
  EXPECT_FALSE(p22.isNegativeDefinite());
  EXPECT_FALSE(p22.isNegativeSemidefinite());
  EXPECT_TRUE(p22.isNonZeroIndefinite());
  EXPECT_TRUE(p22.isPositiveDefinite());
  EXPECT_TRUE(p22.isPositiveSemiDefinite());
  EXPECT_TRUE(p22.isSymmetric());
  EXPECT_TRUE(p22.isIndefinite());
  EXPECT_FALSE(p22.isZero());
  EXPECT_TRUE(p22.isTriangular());
  EXPECT_TRUE(p22.isLowerTriangular());
  EXPECT_TRUE(p22.isUpperTriangular());

  MatrixProperties p23(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_EQ(p23.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p23.positiveness(), MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_FALSE(p23.isConstant());
  EXPECT_TRUE(p23.isDiagonal());
  EXPECT_FALSE(p23.isIdentity());
  EXPECT_FALSE(p23.isInvertible());
  EXPECT_FALSE(p23.isMinusIdentity());
  EXPECT_FALSE(p23.isMultipleOfIdentity());
  EXPECT_FALSE(p23.isNegativeDefinite());
  EXPECT_TRUE(p23.isNegativeSemidefinite());
  EXPECT_FALSE(p23.isNonZeroIndefinite());
  EXPECT_FALSE(p23.isPositiveDefinite());
  EXPECT_FALSE(p23.isPositiveSemiDefinite());
  EXPECT_TRUE(p23.isSymmetric());
  EXPECT_TRUE(p23.isIndefinite());
  EXPECT_FALSE(p23.isZero());
  EXPECT_TRUE(p23.isTriangular());
  EXPECT_TRUE(p23.isLowerTriangular());
  EXPECT_TRUE(p23.isUpperTriangular());

  MatrixProperties p24(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_EQ(p24.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p24.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_FALSE(p24.isConstant());
  EXPECT_TRUE(p24.isDiagonal());
  EXPECT_FALSE(p24.isIdentity());
  EXPECT_TRUE(p24.isInvertible());
  EXPECT_FALSE(p24.isMinusIdentity());
  EXPECT_FALSE(p24.isMultipleOfIdentity());
  EXPECT_TRUE(p24.isNegativeDefinite());
  EXPECT_TRUE(p24.isNegativeSemidefinite());
  EXPECT_TRUE(p24.isNonZeroIndefinite());
  EXPECT_FALSE(p24.isPositiveDefinite());
  EXPECT_FALSE(p24.isPositiveSemiDefinite());
  EXPECT_TRUE(p24.isSymmetric());
  EXPECT_TRUE(p24.isIndefinite());
  EXPECT_FALSE(p24.isZero());
  EXPECT_TRUE(p24.isTriangular());
  EXPECT_TRUE(p24.isLowerTriangular());
  EXPECT_TRUE(p24.isUpperTriangular());

  MatrixProperties p25(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::INDEFINITE);
  EXPECT_EQ(p25.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p25.positiveness(), MatrixProperties::INDEFINITE);
  EXPECT_FALSE(p25.isConstant());
  EXPECT_TRUE(p25.isDiagonal());
  EXPECT_FALSE(p25.isIdentity());
  EXPECT_FALSE(p25.isInvertible());
  EXPECT_FALSE(p25.isMinusIdentity());
  EXPECT_FALSE(p25.isMultipleOfIdentity());
  EXPECT_FALSE(p25.isNegativeDefinite());
  EXPECT_FALSE(p25.isNegativeSemidefinite());
  EXPECT_FALSE(p25.isNonZeroIndefinite());
  EXPECT_FALSE(p25.isPositiveDefinite());
  EXPECT_FALSE(p25.isPositiveSemiDefinite());
  EXPECT_TRUE(p25.isSymmetric());
  EXPECT_TRUE(p25.isIndefinite());
  EXPECT_FALSE(p25.isZero());
  EXPECT_TRUE(p25.isTriangular());
  EXPECT_TRUE(p25.isLowerTriangular());
  EXPECT_TRUE(p25.isUpperTriangular());

  MatrixProperties p26(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_EQ(p26.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p26.positiveness(), MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_FALSE(p26.isConstant());
  EXPECT_TRUE(p26.isDiagonal());
  EXPECT_FALSE(p26.isIdentity());
  EXPECT_TRUE(p26.isInvertible());
  EXPECT_FALSE(p26.isMinusIdentity());
  EXPECT_FALSE(p26.isMultipleOfIdentity());
  EXPECT_FALSE(p26.isNegativeDefinite());
  EXPECT_FALSE(p26.isNegativeSemidefinite());
  EXPECT_TRUE(p26.isNonZeroIndefinite());
  EXPECT_FALSE(p26.isPositiveDefinite());
  EXPECT_FALSE(p26.isPositiveSemiDefinite());
  EXPECT_TRUE(p26.isSymmetric());
  EXPECT_TRUE(p26.isIndefinite());
  EXPECT_FALSE(p26.isZero());
  EXPECT_TRUE(p26.isTriangular());
  EXPECT_TRUE(p26.isLowerTriangular());
  EXPECT_TRUE(p26.isUpperTriangular());


  MatrixProperties p31(MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_EQ(p31.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p31.positiveness(), MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_FALSE(p31.isConstant());
  EXPECT_TRUE(p31.isDiagonal());
  EXPECT_FALSE(p31.isIdentity());
  EXPECT_FALSE(p31.isInvertible());
  EXPECT_FALSE(p31.isMinusIdentity());
  EXPECT_FALSE(p31.isMultipleOfIdentity());
  EXPECT_FALSE(p31.isNegativeDefinite());
  EXPECT_FALSE(p31.isNegativeSemidefinite());
  EXPECT_FALSE(p31.isNonZeroIndefinite());
  EXPECT_FALSE(p31.isPositiveDefinite());
  EXPECT_TRUE(p31.isPositiveSemiDefinite());
  EXPECT_TRUE(p31.isSymmetric());
  EXPECT_TRUE(p31.isIndefinite());
  EXPECT_FALSE(p31.isZero());
  EXPECT_TRUE(p31.isTriangular());
  EXPECT_TRUE(p31.isLowerTriangular());
  EXPECT_TRUE(p31.isUpperTriangular());

  MatrixProperties p32(MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_EQ(p32.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p32.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_FALSE(p32.isConstant());
  EXPECT_TRUE(p32.isDiagonal());
  EXPECT_FALSE(p32.isIdentity());
  EXPECT_TRUE(p32.isInvertible());
  EXPECT_FALSE(p32.isMinusIdentity());
  EXPECT_FALSE(p32.isMultipleOfIdentity());
  EXPECT_FALSE(p32.isNegativeDefinite());
  EXPECT_FALSE(p32.isNegativeSemidefinite());
  EXPECT_TRUE(p32.isNonZeroIndefinite());
  EXPECT_TRUE(p32.isPositiveDefinite());
  EXPECT_TRUE(p32.isPositiveSemiDefinite());
  EXPECT_TRUE(p32.isSymmetric());
  EXPECT_TRUE(p32.isIndefinite());
  EXPECT_FALSE(p32.isZero());
  EXPECT_TRUE(p32.isTriangular());
  EXPECT_TRUE(p32.isLowerTriangular());
  EXPECT_TRUE(p32.isUpperTriangular());

  MatrixProperties p33(MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_EQ(p33.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p33.positiveness(), MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_FALSE(p33.isConstant());
  EXPECT_TRUE(p33.isDiagonal());
  EXPECT_FALSE(p33.isIdentity());
  EXPECT_FALSE(p33.isInvertible());
  EXPECT_FALSE(p33.isMinusIdentity());
  EXPECT_FALSE(p33.isMultipleOfIdentity());
  EXPECT_FALSE(p33.isNegativeDefinite());
  EXPECT_TRUE(p33.isNegativeSemidefinite());
  EXPECT_FALSE(p33.isNonZeroIndefinite());
  EXPECT_FALSE(p33.isPositiveDefinite());
  EXPECT_FALSE(p33.isPositiveSemiDefinite());
  EXPECT_TRUE(p33.isSymmetric());
  EXPECT_TRUE(p33.isIndefinite());
  EXPECT_FALSE(p33.isZero());
  EXPECT_TRUE(p33.isTriangular());
  EXPECT_TRUE(p33.isLowerTriangular());
  EXPECT_TRUE(p33.isUpperTriangular());

  MatrixProperties p34(MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_EQ(p34.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p34.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_FALSE(p34.isConstant());
  EXPECT_TRUE(p34.isDiagonal());
  EXPECT_FALSE(p34.isIdentity());
  EXPECT_TRUE(p34.isInvertible());
  EXPECT_FALSE(p34.isMinusIdentity());
  EXPECT_FALSE(p34.isMultipleOfIdentity());
  EXPECT_TRUE(p34.isNegativeDefinite());
  EXPECT_TRUE(p34.isNegativeSemidefinite());
  EXPECT_TRUE(p34.isNonZeroIndefinite());
  EXPECT_FALSE(p34.isPositiveDefinite());
  EXPECT_FALSE(p34.isPositiveSemiDefinite());
  EXPECT_TRUE(p34.isSymmetric());
  EXPECT_TRUE(p34.isIndefinite());
  EXPECT_FALSE(p34.isZero());
  EXPECT_TRUE(p34.isTriangular());
  EXPECT_TRUE(p34.isLowerTriangular());
  EXPECT_TRUE(p34.isUpperTriangular());

  MatrixProperties p35(MatrixProperties::DIAGONAL, MatrixProperties::INDEFINITE);
  EXPECT_EQ(p35.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p35.positiveness(), MatrixProperties::INDEFINITE);
  EXPECT_FALSE(p35.isConstant());
  EXPECT_TRUE(p35.isDiagonal());
  EXPECT_FALSE(p35.isIdentity());
  EXPECT_FALSE(p35.isInvertible());
  EXPECT_FALSE(p35.isMinusIdentity());
  EXPECT_FALSE(p35.isMultipleOfIdentity());
  EXPECT_FALSE(p35.isNegativeDefinite());
  EXPECT_FALSE(p35.isNegativeSemidefinite());
  EXPECT_FALSE(p35.isNonZeroIndefinite());
  EXPECT_FALSE(p35.isPositiveDefinite());
  EXPECT_FALSE(p35.isPositiveSemiDefinite());
  EXPECT_TRUE(p35.isSymmetric());
  EXPECT_TRUE(p35.isIndefinite());
  EXPECT_FALSE(p35.isZero());
  EXPECT_TRUE(p35.isTriangular());
  EXPECT_TRUE(p35.isLowerTriangular());
  EXPECT_TRUE(p35.isUpperTriangular());

  MatrixProperties p36(MatrixProperties::DIAGONAL, MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_EQ(p36.shape(), MatrixProperties::DIAGONAL);
  EXPECT_EQ(p36.positiveness(), MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_FALSE(p36.isConstant());
  EXPECT_TRUE(p36.isDiagonal());
  EXPECT_FALSE(p36.isIdentity());
  EXPECT_TRUE(p36.isInvertible());
  EXPECT_FALSE(p36.isMinusIdentity());
  EXPECT_FALSE(p36.isMultipleOfIdentity());
  EXPECT_FALSE(p36.isNegativeDefinite());
  EXPECT_FALSE(p36.isNegativeSemidefinite());
  EXPECT_TRUE(p36.isNonZeroIndefinite());
  EXPECT_FALSE(p36.isPositiveDefinite());
  EXPECT_FALSE(p36.isPositiveSemiDefinite());
  EXPECT_TRUE(p36.isSymmetric());
  EXPECT_TRUE(p36.isIndefinite());
  EXPECT_FALSE(p36.isZero());
  EXPECT_TRUE(p36.isTriangular());
  EXPECT_TRUE(p36.isLowerTriangular());
  EXPECT_TRUE(p36.isUpperTriangular());


  MatrixProperties p41(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_EQ(p41.shape(), MatrixProperties::MULTIPLE_OF_IDENTITY);
  EXPECT_EQ(p41.positiveness(), MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_FALSE(p41.isConstant());
  EXPECT_TRUE(p41.isDiagonal());
  EXPECT_FALSE(p41.isIdentity());
  EXPECT_FALSE(p41.isInvertible());
  EXPECT_FALSE(p41.isMinusIdentity());
  EXPECT_TRUE(p41.isMultipleOfIdentity());
  EXPECT_FALSE(p41.isNegativeDefinite());
  EXPECT_FALSE(p41.isNegativeSemidefinite());
  EXPECT_FALSE(p41.isNonZeroIndefinite());
  EXPECT_FALSE(p41.isPositiveDefinite());
  EXPECT_TRUE(p41.isPositiveSemiDefinite());
  EXPECT_TRUE(p41.isSymmetric());
  EXPECT_TRUE(p41.isIndefinite());
  EXPECT_FALSE(p41.isZero());
  EXPECT_TRUE(p41.isTriangular());
  EXPECT_TRUE(p41.isLowerTriangular());
  EXPECT_TRUE(p41.isUpperTriangular());

  MatrixProperties p42(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_EQ(p42.shape(), MatrixProperties::MULTIPLE_OF_IDENTITY);
  EXPECT_EQ(p42.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_FALSE(p42.isConstant());
  EXPECT_TRUE(p42.isDiagonal());
  EXPECT_FALSE(p42.isIdentity());
  EXPECT_TRUE(p42.isInvertible());
  EXPECT_FALSE(p42.isMinusIdentity());
  EXPECT_TRUE(p42.isMultipleOfIdentity());
  EXPECT_FALSE(p42.isNegativeDefinite());
  EXPECT_FALSE(p42.isNegativeSemidefinite());
  EXPECT_TRUE(p42.isNonZeroIndefinite());
  EXPECT_TRUE(p42.isPositiveDefinite());
  EXPECT_TRUE(p42.isPositiveSemiDefinite());
  EXPECT_TRUE(p42.isSymmetric());
  EXPECT_TRUE(p42.isIndefinite());
  EXPECT_FALSE(p42.isZero());
  EXPECT_TRUE(p42.isTriangular());
  EXPECT_TRUE(p42.isLowerTriangular());
  EXPECT_TRUE(p42.isUpperTriangular());

  MatrixProperties p43(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_EQ(p43.shape(), MatrixProperties::MULTIPLE_OF_IDENTITY);
  EXPECT_EQ(p43.positiveness(), MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_FALSE(p43.isConstant());
  EXPECT_TRUE(p43.isDiagonal());
  EXPECT_FALSE(p43.isIdentity());
  EXPECT_FALSE(p43.isInvertible());
  EXPECT_FALSE(p43.isMinusIdentity());
  EXPECT_TRUE(p43.isMultipleOfIdentity());
  EXPECT_FALSE(p43.isNegativeDefinite());
  EXPECT_TRUE(p43.isNegativeSemidefinite());
  EXPECT_FALSE(p43.isNonZeroIndefinite());
  EXPECT_FALSE(p43.isPositiveDefinite());
  EXPECT_FALSE(p43.isPositiveSemiDefinite());
  EXPECT_TRUE(p43.isSymmetric());
  EXPECT_TRUE(p43.isIndefinite());
  EXPECT_FALSE(p43.isZero());
  EXPECT_TRUE(p43.isTriangular());
  EXPECT_TRUE(p43.isLowerTriangular());
  EXPECT_TRUE(p43.isUpperTriangular());

  MatrixProperties p44(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_EQ(p44.shape(), MatrixProperties::MULTIPLE_OF_IDENTITY);
  EXPECT_EQ(p44.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_FALSE(p44.isConstant());
  EXPECT_TRUE(p44.isDiagonal());
  EXPECT_FALSE(p44.isIdentity());
  EXPECT_TRUE(p44.isInvertible());
  EXPECT_FALSE(p44.isMinusIdentity());
  EXPECT_TRUE(p44.isMultipleOfIdentity());
  EXPECT_TRUE(p44.isNegativeDefinite());
  EXPECT_TRUE(p44.isNegativeSemidefinite());
  EXPECT_TRUE(p44.isNonZeroIndefinite());
  EXPECT_FALSE(p44.isPositiveDefinite());
  EXPECT_FALSE(p44.isPositiveSemiDefinite());
  EXPECT_TRUE(p44.isSymmetric());
  EXPECT_TRUE(p44.isIndefinite());
  EXPECT_FALSE(p44.isZero());
  EXPECT_TRUE(p44.isTriangular());
  EXPECT_TRUE(p44.isLowerTriangular());
  EXPECT_TRUE(p44.isUpperTriangular());

  MatrixProperties p45(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::INDEFINITE);
  EXPECT_EQ(p45.shape(), MatrixProperties::MULTIPLE_OF_IDENTITY);
  EXPECT_EQ(p45.positiveness(), MatrixProperties::INDEFINITE);
  EXPECT_FALSE(p45.isConstant());
  EXPECT_TRUE(p45.isDiagonal());
  EXPECT_FALSE(p45.isIdentity());
  EXPECT_FALSE(p45.isInvertible());
  EXPECT_FALSE(p45.isMinusIdentity());
  EXPECT_TRUE(p45.isMultipleOfIdentity());
  EXPECT_FALSE(p45.isNegativeDefinite());
  EXPECT_FALSE(p45.isNegativeSemidefinite());
  EXPECT_FALSE(p45.isNonZeroIndefinite());
  EXPECT_FALSE(p45.isPositiveDefinite());
  EXPECT_FALSE(p45.isPositiveSemiDefinite());
  EXPECT_TRUE(p45.isSymmetric());
  EXPECT_TRUE(p45.isIndefinite());
  EXPECT_FALSE(p45.isZero());
  EXPECT_TRUE(p45.isTriangular());
  EXPECT_TRUE(p45.isLowerTriangular());
  EXPECT_TRUE(p45.isUpperTriangular());

  MatrixProperties p46(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_EQ(p46.shape(), MatrixProperties::MULTIPLE_OF_IDENTITY);
  EXPECT_EQ(p46.positiveness(), MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_FALSE(p46.isConstant());
  EXPECT_TRUE(p46.isDiagonal());
  EXPECT_FALSE(p46.isIdentity());
  EXPECT_TRUE(p46.isInvertible());
  EXPECT_FALSE(p46.isMinusIdentity());
  EXPECT_TRUE(p46.isMultipleOfIdentity());
  EXPECT_FALSE(p46.isNegativeDefinite());
  EXPECT_FALSE(p46.isNegativeSemidefinite());
  EXPECT_TRUE(p46.isNonZeroIndefinite());
  EXPECT_FALSE(p46.isPositiveDefinite());
  EXPECT_FALSE(p46.isPositiveSemiDefinite());
  EXPECT_TRUE(p46.isSymmetric());
  EXPECT_TRUE(p46.isIndefinite());
  EXPECT_FALSE(p46.isZero());
  EXPECT_TRUE(p46.isTriangular());
  EXPECT_TRUE(p46.isLowerTriangular());
  EXPECT_TRUE(p46.isUpperTriangular());

  MatrixProperties p51(MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_EQ(p51.shape(), MatrixProperties::IDENTITY);
  EXPECT_EQ(p51.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_TRUE(p51.isConstant());
  EXPECT_TRUE(p51.isDiagonal());
  EXPECT_TRUE(p51.isIdentity());
  EXPECT_TRUE(p51.isInvertible());
  EXPECT_FALSE(p51.isMinusIdentity());
  EXPECT_TRUE(p51.isMultipleOfIdentity());
  EXPECT_FALSE(p51.isNegativeDefinite());
  EXPECT_FALSE(p51.isNegativeSemidefinite());
  EXPECT_TRUE(p51.isNonZeroIndefinite());
  EXPECT_TRUE(p51.isPositiveDefinite());
  EXPECT_TRUE(p51.isPositiveSemiDefinite());
  EXPECT_TRUE(p51.isSymmetric());
  EXPECT_TRUE(p51.isIndefinite());
  EXPECT_FALSE(p51.isZero());
  EXPECT_TRUE(p51.isTriangular());
  EXPECT_TRUE(p51.isLowerTriangular());
  EXPECT_TRUE(p51.isUpperTriangular());

  MatrixProperties p52(MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_EQ(p52.shape(), MatrixProperties::IDENTITY);
  EXPECT_EQ(p52.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_TRUE(p52.isConstant());
  EXPECT_TRUE(p52.isDiagonal());
  EXPECT_TRUE(p52.isIdentity());
  EXPECT_TRUE(p52.isInvertible());
  EXPECT_FALSE(p52.isMinusIdentity());
  EXPECT_TRUE(p52.isMultipleOfIdentity());
  EXPECT_FALSE(p52.isNegativeDefinite());
  EXPECT_FALSE(p52.isNegativeSemidefinite());
  EXPECT_TRUE(p52.isNonZeroIndefinite());
  EXPECT_TRUE(p52.isPositiveDefinite());
  EXPECT_TRUE(p52.isPositiveSemiDefinite());
  EXPECT_TRUE(p52.isSymmetric());
  EXPECT_TRUE(p52.isIndefinite());
  EXPECT_FALSE(p52.isZero());
  EXPECT_TRUE(p52.isTriangular());
  EXPECT_TRUE(p52.isLowerTriangular());
  EXPECT_TRUE(p52.isUpperTriangular());

  EXPECT_THROW(
    MatrixProperties p53(MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE)
    , std::runtime_error);
  EXPECT_THROW(
    MatrixProperties p54(MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_DEFINITE)
    , std::runtime_error);

  MatrixProperties p55(MatrixProperties::IDENTITY, MatrixProperties::INDEFINITE);
  EXPECT_EQ(p55.shape(), MatrixProperties::IDENTITY);
  EXPECT_EQ(p55.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_TRUE(p55.isConstant());
  EXPECT_TRUE(p55.isDiagonal());
  EXPECT_TRUE(p55.isIdentity());
  EXPECT_TRUE(p55.isInvertible());
  EXPECT_FALSE(p55.isMinusIdentity());
  EXPECT_TRUE(p55.isMultipleOfIdentity());
  EXPECT_FALSE(p55.isNegativeDefinite());
  EXPECT_FALSE(p55.isNegativeSemidefinite());
  EXPECT_TRUE(p55.isNonZeroIndefinite());
  EXPECT_TRUE(p55.isPositiveDefinite());
  EXPECT_TRUE(p55.isPositiveSemiDefinite());
  EXPECT_TRUE(p55.isSymmetric());
  EXPECT_TRUE(p55.isIndefinite());
  EXPECT_FALSE(p55.isZero());
  EXPECT_TRUE(p55.isTriangular());
  EXPECT_TRUE(p55.isLowerTriangular());
  EXPECT_TRUE(p55.isUpperTriangular());

  MatrixProperties p56(MatrixProperties::IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_EQ(p56.shape(), MatrixProperties::IDENTITY);
  EXPECT_EQ(p56.positiveness(), MatrixProperties::POSITIVE_DEFINITE);
  EXPECT_TRUE(p56.isConstant());
  EXPECT_TRUE(p56.isDiagonal());
  EXPECT_TRUE(p56.isIdentity());
  EXPECT_TRUE(p56.isInvertible());
  EXPECT_FALSE(p56.isMinusIdentity());
  EXPECT_TRUE(p56.isMultipleOfIdentity());
  EXPECT_FALSE(p56.isNegativeDefinite());
  EXPECT_FALSE(p56.isNegativeSemidefinite());
  EXPECT_TRUE(p56.isNonZeroIndefinite());
  EXPECT_TRUE(p56.isPositiveDefinite());
  EXPECT_TRUE(p56.isPositiveSemiDefinite());
  EXPECT_TRUE(p56.isSymmetric());
  EXPECT_TRUE(p56.isIndefinite());
  EXPECT_FALSE(p56.isZero());
  EXPECT_TRUE(p56.isTriangular());
  EXPECT_TRUE(p56.isLowerTriangular());
  EXPECT_TRUE(p56.isUpperTriangular());

  EXPECT_THROW(
    MatrixProperties p61(MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE)
    , std::runtime_error);
  EXPECT_THROW(
    MatrixProperties p62(MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_DEFINITE)
    , std::runtime_error);


  MatrixProperties p63(MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_EQ(p63.shape(), MatrixProperties::MINUS_IDENTITY);
  EXPECT_EQ(p63.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_TRUE(p63.isConstant());
  EXPECT_TRUE(p63.isDiagonal());
  EXPECT_FALSE(p63.isIdentity());
  EXPECT_TRUE(p63.isInvertible());
  EXPECT_TRUE(p63.isMinusIdentity());
  EXPECT_TRUE(p63.isMultipleOfIdentity());
  EXPECT_TRUE(p63.isNegativeDefinite());
  EXPECT_TRUE(p63.isNegativeSemidefinite());
  EXPECT_TRUE(p63.isNonZeroIndefinite());
  EXPECT_FALSE(p63.isPositiveDefinite());
  EXPECT_FALSE(p63.isPositiveSemiDefinite());
  EXPECT_TRUE(p63.isSymmetric());
  EXPECT_TRUE(p63.isIndefinite());
  EXPECT_FALSE(p63.isZero());
  EXPECT_TRUE(p63.isTriangular());
  EXPECT_TRUE(p63.isLowerTriangular());
  EXPECT_TRUE(p63.isUpperTriangular());

  MatrixProperties p64(MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_EQ(p64.shape(), MatrixProperties::MINUS_IDENTITY);
  EXPECT_EQ(p64.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_TRUE(p64.isConstant());
  EXPECT_TRUE(p64.isDiagonal());
  EXPECT_FALSE(p64.isIdentity());
  EXPECT_TRUE(p64.isInvertible());
  EXPECT_TRUE(p64.isMinusIdentity());
  EXPECT_TRUE(p64.isMultipleOfIdentity());
  EXPECT_TRUE(p64.isNegativeDefinite());
  EXPECT_TRUE(p64.isNegativeSemidefinite());
  EXPECT_TRUE(p64.isNonZeroIndefinite());
  EXPECT_FALSE(p64.isPositiveDefinite());
  EXPECT_FALSE(p64.isPositiveSemiDefinite());
  EXPECT_TRUE(p64.isSymmetric());
  EXPECT_TRUE(p64.isIndefinite());
  EXPECT_FALSE(p64.isZero());
  EXPECT_TRUE(p64.isTriangular());
  EXPECT_TRUE(p64.isLowerTriangular());
  EXPECT_TRUE(p64.isUpperTriangular());

  MatrixProperties p65(MatrixProperties::MINUS_IDENTITY, MatrixProperties::INDEFINITE);
  EXPECT_EQ(p65.shape(), MatrixProperties::MINUS_IDENTITY);
  EXPECT_EQ(p65.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_TRUE(p65.isConstant());
  EXPECT_TRUE(p65.isDiagonal());
  EXPECT_FALSE(p65.isIdentity());
  EXPECT_TRUE(p65.isInvertible());
  EXPECT_TRUE(p65.isMinusIdentity());
  EXPECT_TRUE(p65.isMultipleOfIdentity());
  EXPECT_TRUE(p65.isNegativeDefinite());
  EXPECT_TRUE(p65.isNegativeSemidefinite());
  EXPECT_TRUE(p65.isNonZeroIndefinite());
  EXPECT_FALSE(p65.isPositiveDefinite());
  EXPECT_FALSE(p65.isPositiveSemiDefinite());
  EXPECT_TRUE(p65.isSymmetric());
  EXPECT_TRUE(p65.isIndefinite());
  EXPECT_FALSE(p65.isZero());
  EXPECT_TRUE(p65.isTriangular());
  EXPECT_TRUE(p65.isLowerTriangular());
  EXPECT_TRUE(p65.isUpperTriangular());

  MatrixProperties p66(MatrixProperties::MINUS_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);
  EXPECT_EQ(p66.shape(), MatrixProperties::MINUS_IDENTITY);
  EXPECT_EQ(p66.positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_TRUE(p66.isConstant());
  EXPECT_TRUE(p66.isDiagonal());
  EXPECT_FALSE(p66.isIdentity());
  EXPECT_TRUE(p66.isInvertible());
  EXPECT_TRUE(p66.isMinusIdentity());
  EXPECT_TRUE(p66.isMultipleOfIdentity());
  EXPECT_TRUE(p66.isNegativeDefinite());
  EXPECT_TRUE(p66.isNegativeSemidefinite());
  EXPECT_TRUE(p66.isNonZeroIndefinite());
  EXPECT_FALSE(p66.isPositiveDefinite());
  EXPECT_FALSE(p66.isPositiveSemiDefinite());
  EXPECT_TRUE(p66.isSymmetric());
  EXPECT_TRUE(p66.isIndefinite());
  EXPECT_FALSE(p66.isZero());
  EXPECT_TRUE(p66.isTriangular());
  EXPECT_TRUE(p66.isLowerTriangular());
  EXPECT_TRUE(p66.isUpperTriangular());

  MatrixProperties p71(MatrixProperties::ZERO, MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_EQ(p71.shape(), MatrixProperties::ZERO);
  EXPECT_EQ(p71.positiveness(), MatrixProperties::POSITIVE_SEMIDEFINITE);
  EXPECT_TRUE(p71.isConstant());
  EXPECT_TRUE(p71.isDiagonal());
  EXPECT_FALSE(p71.isIdentity());
  EXPECT_FALSE(p71.isInvertible());
  EXPECT_FALSE(p71.isMinusIdentity());
  EXPECT_TRUE(p71.isMultipleOfIdentity());
  EXPECT_FALSE(p71.isNegativeDefinite());
  EXPECT_TRUE(p71.isNegativeSemidefinite());
  EXPECT_FALSE(p71.isNonZeroIndefinite());
  EXPECT_FALSE(p71.isPositiveDefinite());
  EXPECT_TRUE(p71.isPositiveSemiDefinite());
  EXPECT_TRUE(p71.isSymmetric());
  EXPECT_TRUE(p71.isIndefinite());
  EXPECT_TRUE(p71.isZero());
  EXPECT_TRUE(p71.isTriangular());
  EXPECT_TRUE(p71.isLowerTriangular());
  EXPECT_TRUE(p71.isUpperTriangular());


  EXPECT_THROW(
    MatrixProperties p72(MatrixProperties::ZERO, MatrixProperties::POSITIVE_DEFINITE)
    , std::runtime_error);

  MatrixProperties p73(MatrixProperties::ZERO, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_EQ(p73.shape(), MatrixProperties::ZERO);
  EXPECT_EQ(p73.positiveness(), MatrixProperties::NEGATIVE_SEMIDEFINITE);
  EXPECT_TRUE(p73.isConstant());
  EXPECT_TRUE(p73.isDiagonal());
  EXPECT_FALSE(p73.isIdentity());
  EXPECT_FALSE(p73.isInvertible());
  EXPECT_FALSE(p73.isMinusIdentity());
  EXPECT_TRUE(p73.isMultipleOfIdentity());
  EXPECT_FALSE(p73.isNegativeDefinite());
  EXPECT_TRUE(p73.isNegativeSemidefinite());
  EXPECT_FALSE(p73.isNonZeroIndefinite());
  EXPECT_FALSE(p73.isPositiveDefinite());
  EXPECT_TRUE(p73.isPositiveSemiDefinite());
  EXPECT_TRUE(p73.isSymmetric());
  EXPECT_TRUE(p73.isIndefinite());
  EXPECT_TRUE(p73.isZero());
  EXPECT_TRUE(p73.isTriangular());
  EXPECT_TRUE(p73.isLowerTriangular());
  EXPECT_TRUE(p73.isUpperTriangular());

  EXPECT_THROW(
    MatrixProperties p74(MatrixProperties::ZERO, MatrixProperties::NEGATIVE_DEFINITE)
    , std::runtime_error);

  MatrixProperties p75(MatrixProperties::ZERO, MatrixProperties::INDEFINITE);
  EXPECT_EQ(p75.shape(), MatrixProperties::ZERO);
  EXPECT_EQ(p75.positiveness(), MatrixProperties::INDEFINITE);
  EXPECT_TRUE(p75.isConstant());
  EXPECT_TRUE(p75.isDiagonal());
  EXPECT_FALSE(p75.isIdentity());
  EXPECT_FALSE(p75.isInvertible());
  EXPECT_FALSE(p75.isMinusIdentity());
  EXPECT_TRUE(p75.isMultipleOfIdentity());
  EXPECT_FALSE(p75.isNegativeDefinite());
  EXPECT_TRUE(p75.isNegativeSemidefinite());
  EXPECT_FALSE(p75.isNonZeroIndefinite());
  EXPECT_FALSE(p75.isPositiveDefinite());
  EXPECT_TRUE(p75.isPositiveSemiDefinite());
  EXPECT_TRUE(p75.isSymmetric());
  EXPECT_TRUE(p75.isIndefinite());
  EXPECT_TRUE(p75.isZero());
  EXPECT_TRUE(p75.isTriangular());
  EXPECT_TRUE(p75.isLowerTriangular());
  EXPECT_TRUE(p75.isUpperTriangular());

  EXPECT_THROW(
    MatrixProperties p76(MatrixProperties::ZERO, MatrixProperties::NON_ZERO_INDEFINITE)
    , std::runtime_error);
}


#define buildAndCheck(shouldThrow, ... ) \
  if (shouldThrow) {\
    EXPECT_THROW(MatrixProperties(__VA_ARGS__), std::runtime_error); \
  } else {\
    EXPECT_NO_THROW(MatrixProperties(__VA_ARGS__)); }

TEST(MatrixPropertiesTest, constnessCompatibility) {  // NOLINT
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::GENERAL, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::GENERAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::GENERAL, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::GENERAL, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::GENERAL, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::DIAGONAL, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::DIAGONAL, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::DIAGONAL, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::IDENTITY, MatrixProperties::NA);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NA);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::ZERO, MatrixProperties::NA);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::ZERO, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::ZERO, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::ZERO, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::ZERO, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::ZERO, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(false), MatrixProperties::ZERO, MatrixProperties::NON_ZERO_INDEFINITE);


  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::GENERAL, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::GENERAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::GENERAL, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::GENERAL, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::GENERAL, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::DIAGONAL, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::DIAGONAL, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::DIAGONAL, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::IDENTITY, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(true), MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(true), MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NA);
  buildAndCheck(true, MatrixProperties::Constness(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::ZERO, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::ZERO, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(true), MatrixProperties::ZERO, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::ZERO, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(true), MatrixProperties::ZERO, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Constness(true), MatrixProperties::ZERO, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Constness(true), MatrixProperties::ZERO, MatrixProperties::NON_ZERO_INDEFINITE);
}

TEST(MatrixPropertiesTest, invertibilityCompatibility) {  // NOLINT
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::GENERAL, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::GENERAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::GENERAL, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::GENERAL, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::GENERAL, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::DIAGONAL, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::DIAGONAL, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::DIAGONAL, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::IDENTITY, MatrixProperties::NA);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NA);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::ZERO, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::ZERO, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::ZERO, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::ZERO, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::ZERO, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(false), MatrixProperties::ZERO, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(false), MatrixProperties::ZERO, MatrixProperties::NON_ZERO_INDEFINITE);


  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::GENERAL, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::GENERAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::GENERAL, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::GENERAL, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::GENERAL, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::DIAGONAL, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::DIAGONAL, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::DIAGONAL, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::IDENTITY, MatrixProperties::NA);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NA);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::INDEFINITE);
  buildAndCheck(false, MatrixProperties::Invertibility(true), MatrixProperties::MINUS_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);

  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::ZERO, MatrixProperties::NA);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::ZERO, MatrixProperties::POSITIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::ZERO, MatrixProperties::POSITIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::ZERO, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::ZERO, MatrixProperties::NEGATIVE_DEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::ZERO, MatrixProperties::INDEFINITE);
  buildAndCheck(true, MatrixProperties::Invertibility(true), MatrixProperties::ZERO, MatrixProperties::NON_ZERO_INDEFINITE);
}

TEST(MatrixPropertiesTest, argumentOrderAndRepetition) {  // NOLINT
  MatrixProperties::Shape s = MatrixProperties::GENERAL;
  MatrixProperties::Positiveness p = MatrixProperties::NA;
  MatrixProperties::Constness c;
  MatrixProperties::Invertibility i;

  buildAndCheck(false, s);
  buildAndCheck(false, s);
  buildAndCheck(false, c);
  buildAndCheck(false, i);


  buildAndCheck(false, s, p);
  buildAndCheck(false, s, c);
  buildAndCheck(false, s, i);

  buildAndCheck(false, p, s);
  buildAndCheck(false, p, c);
  buildAndCheck(false, p, i);

  buildAndCheck(false, c, s);
  buildAndCheck(false, c, p);
  buildAndCheck(false, c, i);

  buildAndCheck(false, i, s);
  buildAndCheck(false, i, p);
  buildAndCheck(false, i, c);


  buildAndCheck(false, s, p, c);
  buildAndCheck(false, s, p, i);
  buildAndCheck(false, s, c, p);
  buildAndCheck(false, s, c, i);
  buildAndCheck(false, s, i, p);
  buildAndCheck(false, s, i, c);

  buildAndCheck(false, p, s, c);
  buildAndCheck(false, p, s, i);
  buildAndCheck(false, p, c, s);
  buildAndCheck(false, p, c, i);
  buildAndCheck(false, p, i, s);
  buildAndCheck(false, p, i, c);

  buildAndCheck(false, c, s, p);
  buildAndCheck(false, c, s, i);
  buildAndCheck(false, c, p, s);
  buildAndCheck(false, c, p, i);
  buildAndCheck(false, c, i, s);
  buildAndCheck(false, c, i, p);

  buildAndCheck(false, i, s, p);
  buildAndCheck(false, i, s, c);
  buildAndCheck(false, i, p, s);
  buildAndCheck(false, i, p, c);
  buildAndCheck(false, i, c, s);
  buildAndCheck(false, i, c, p);


  buildAndCheck(false, s, p, c, i);
  buildAndCheck(false, s, p, i, c);
  buildAndCheck(false, s, c, p, i);
  buildAndCheck(false, s, c, i, p);
  buildAndCheck(false, s, i, p, c);
  buildAndCheck(false, s, i, c, p);

  buildAndCheck(false, p, s, c, i);
  buildAndCheck(false, p, s, i, c);
  buildAndCheck(false, p, c, s, i);
  buildAndCheck(false, p, c, i, s);
  buildAndCheck(false, p, i, s, c);
  buildAndCheck(false, p, i, c, s);

  buildAndCheck(false, c, s, p, i);
  buildAndCheck(false, c, s, i, p);
  buildAndCheck(false, c, p, s, i);
  buildAndCheck(false, c, p, i, s);
  buildAndCheck(false, c, i, s, p);
  buildAndCheck(false, c, i, p, s);

  buildAndCheck(false, i, s, p, c);
  buildAndCheck(false, i, s, c, p);
  buildAndCheck(false, i, p, s, c);
  buildAndCheck(false, i, p, c, s);
  buildAndCheck(false, i, c, s, p);
  buildAndCheck(false, i, c, p, s);
}

TEST(MatrixPropertiesTest, keepProperties) {  // NOLINT
  MatrixWithProperties M(5, 5);
  MatrixProperties p(MatrixProperties::MINUS_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);
  M.properties(p);
  EXPECT_EQ(M.properties().shape(), MatrixProperties::MINUS_IDENTITY);
  EXPECT_EQ(M.properties().positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_TRUE(M.properties().isConstant());
  EXPECT_TRUE(M.properties().isInvertible());

  Eigen::MatrixXd m = Eigen::MatrixXd::Random(5, 5);
  M = m;
  EXPECT_EQ(M.properties().shape(), MatrixProperties::GENERAL);
  EXPECT_EQ(M.properties().positiveness(), MatrixProperties::NA);
  EXPECT_FALSE(M.properties().isConstant());
  EXPECT_FALSE(M.properties().isInvertible());

  M.properties(p);
  M.keepProperties(true) = m;
  EXPECT_EQ(M.properties().shape(), MatrixProperties::MINUS_IDENTITY);
  EXPECT_EQ(M.properties().positiveness(), MatrixProperties::NEGATIVE_DEFINITE);
  EXPECT_TRUE(M.properties().isConstant());
  EXPECT_TRUE(M.properties().isInvertible());

  M.keepProperties(false) = m;
  EXPECT_EQ(M.properties().shape(), MatrixProperties::GENERAL);
  EXPECT_EQ(M.properties().positiveness(), MatrixProperties::NA);
  EXPECT_FALSE(M.properties().isConstant());
  EXPECT_FALSE(M.properties().isInvertible());
}
