#include "MatrixProperties.h"

// boost
#define BOOST_TEST_MODULE ConstraintTest
#include <boost/test/unit_test.hpp>

using namespace tvm;

BOOST_AUTO_TEST_CASE(ShapePropertiesTest)
{
  MatrixProperties p0;
  BOOST_CHECK(p0.shape() == MatrixProperties::GENERAL);
  BOOST_CHECK(p0.positiveness() == MatrixProperties::NA);
  BOOST_CHECK(!p0.isConstant());
  BOOST_CHECK(!p0.isDiagonal());
  BOOST_CHECK(!p0.isIdentity());
  BOOST_CHECK(!p0.isInvertible());
  BOOST_CHECK(!p0.isMinusIdentity());
  BOOST_CHECK(!p0.isMultipleOfIdentity());
  BOOST_CHECK(!p0.isNegativeDefinite());
  BOOST_CHECK(!p0.isNegativeSemidefinite());
  BOOST_CHECK(!p0.isNonZeroIndefinite());
  BOOST_CHECK(!p0.isPositiveDefinite());
  BOOST_CHECK(!p0.isPositiveSemiDefinite());
  BOOST_CHECK(!p0.isSymmetric());
  BOOST_CHECK(!p0.isIndefinite());
  BOOST_CHECK(!p0.isZero());
  BOOST_CHECK(!p0.isTriangular());
  BOOST_CHECK(!p0.isLowerTriangular());
  BOOST_CHECK(!p0.isUpperTriangular());

  MatrixProperties p1(MatrixProperties::LOWER_TRIANGULAR);
  BOOST_CHECK(p1.shape() == MatrixProperties::LOWER_TRIANGULAR);
  BOOST_CHECK(p1.positiveness() == MatrixProperties::NA);
  BOOST_CHECK(!p1.isConstant());
  BOOST_CHECK(!p1.isDiagonal());
  BOOST_CHECK(!p1.isIdentity());
  BOOST_CHECK(!p1.isInvertible());
  BOOST_CHECK(!p1.isMinusIdentity());
  BOOST_CHECK(!p1.isMultipleOfIdentity());
  BOOST_CHECK(!p1.isNegativeDefinite());
  BOOST_CHECK(!p1.isNegativeSemidefinite());
  BOOST_CHECK(!p1.isNonZeroIndefinite());
  BOOST_CHECK(!p1.isPositiveDefinite());
  BOOST_CHECK(!p1.isPositiveSemiDefinite());
  BOOST_CHECK(!p1.isSymmetric());
  BOOST_CHECK(!p1.isIndefinite());
  BOOST_CHECK(!p1.isZero());
  BOOST_CHECK(p1.isTriangular());
  BOOST_CHECK(p1.isLowerTriangular());
  BOOST_CHECK(!p1.isUpperTriangular());

  MatrixProperties p2(MatrixProperties::UPPER_TRIANGULAR);
  BOOST_CHECK(p2.shape() == MatrixProperties::UPPER_TRIANGULAR);
  BOOST_CHECK(p2.positiveness() == MatrixProperties::NA);
  BOOST_CHECK(!p2.isConstant());
  BOOST_CHECK(!p2.isDiagonal());
  BOOST_CHECK(!p2.isIdentity());
  BOOST_CHECK(!p2.isInvertible());
  BOOST_CHECK(!p2.isMinusIdentity());
  BOOST_CHECK(!p2.isMultipleOfIdentity());
  BOOST_CHECK(!p2.isNegativeDefinite());
  BOOST_CHECK(!p2.isNegativeSemidefinite());
  BOOST_CHECK(!p2.isNonZeroIndefinite());
  BOOST_CHECK(!p2.isPositiveDefinite());
  BOOST_CHECK(!p2.isPositiveSemiDefinite());
  BOOST_CHECK(!p2.isSymmetric());
  BOOST_CHECK(!p2.isIndefinite());
  BOOST_CHECK(!p2.isZero());
  BOOST_CHECK(p2.isTriangular());
  BOOST_CHECK(!p2.isLowerTriangular());
  BOOST_CHECK(p2.isUpperTriangular());

  MatrixProperties p3(MatrixProperties::DIAGONAL);
  BOOST_CHECK(p3.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p3.positiveness() == MatrixProperties::INDEFINITE);
  BOOST_CHECK(!p3.isConstant());
  BOOST_CHECK(p3.isDiagonal());
  BOOST_CHECK(!p3.isIdentity());
  BOOST_CHECK(!p3.isInvertible());
  BOOST_CHECK(!p3.isMinusIdentity());
  BOOST_CHECK(!p3.isMultipleOfIdentity());
  BOOST_CHECK(!p3.isNegativeDefinite());
  BOOST_CHECK(!p3.isNegativeSemidefinite());
  BOOST_CHECK(!p3.isNonZeroIndefinite());
  BOOST_CHECK(!p3.isPositiveDefinite());
  BOOST_CHECK(!p3.isPositiveSemiDefinite());
  BOOST_CHECK(p3.isSymmetric());
  BOOST_CHECK(p3.isIndefinite());
  BOOST_CHECK(!p3.isZero());
  BOOST_CHECK(p3.isTriangular());
  BOOST_CHECK(p3.isLowerTriangular());
  BOOST_CHECK(p3.isUpperTriangular());

  MatrixProperties p4(MatrixProperties::MULTIPLE_OF_IDENTITY);
  BOOST_CHECK(p4.shape() == MatrixProperties::MULTIPLE_OF_IDENTITY);
  BOOST_CHECK(p4.positiveness() == MatrixProperties::INDEFINITE);
  BOOST_CHECK(!p4.isConstant());
  BOOST_CHECK(p4.isDiagonal());
  BOOST_CHECK(!p4.isIdentity());
  BOOST_CHECK(!p4.isInvertible());
  BOOST_CHECK(!p4.isMinusIdentity());
  BOOST_CHECK(p4.isMultipleOfIdentity());
  BOOST_CHECK(!p4.isNegativeDefinite());
  BOOST_CHECK(!p4.isNegativeSemidefinite());
  BOOST_CHECK(!p4.isNonZeroIndefinite());
  BOOST_CHECK(!p4.isPositiveDefinite());
  BOOST_CHECK(!p4.isPositiveSemiDefinite());
  BOOST_CHECK(p4.isSymmetric());
  BOOST_CHECK(p4.isIndefinite());
  BOOST_CHECK(!p4.isZero());
  BOOST_CHECK(p4.isTriangular());
  BOOST_CHECK(p4.isLowerTriangular());
  BOOST_CHECK(p4.isUpperTriangular());

  MatrixProperties p5(MatrixProperties::IDENTITY);
  BOOST_CHECK(p5.shape() == MatrixProperties::IDENTITY);
  BOOST_CHECK(p5.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p5.isConstant());
  BOOST_CHECK(p5.isDiagonal());
  BOOST_CHECK(p5.isIdentity());
  BOOST_CHECK(p5.isInvertible());
  BOOST_CHECK(!p5.isMinusIdentity());
  BOOST_CHECK(p5.isMultipleOfIdentity());
  BOOST_CHECK(!p5.isNegativeDefinite());
  BOOST_CHECK(!p5.isNegativeSemidefinite());
  BOOST_CHECK(p5.isNonZeroIndefinite());
  BOOST_CHECK(p5.isPositiveDefinite());
  BOOST_CHECK(p5.isPositiveSemiDefinite());
  BOOST_CHECK(p5.isSymmetric());
  BOOST_CHECK(p5.isIndefinite());
  BOOST_CHECK(!p5.isZero());
  BOOST_CHECK(p5.isTriangular());
  BOOST_CHECK(p5.isLowerTriangular());
  BOOST_CHECK(p5.isUpperTriangular());

  MatrixProperties p6(MatrixProperties::MINUS_IDENTITY);
  BOOST_CHECK(p6.shape() == MatrixProperties::MINUS_IDENTITY);
  BOOST_CHECK(p6.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p6.isConstant());
  BOOST_CHECK(p6.isDiagonal());
  BOOST_CHECK(!p6.isIdentity());
  BOOST_CHECK(p6.isInvertible());
  BOOST_CHECK(p6.isMinusIdentity());
  BOOST_CHECK(p6.isMultipleOfIdentity());
  BOOST_CHECK(p6.isNegativeDefinite());
  BOOST_CHECK(p6.isNegativeSemidefinite());
  BOOST_CHECK(p6.isNonZeroIndefinite());
  BOOST_CHECK(!p6.isPositiveDefinite());
  BOOST_CHECK(!p6.isPositiveSemiDefinite());
  BOOST_CHECK(p6.isSymmetric());
  BOOST_CHECK(p6.isIndefinite());
  BOOST_CHECK(!p6.isZero());
  BOOST_CHECK(p6.isTriangular());
  BOOST_CHECK(p6.isLowerTriangular());
  BOOST_CHECK(p6.isUpperTriangular());

  MatrixProperties p7(MatrixProperties::ZERO);
  BOOST_CHECK(p7.shape() == MatrixProperties::ZERO);
  BOOST_CHECK(p7.positiveness() == MatrixProperties::INDEFINITE);
  BOOST_CHECK(p7.isConstant());
  BOOST_CHECK(p7.isDiagonal());
  BOOST_CHECK(!p7.isIdentity());
  BOOST_CHECK(!p7.isInvertible());
  BOOST_CHECK(!p7.isMinusIdentity());
  BOOST_CHECK(p7.isMultipleOfIdentity());
  BOOST_CHECK(!p7.isNegativeDefinite());
  BOOST_CHECK(p7.isNegativeSemidefinite());
  BOOST_CHECK(!p7.isNonZeroIndefinite());
  BOOST_CHECK(!p7.isPositiveDefinite());
  BOOST_CHECK(p7.isPositiveSemiDefinite());
  BOOST_CHECK(p7.isSymmetric());
  BOOST_CHECK(p7.isIndefinite());
  BOOST_CHECK(p7.isZero());
  BOOST_CHECK(p7.isTriangular());
  BOOST_CHECK(p7.isLowerTriangular());
  BOOST_CHECK(p7.isUpperTriangular());
}

BOOST_AUTO_TEST_CASE(DeductionPropertiesTest)
{
  MatrixProperties p01(MatrixProperties::GENERAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(p01.shape() == MatrixProperties::GENERAL);
  BOOST_CHECK(p01.positiveness() == MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(!p01.isConstant());
  BOOST_CHECK(!p01.isDiagonal());
  BOOST_CHECK(!p01.isIdentity());
  BOOST_CHECK(!p01.isInvertible());
  BOOST_CHECK(!p01.isMinusIdentity());
  BOOST_CHECK(!p01.isMultipleOfIdentity());
  BOOST_CHECK(!p01.isNegativeDefinite());
  BOOST_CHECK(!p01.isNegativeSemidefinite());
  BOOST_CHECK(!p01.isNonZeroIndefinite());
  BOOST_CHECK(!p01.isPositiveDefinite());
  BOOST_CHECK(p01.isPositiveSemiDefinite());
  BOOST_CHECK(p01.isSymmetric());
  BOOST_CHECK(p01.isIndefinite());
  BOOST_CHECK(!p01.isZero());
  BOOST_CHECK(!p01.isTriangular());
  BOOST_CHECK(!p01.isLowerTriangular());
  BOOST_CHECK(!p01.isUpperTriangular());

  MatrixProperties p02(MatrixProperties::GENERAL, MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p02.shape() == MatrixProperties::GENERAL);
  BOOST_CHECK(p02.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(!p02.isConstant());
  BOOST_CHECK(!p02.isDiagonal());
  BOOST_CHECK(!p02.isIdentity());
  BOOST_CHECK(p02.isInvertible());
  BOOST_CHECK(!p02.isMinusIdentity());
  BOOST_CHECK(!p02.isMultipleOfIdentity());
  BOOST_CHECK(!p02.isNegativeDefinite());
  BOOST_CHECK(!p02.isNegativeSemidefinite());
  BOOST_CHECK(p02.isNonZeroIndefinite());
  BOOST_CHECK(p02.isPositiveDefinite());
  BOOST_CHECK(p02.isPositiveSemiDefinite());
  BOOST_CHECK(p02.isSymmetric());
  BOOST_CHECK(p02.isIndefinite());
  BOOST_CHECK(!p02.isZero());
  BOOST_CHECK(!p02.isTriangular());
  BOOST_CHECK(!p02.isLowerTriangular());
  BOOST_CHECK(!p02.isUpperTriangular());

  MatrixProperties p03(MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(p03.shape() == MatrixProperties::GENERAL);
  BOOST_CHECK(p03.positiveness() == MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(!p03.isConstant());
  BOOST_CHECK(!p03.isDiagonal());
  BOOST_CHECK(!p03.isIdentity());
  BOOST_CHECK(!p03.isInvertible());
  BOOST_CHECK(!p03.isMinusIdentity());
  BOOST_CHECK(!p03.isMultipleOfIdentity());
  BOOST_CHECK(!p03.isNegativeDefinite());
  BOOST_CHECK(p03.isNegativeSemidefinite());
  BOOST_CHECK(!p03.isNonZeroIndefinite());
  BOOST_CHECK(!p03.isPositiveDefinite());
  BOOST_CHECK(!p03.isPositiveSemiDefinite());
  BOOST_CHECK(p03.isSymmetric());
  BOOST_CHECK(p03.isIndefinite());
  BOOST_CHECK(!p03.isZero());
  BOOST_CHECK(!p03.isTriangular());
  BOOST_CHECK(!p03.isLowerTriangular());
  BOOST_CHECK(!p03.isUpperTriangular());

  MatrixProperties p04(MatrixProperties::GENERAL, MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p04.shape() == MatrixProperties::GENERAL);
  BOOST_CHECK(p04.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(!p04.isConstant());
  BOOST_CHECK(!p04.isDiagonal());
  BOOST_CHECK(!p04.isIdentity());
  BOOST_CHECK(p04.isInvertible());
  BOOST_CHECK(!p04.isMinusIdentity());
  BOOST_CHECK(!p04.isMultipleOfIdentity());
  BOOST_CHECK(p04.isNegativeDefinite());
  BOOST_CHECK(p04.isNegativeSemidefinite());
  BOOST_CHECK(p04.isNonZeroIndefinite());
  BOOST_CHECK(!p04.isPositiveDefinite());
  BOOST_CHECK(!p04.isPositiveSemiDefinite());
  BOOST_CHECK(p04.isSymmetric());
  BOOST_CHECK(p04.isIndefinite());
  BOOST_CHECK(!p04.isZero());
  BOOST_CHECK(!p04.isTriangular());
  BOOST_CHECK(!p04.isLowerTriangular());
  BOOST_CHECK(!p04.isUpperTriangular());

  MatrixProperties p05(MatrixProperties::GENERAL, MatrixProperties::INDEFINITE);
  BOOST_CHECK(p05.shape() == MatrixProperties::GENERAL);
  BOOST_CHECK(p05.positiveness() == MatrixProperties::INDEFINITE);
  BOOST_CHECK(!p05.isConstant());
  BOOST_CHECK(!p05.isDiagonal());
  BOOST_CHECK(!p05.isIdentity());
  BOOST_CHECK(!p05.isInvertible());
  BOOST_CHECK(!p05.isMinusIdentity());
  BOOST_CHECK(!p05.isMultipleOfIdentity());
  BOOST_CHECK(!p05.isNegativeDefinite());
  BOOST_CHECK(!p05.isNegativeSemidefinite());
  BOOST_CHECK(!p05.isNonZeroIndefinite());
  BOOST_CHECK(!p05.isPositiveDefinite());
  BOOST_CHECK(!p05.isPositiveSemiDefinite());
  BOOST_CHECK(p05.isSymmetric());
  BOOST_CHECK(p05.isIndefinite());
  BOOST_CHECK(!p05.isZero());
  BOOST_CHECK(!p05.isTriangular());
  BOOST_CHECK(!p05.isLowerTriangular());
  BOOST_CHECK(!p05.isUpperTriangular());

  MatrixProperties p06(MatrixProperties::GENERAL, MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(p06.shape() == MatrixProperties::GENERAL);
  BOOST_CHECK(p06.positiveness() == MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(!p06.isConstant());
  BOOST_CHECK(!p06.isDiagonal());
  BOOST_CHECK(!p06.isIdentity());
  BOOST_CHECK(p06.isInvertible());
  BOOST_CHECK(!p06.isMinusIdentity());
  BOOST_CHECK(!p06.isMultipleOfIdentity());
  BOOST_CHECK(!p06.isNegativeDefinite());
  BOOST_CHECK(!p06.isNegativeSemidefinite());
  BOOST_CHECK(p06.isNonZeroIndefinite());
  BOOST_CHECK(!p06.isPositiveDefinite());
  BOOST_CHECK(!p06.isPositiveSemiDefinite());
  BOOST_CHECK(p06.isSymmetric());
  BOOST_CHECK(p06.isIndefinite());
  BOOST_CHECK(!p06.isZero());
  BOOST_CHECK(!p06.isTriangular());
  BOOST_CHECK(!p06.isLowerTriangular());
  BOOST_CHECK(!p06.isUpperTriangular());

  MatrixProperties p11(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(p11.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p11.positiveness() == MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(!p11.isConstant());
  BOOST_CHECK(p11.isDiagonal());
  BOOST_CHECK(!p11.isIdentity());
  BOOST_CHECK(!p11.isInvertible());
  BOOST_CHECK(!p11.isMinusIdentity());
  BOOST_CHECK(!p11.isMultipleOfIdentity());
  BOOST_CHECK(!p11.isNegativeDefinite());
  BOOST_CHECK(!p11.isNegativeSemidefinite());
  BOOST_CHECK(!p11.isNonZeroIndefinite());
  BOOST_CHECK(!p11.isPositiveDefinite());
  BOOST_CHECK(p11.isPositiveSemiDefinite());
  BOOST_CHECK(p11.isSymmetric());
  BOOST_CHECK(p11.isIndefinite());
  BOOST_CHECK(!p11.isZero());
  BOOST_CHECK(p11.isTriangular());
  BOOST_CHECK(p11.isLowerTriangular());
  BOOST_CHECK(p11.isUpperTriangular());

  MatrixProperties p12(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p12.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p12.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(!p12.isConstant());
  BOOST_CHECK(p12.isDiagonal());
  BOOST_CHECK(!p12.isIdentity());
  BOOST_CHECK(p12.isInvertible());
  BOOST_CHECK(!p12.isMinusIdentity());
  BOOST_CHECK(!p12.isMultipleOfIdentity());
  BOOST_CHECK(!p12.isNegativeDefinite());
  BOOST_CHECK(!p12.isNegativeSemidefinite());
  BOOST_CHECK(p12.isNonZeroIndefinite());
  BOOST_CHECK(p12.isPositiveDefinite());
  BOOST_CHECK(p12.isPositiveSemiDefinite());
  BOOST_CHECK(p12.isSymmetric());
  BOOST_CHECK(p12.isIndefinite());
  BOOST_CHECK(!p12.isZero());
  BOOST_CHECK(p12.isTriangular());
  BOOST_CHECK(p12.isLowerTriangular());
  BOOST_CHECK(p12.isUpperTriangular());

  MatrixProperties p13(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(p13.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p13.positiveness() == MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(!p13.isConstant());
  BOOST_CHECK(p13.isDiagonal());
  BOOST_CHECK(!p13.isIdentity());
  BOOST_CHECK(!p13.isInvertible());
  BOOST_CHECK(!p13.isMinusIdentity());
  BOOST_CHECK(!p13.isMultipleOfIdentity());
  BOOST_CHECK(!p13.isNegativeDefinite());
  BOOST_CHECK(p13.isNegativeSemidefinite());
  BOOST_CHECK(!p13.isNonZeroIndefinite());
  BOOST_CHECK(!p13.isPositiveDefinite());
  BOOST_CHECK(!p13.isPositiveSemiDefinite());
  BOOST_CHECK(p13.isSymmetric());
  BOOST_CHECK(p13.isIndefinite());
  BOOST_CHECK(!p13.isZero());
  BOOST_CHECK(p13.isTriangular());
  BOOST_CHECK(p13.isLowerTriangular());
  BOOST_CHECK(p13.isUpperTriangular());

  MatrixProperties p14(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p14.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p14.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(!p14.isConstant());
  BOOST_CHECK(p14.isDiagonal());
  BOOST_CHECK(!p14.isIdentity());
  BOOST_CHECK(p14.isInvertible());
  BOOST_CHECK(!p14.isMinusIdentity());
  BOOST_CHECK(!p14.isMultipleOfIdentity());
  BOOST_CHECK(p14.isNegativeDefinite());
  BOOST_CHECK(p14.isNegativeSemidefinite());
  BOOST_CHECK(p14.isNonZeroIndefinite());
  BOOST_CHECK(!p14.isPositiveDefinite());
  BOOST_CHECK(!p14.isPositiveSemiDefinite());
  BOOST_CHECK(p14.isSymmetric());
  BOOST_CHECK(p14.isIndefinite());
  BOOST_CHECK(!p14.isZero());
  BOOST_CHECK(p14.isTriangular());
  BOOST_CHECK(p14.isLowerTriangular());
  BOOST_CHECK(p14.isUpperTriangular());

  MatrixProperties p15(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::INDEFINITE);
  BOOST_CHECK(p15.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p15.positiveness() == MatrixProperties::INDEFINITE);
  BOOST_CHECK(!p15.isConstant());
  BOOST_CHECK(p15.isDiagonal());
  BOOST_CHECK(!p15.isIdentity());
  BOOST_CHECK(!p15.isInvertible());
  BOOST_CHECK(!p15.isMinusIdentity());
  BOOST_CHECK(!p15.isMultipleOfIdentity());
  BOOST_CHECK(!p15.isNegativeDefinite());
  BOOST_CHECK(!p15.isNegativeSemidefinite());
  BOOST_CHECK(!p15.isNonZeroIndefinite());
  BOOST_CHECK(!p15.isPositiveDefinite());
  BOOST_CHECK(!p15.isPositiveSemiDefinite());
  BOOST_CHECK(p15.isSymmetric());
  BOOST_CHECK(p15.isIndefinite());
  BOOST_CHECK(!p15.isZero());
  BOOST_CHECK(p15.isTriangular());
  BOOST_CHECK(p15.isLowerTriangular());
  BOOST_CHECK(p15.isUpperTriangular());

  MatrixProperties p16(MatrixProperties::LOWER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(p16.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p16.positiveness() == MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(!p16.isConstant());
  BOOST_CHECK(p16.isDiagonal());
  BOOST_CHECK(!p16.isIdentity());
  BOOST_CHECK(p16.isInvertible());
  BOOST_CHECK(!p16.isMinusIdentity());
  BOOST_CHECK(!p16.isMultipleOfIdentity());
  BOOST_CHECK(!p16.isNegativeDefinite());
  BOOST_CHECK(!p16.isNegativeSemidefinite());
  BOOST_CHECK(p16.isNonZeroIndefinite());
  BOOST_CHECK(!p16.isPositiveDefinite());
  BOOST_CHECK(!p16.isPositiveSemiDefinite());
  BOOST_CHECK(p16.isSymmetric());
  BOOST_CHECK(p16.isIndefinite());
  BOOST_CHECK(!p16.isZero());
  BOOST_CHECK(p16.isTriangular());
  BOOST_CHECK(p16.isLowerTriangular());
  BOOST_CHECK(p16.isUpperTriangular());


  MatrixProperties p21(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(p21.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p21.positiveness() == MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(!p21.isConstant());
  BOOST_CHECK(p21.isDiagonal());
  BOOST_CHECK(!p21.isIdentity());
  BOOST_CHECK(!p21.isInvertible());
  BOOST_CHECK(!p21.isMinusIdentity());
  BOOST_CHECK(!p21.isMultipleOfIdentity());
  BOOST_CHECK(!p21.isNegativeDefinite());
  BOOST_CHECK(!p21.isNegativeSemidefinite());
  BOOST_CHECK(!p21.isNonZeroIndefinite());
  BOOST_CHECK(!p21.isPositiveDefinite());
  BOOST_CHECK(p21.isPositiveSemiDefinite());
  BOOST_CHECK(p21.isSymmetric());
  BOOST_CHECK(p21.isIndefinite());
  BOOST_CHECK(!p21.isZero());
  BOOST_CHECK(p21.isTriangular());
  BOOST_CHECK(p21.isLowerTriangular());
  BOOST_CHECK(p21.isUpperTriangular());

  MatrixProperties p22(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p22.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p22.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(!p22.isConstant());
  BOOST_CHECK(p22.isDiagonal());
  BOOST_CHECK(!p22.isIdentity());
  BOOST_CHECK(p22.isInvertible());
  BOOST_CHECK(!p22.isMinusIdentity());
  BOOST_CHECK(!p22.isMultipleOfIdentity());
  BOOST_CHECK(!p22.isNegativeDefinite());
  BOOST_CHECK(!p22.isNegativeSemidefinite());
  BOOST_CHECK(p22.isNonZeroIndefinite());
  BOOST_CHECK(p22.isPositiveDefinite());
  BOOST_CHECK(p22.isPositiveSemiDefinite());
  BOOST_CHECK(p22.isSymmetric());
  BOOST_CHECK(p22.isIndefinite());
  BOOST_CHECK(!p22.isZero());
  BOOST_CHECK(p22.isTriangular());
  BOOST_CHECK(p22.isLowerTriangular());
  BOOST_CHECK(p22.isUpperTriangular());

  MatrixProperties p23(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(p23.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p23.positiveness() == MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(!p23.isConstant());
  BOOST_CHECK(p23.isDiagonal());
  BOOST_CHECK(!p23.isIdentity());
  BOOST_CHECK(!p23.isInvertible());
  BOOST_CHECK(!p23.isMinusIdentity());
  BOOST_CHECK(!p23.isMultipleOfIdentity());
  BOOST_CHECK(!p23.isNegativeDefinite());
  BOOST_CHECK(p23.isNegativeSemidefinite());
  BOOST_CHECK(!p23.isNonZeroIndefinite());
  BOOST_CHECK(!p23.isPositiveDefinite());
  BOOST_CHECK(!p23.isPositiveSemiDefinite());
  BOOST_CHECK(p23.isSymmetric());
  BOOST_CHECK(p23.isIndefinite());
  BOOST_CHECK(!p23.isZero());
  BOOST_CHECK(p23.isTriangular());
  BOOST_CHECK(p23.isLowerTriangular());
  BOOST_CHECK(p23.isUpperTriangular());

  MatrixProperties p24(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p24.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p24.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(!p24.isConstant());
  BOOST_CHECK(p24.isDiagonal());
  BOOST_CHECK(!p24.isIdentity());
  BOOST_CHECK(p24.isInvertible());
  BOOST_CHECK(!p24.isMinusIdentity());
  BOOST_CHECK(!p24.isMultipleOfIdentity());
  BOOST_CHECK(p24.isNegativeDefinite());
  BOOST_CHECK(p24.isNegativeSemidefinite());
  BOOST_CHECK(p24.isNonZeroIndefinite());
  BOOST_CHECK(!p24.isPositiveDefinite());
  BOOST_CHECK(!p24.isPositiveSemiDefinite());
  BOOST_CHECK(p24.isSymmetric());
  BOOST_CHECK(p24.isIndefinite());
  BOOST_CHECK(!p24.isZero());
  BOOST_CHECK(p24.isTriangular());
  BOOST_CHECK(p24.isLowerTriangular());
  BOOST_CHECK(p24.isUpperTriangular());

  MatrixProperties p25(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::INDEFINITE);
  BOOST_CHECK(p25.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p25.positiveness() == MatrixProperties::INDEFINITE);
  BOOST_CHECK(!p25.isConstant());
  BOOST_CHECK(p25.isDiagonal());
  BOOST_CHECK(!p25.isIdentity());
  BOOST_CHECK(!p25.isInvertible());
  BOOST_CHECK(!p25.isMinusIdentity());
  BOOST_CHECK(!p25.isMultipleOfIdentity());
  BOOST_CHECK(!p25.isNegativeDefinite());
  BOOST_CHECK(!p25.isNegativeSemidefinite());
  BOOST_CHECK(!p25.isNonZeroIndefinite());
  BOOST_CHECK(!p25.isPositiveDefinite());
  BOOST_CHECK(!p25.isPositiveSemiDefinite());
  BOOST_CHECK(p25.isSymmetric());
  BOOST_CHECK(p25.isIndefinite());
  BOOST_CHECK(!p25.isZero());
  BOOST_CHECK(p25.isTriangular());
  BOOST_CHECK(p25.isLowerTriangular());
  BOOST_CHECK(p25.isUpperTriangular());

  MatrixProperties p26(MatrixProperties::UPPER_TRIANGULAR, MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(p26.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p26.positiveness() == MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(!p26.isConstant());
  BOOST_CHECK(p26.isDiagonal());
  BOOST_CHECK(!p26.isIdentity());
  BOOST_CHECK(p26.isInvertible());
  BOOST_CHECK(!p26.isMinusIdentity());
  BOOST_CHECK(!p26.isMultipleOfIdentity());
  BOOST_CHECK(!p26.isNegativeDefinite());
  BOOST_CHECK(!p26.isNegativeSemidefinite());
  BOOST_CHECK(p26.isNonZeroIndefinite());
  BOOST_CHECK(!p26.isPositiveDefinite());
  BOOST_CHECK(!p26.isPositiveSemiDefinite());
  BOOST_CHECK(p26.isSymmetric());
  BOOST_CHECK(p26.isIndefinite());
  BOOST_CHECK(!p26.isZero());
  BOOST_CHECK(p26.isTriangular());
  BOOST_CHECK(p26.isLowerTriangular());
  BOOST_CHECK(p26.isUpperTriangular());


  MatrixProperties p31(MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(p31.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p31.positiveness() == MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(!p31.isConstant());
  BOOST_CHECK(p31.isDiagonal());
  BOOST_CHECK(!p31.isIdentity());
  BOOST_CHECK(!p31.isInvertible());
  BOOST_CHECK(!p31.isMinusIdentity());
  BOOST_CHECK(!p31.isMultipleOfIdentity());
  BOOST_CHECK(!p31.isNegativeDefinite());
  BOOST_CHECK(!p31.isNegativeSemidefinite());
  BOOST_CHECK(!p31.isNonZeroIndefinite());
  BOOST_CHECK(!p31.isPositiveDefinite());
  BOOST_CHECK(p31.isPositiveSemiDefinite());
  BOOST_CHECK(p31.isSymmetric());
  BOOST_CHECK(p31.isIndefinite());
  BOOST_CHECK(!p31.isZero());
  BOOST_CHECK(p31.isTriangular());
  BOOST_CHECK(p31.isLowerTriangular());
  BOOST_CHECK(p31.isUpperTriangular());

  MatrixProperties p32(MatrixProperties::DIAGONAL, MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p32.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p32.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(!p32.isConstant());
  BOOST_CHECK(p32.isDiagonal());
  BOOST_CHECK(!p32.isIdentity());
  BOOST_CHECK(p32.isInvertible());
  BOOST_CHECK(!p32.isMinusIdentity());
  BOOST_CHECK(!p32.isMultipleOfIdentity());
  BOOST_CHECK(!p32.isNegativeDefinite());
  BOOST_CHECK(!p32.isNegativeSemidefinite());
  BOOST_CHECK(p32.isNonZeroIndefinite());
  BOOST_CHECK(p32.isPositiveDefinite());
  BOOST_CHECK(p32.isPositiveSemiDefinite());
  BOOST_CHECK(p32.isSymmetric());
  BOOST_CHECK(p32.isIndefinite());
  BOOST_CHECK(!p32.isZero());
  BOOST_CHECK(p32.isTriangular());
  BOOST_CHECK(p32.isLowerTriangular());
  BOOST_CHECK(p32.isUpperTriangular());

  MatrixProperties p33(MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(p33.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p33.positiveness() == MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(!p33.isConstant());
  BOOST_CHECK(p33.isDiagonal());
  BOOST_CHECK(!p33.isIdentity());
  BOOST_CHECK(!p33.isInvertible());
  BOOST_CHECK(!p33.isMinusIdentity());
  BOOST_CHECK(!p33.isMultipleOfIdentity());
  BOOST_CHECK(!p33.isNegativeDefinite());
  BOOST_CHECK(p33.isNegativeSemidefinite());
  BOOST_CHECK(!p33.isNonZeroIndefinite());
  BOOST_CHECK(!p33.isPositiveDefinite());
  BOOST_CHECK(!p33.isPositiveSemiDefinite());
  BOOST_CHECK(p33.isSymmetric());
  BOOST_CHECK(p33.isIndefinite());
  BOOST_CHECK(!p33.isZero());
  BOOST_CHECK(p33.isTriangular());
  BOOST_CHECK(p33.isLowerTriangular());
  BOOST_CHECK(p33.isUpperTriangular());

  MatrixProperties p34(MatrixProperties::DIAGONAL, MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p34.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p34.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(!p34.isConstant());
  BOOST_CHECK(p34.isDiagonal());
  BOOST_CHECK(!p34.isIdentity());
  BOOST_CHECK(p34.isInvertible());
  BOOST_CHECK(!p34.isMinusIdentity());
  BOOST_CHECK(!p34.isMultipleOfIdentity());
  BOOST_CHECK(p34.isNegativeDefinite());
  BOOST_CHECK(p34.isNegativeSemidefinite());
  BOOST_CHECK(p34.isNonZeroIndefinite());
  BOOST_CHECK(!p34.isPositiveDefinite());
  BOOST_CHECK(!p34.isPositiveSemiDefinite());
  BOOST_CHECK(p34.isSymmetric());
  BOOST_CHECK(p34.isIndefinite());
  BOOST_CHECK(!p34.isZero());
  BOOST_CHECK(p34.isTriangular());
  BOOST_CHECK(p34.isLowerTriangular());
  BOOST_CHECK(p34.isUpperTriangular());

  MatrixProperties p35(MatrixProperties::DIAGONAL, MatrixProperties::INDEFINITE);
  BOOST_CHECK(p35.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p35.positiveness() == MatrixProperties::INDEFINITE);
  BOOST_CHECK(!p35.isConstant());
  BOOST_CHECK(p35.isDiagonal());
  BOOST_CHECK(!p35.isIdentity());
  BOOST_CHECK(!p35.isInvertible());
  BOOST_CHECK(!p35.isMinusIdentity());
  BOOST_CHECK(!p35.isMultipleOfIdentity());
  BOOST_CHECK(!p35.isNegativeDefinite());
  BOOST_CHECK(!p35.isNegativeSemidefinite());
  BOOST_CHECK(!p35.isNonZeroIndefinite());
  BOOST_CHECK(!p35.isPositiveDefinite());
  BOOST_CHECK(!p35.isPositiveSemiDefinite());
  BOOST_CHECK(p35.isSymmetric());
  BOOST_CHECK(p35.isIndefinite());
  BOOST_CHECK(!p35.isZero());
  BOOST_CHECK(p35.isTriangular());
  BOOST_CHECK(p35.isLowerTriangular());
  BOOST_CHECK(p35.isUpperTriangular());

  MatrixProperties p36(MatrixProperties::DIAGONAL, MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(p36.shape() == MatrixProperties::DIAGONAL);
  BOOST_CHECK(p36.positiveness() == MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(!p36.isConstant());
  BOOST_CHECK(p36.isDiagonal());
  BOOST_CHECK(!p36.isIdentity());
  BOOST_CHECK(p36.isInvertible());
  BOOST_CHECK(!p36.isMinusIdentity());
  BOOST_CHECK(!p36.isMultipleOfIdentity());
  BOOST_CHECK(!p36.isNegativeDefinite());
  BOOST_CHECK(!p36.isNegativeSemidefinite());
  BOOST_CHECK(p36.isNonZeroIndefinite());
  BOOST_CHECK(!p36.isPositiveDefinite());
  BOOST_CHECK(!p36.isPositiveSemiDefinite());
  BOOST_CHECK(p36.isSymmetric());
  BOOST_CHECK(p36.isIndefinite());
  BOOST_CHECK(!p36.isZero());
  BOOST_CHECK(p36.isTriangular());
  BOOST_CHECK(p36.isLowerTriangular());
  BOOST_CHECK(p36.isUpperTriangular());
  

  MatrixProperties p41(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(p41.shape() == MatrixProperties::MULTIPLE_OF_IDENTITY);
  BOOST_CHECK(p41.positiveness() == MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(!p41.isConstant());
  BOOST_CHECK(p41.isDiagonal());
  BOOST_CHECK(!p41.isIdentity());
  BOOST_CHECK(!p41.isInvertible());
  BOOST_CHECK(!p41.isMinusIdentity());
  BOOST_CHECK(p41.isMultipleOfIdentity());
  BOOST_CHECK(!p41.isNegativeDefinite());
  BOOST_CHECK(!p41.isNegativeSemidefinite());
  BOOST_CHECK(!p41.isNonZeroIndefinite());
  BOOST_CHECK(!p41.isPositiveDefinite());
  BOOST_CHECK(p41.isPositiveSemiDefinite());
  BOOST_CHECK(p41.isSymmetric());
  BOOST_CHECK(p41.isIndefinite());
  BOOST_CHECK(!p41.isZero());
  BOOST_CHECK(p41.isTriangular());
  BOOST_CHECK(p41.isLowerTriangular());
  BOOST_CHECK(p41.isUpperTriangular());

  MatrixProperties p42(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p42.shape() == MatrixProperties::MULTIPLE_OF_IDENTITY);
  BOOST_CHECK(p42.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(!p42.isConstant());
  BOOST_CHECK(p42.isDiagonal());
  BOOST_CHECK(!p42.isIdentity());
  BOOST_CHECK(p42.isInvertible());
  BOOST_CHECK(!p42.isMinusIdentity());
  BOOST_CHECK(p42.isMultipleOfIdentity());
  BOOST_CHECK(!p42.isNegativeDefinite());
  BOOST_CHECK(!p42.isNegativeSemidefinite());
  BOOST_CHECK(p42.isNonZeroIndefinite());
  BOOST_CHECK(p42.isPositiveDefinite());
  BOOST_CHECK(p42.isPositiveSemiDefinite());
  BOOST_CHECK(p42.isSymmetric());
  BOOST_CHECK(p42.isIndefinite());
  BOOST_CHECK(!p42.isZero());
  BOOST_CHECK(p42.isTriangular());
  BOOST_CHECK(p42.isLowerTriangular());
  BOOST_CHECK(p42.isUpperTriangular());

  MatrixProperties p43(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(p43.shape() == MatrixProperties::MULTIPLE_OF_IDENTITY);
  BOOST_CHECK(p43.positiveness() == MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(!p43.isConstant());
  BOOST_CHECK(p43.isDiagonal());
  BOOST_CHECK(!p43.isIdentity());
  BOOST_CHECK(!p43.isInvertible());
  BOOST_CHECK(!p43.isMinusIdentity());
  BOOST_CHECK(p43.isMultipleOfIdentity());
  BOOST_CHECK(!p43.isNegativeDefinite());
  BOOST_CHECK(p43.isNegativeSemidefinite());
  BOOST_CHECK(!p43.isNonZeroIndefinite());
  BOOST_CHECK(!p43.isPositiveDefinite());
  BOOST_CHECK(!p43.isPositiveSemiDefinite());
  BOOST_CHECK(p43.isSymmetric());
  BOOST_CHECK(p43.isIndefinite());
  BOOST_CHECK(!p43.isZero());
  BOOST_CHECK(p43.isTriangular());
  BOOST_CHECK(p43.isLowerTriangular());
  BOOST_CHECK(p43.isUpperTriangular());

  MatrixProperties p44(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p44.shape() == MatrixProperties::MULTIPLE_OF_IDENTITY);
  BOOST_CHECK(p44.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(!p44.isConstant());
  BOOST_CHECK(p44.isDiagonal());
  BOOST_CHECK(!p44.isIdentity());
  BOOST_CHECK(p44.isInvertible());
  BOOST_CHECK(!p44.isMinusIdentity());
  BOOST_CHECK(p44.isMultipleOfIdentity());
  BOOST_CHECK(p44.isNegativeDefinite());
  BOOST_CHECK(p44.isNegativeSemidefinite());
  BOOST_CHECK(p44.isNonZeroIndefinite());
  BOOST_CHECK(!p44.isPositiveDefinite());
  BOOST_CHECK(!p44.isPositiveSemiDefinite());
  BOOST_CHECK(p44.isSymmetric());
  BOOST_CHECK(p44.isIndefinite());
  BOOST_CHECK(!p44.isZero());
  BOOST_CHECK(p44.isTriangular());
  BOOST_CHECK(p44.isLowerTriangular());
  BOOST_CHECK(p44.isUpperTriangular());

  MatrixProperties p45(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::INDEFINITE);
  BOOST_CHECK(p45.shape() == MatrixProperties::MULTIPLE_OF_IDENTITY);
  BOOST_CHECK(p45.positiveness() == MatrixProperties::INDEFINITE);
  BOOST_CHECK(!p45.isConstant());
  BOOST_CHECK(p45.isDiagonal());
  BOOST_CHECK(!p45.isIdentity());
  BOOST_CHECK(!p45.isInvertible());
  BOOST_CHECK(!p45.isMinusIdentity());
  BOOST_CHECK(p45.isMultipleOfIdentity());
  BOOST_CHECK(!p45.isNegativeDefinite());
  BOOST_CHECK(!p45.isNegativeSemidefinite());
  BOOST_CHECK(!p45.isNonZeroIndefinite());
  BOOST_CHECK(!p45.isPositiveDefinite());
  BOOST_CHECK(!p45.isPositiveSemiDefinite());
  BOOST_CHECK(p45.isSymmetric());
  BOOST_CHECK(p45.isIndefinite());
  BOOST_CHECK(!p45.isZero());
  BOOST_CHECK(p45.isTriangular());
  BOOST_CHECK(p45.isLowerTriangular());
  BOOST_CHECK(p45.isUpperTriangular());

  MatrixProperties p46(MatrixProperties::MULTIPLE_OF_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(p46.shape() == MatrixProperties::MULTIPLE_OF_IDENTITY);
  BOOST_CHECK(p46.positiveness() == MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(!p46.isConstant());
  BOOST_CHECK(p46.isDiagonal());
  BOOST_CHECK(!p46.isIdentity());
  BOOST_CHECK(p46.isInvertible());
  BOOST_CHECK(!p46.isMinusIdentity());
  BOOST_CHECK(p46.isMultipleOfIdentity());
  BOOST_CHECK(!p46.isNegativeDefinite());
  BOOST_CHECK(!p46.isNegativeSemidefinite());
  BOOST_CHECK(p46.isNonZeroIndefinite());
  BOOST_CHECK(!p46.isPositiveDefinite());
  BOOST_CHECK(!p46.isPositiveSemiDefinite());
  BOOST_CHECK(p46.isSymmetric());
  BOOST_CHECK(p46.isIndefinite());
  BOOST_CHECK(!p46.isZero());
  BOOST_CHECK(p46.isTriangular());
  BOOST_CHECK(p46.isLowerTriangular());
  BOOST_CHECK(p46.isUpperTriangular());

  MatrixProperties p51(MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(p51.shape() == MatrixProperties::IDENTITY);
  BOOST_CHECK(p51.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p51.isConstant());
  BOOST_CHECK(p51.isDiagonal());
  BOOST_CHECK(p51.isIdentity());
  BOOST_CHECK(p51.isInvertible());
  BOOST_CHECK(!p51.isMinusIdentity());
  BOOST_CHECK(p51.isMultipleOfIdentity());
  BOOST_CHECK(!p51.isNegativeDefinite());
  BOOST_CHECK(!p51.isNegativeSemidefinite());
  BOOST_CHECK(p51.isNonZeroIndefinite());
  BOOST_CHECK(p51.isPositiveDefinite());
  BOOST_CHECK(p51.isPositiveSemiDefinite());
  BOOST_CHECK(p51.isSymmetric());
  BOOST_CHECK(p51.isIndefinite());
  BOOST_CHECK(!p51.isZero());
  BOOST_CHECK(p51.isTriangular());
  BOOST_CHECK(p51.isLowerTriangular());
  BOOST_CHECK(p51.isUpperTriangular());

  MatrixProperties p52(MatrixProperties::IDENTITY, MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p52.shape() == MatrixProperties::IDENTITY);
  BOOST_CHECK(p52.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p52.isConstant());
  BOOST_CHECK(p52.isDiagonal());
  BOOST_CHECK(p52.isIdentity());
  BOOST_CHECK(p52.isInvertible());
  BOOST_CHECK(!p52.isMinusIdentity());
  BOOST_CHECK(p52.isMultipleOfIdentity());
  BOOST_CHECK(!p52.isNegativeDefinite());
  BOOST_CHECK(!p52.isNegativeSemidefinite());
  BOOST_CHECK(p52.isNonZeroIndefinite());
  BOOST_CHECK(p52.isPositiveDefinite());
  BOOST_CHECK(p52.isPositiveSemiDefinite());
  BOOST_CHECK(p52.isSymmetric());
  BOOST_CHECK(p52.isIndefinite());
  BOOST_CHECK(!p52.isZero());
  BOOST_CHECK(p52.isTriangular());
  BOOST_CHECK(p52.isLowerTriangular());
  BOOST_CHECK(p52.isUpperTriangular());

  BOOST_CHECK_THROW(
    MatrixProperties p53(MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE)
    , std::runtime_error);
  BOOST_CHECK_THROW(
    MatrixProperties p54(MatrixProperties::IDENTITY, MatrixProperties::NEGATIVE_DEFINITE)
    , std::runtime_error);

  MatrixProperties p55(MatrixProperties::IDENTITY, MatrixProperties::INDEFINITE);
  BOOST_CHECK(p55.shape() == MatrixProperties::IDENTITY);
  BOOST_CHECK(p55.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p55.isConstant());
  BOOST_CHECK(p55.isDiagonal());
  BOOST_CHECK(p55.isIdentity());
  BOOST_CHECK(p55.isInvertible());
  BOOST_CHECK(!p55.isMinusIdentity());
  BOOST_CHECK(p55.isMultipleOfIdentity());
  BOOST_CHECK(!p55.isNegativeDefinite());
  BOOST_CHECK(!p55.isNegativeSemidefinite());
  BOOST_CHECK(p55.isNonZeroIndefinite());
  BOOST_CHECK(p55.isPositiveDefinite());
  BOOST_CHECK(p55.isPositiveSemiDefinite());
  BOOST_CHECK(p55.isSymmetric());
  BOOST_CHECK(p55.isIndefinite());
  BOOST_CHECK(!p55.isZero());
  BOOST_CHECK(p55.isTriangular());
  BOOST_CHECK(p55.isLowerTriangular());
  BOOST_CHECK(p55.isUpperTriangular());

  MatrixProperties p56(MatrixProperties::IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(p56.shape() == MatrixProperties::IDENTITY);
  BOOST_CHECK(p56.positiveness() == MatrixProperties::POSITIVE_DEFINITE);
  BOOST_CHECK(p56.isConstant());
  BOOST_CHECK(p56.isDiagonal());
  BOOST_CHECK(p56.isIdentity());
  BOOST_CHECK(p56.isInvertible());
  BOOST_CHECK(!p56.isMinusIdentity());
  BOOST_CHECK(p56.isMultipleOfIdentity());
  BOOST_CHECK(!p56.isNegativeDefinite());
  BOOST_CHECK(!p56.isNegativeSemidefinite());
  BOOST_CHECK(p56.isNonZeroIndefinite());
  BOOST_CHECK(p56.isPositiveDefinite());
  BOOST_CHECK(p56.isPositiveSemiDefinite());
  BOOST_CHECK(p56.isSymmetric());
  BOOST_CHECK(p56.isIndefinite());
  BOOST_CHECK(!p56.isZero());
  BOOST_CHECK(p56.isTriangular());
  BOOST_CHECK(p56.isLowerTriangular());
  BOOST_CHECK(p56.isUpperTriangular());

  BOOST_CHECK_THROW(
    MatrixProperties p61(MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_SEMIDEFINITE)
    , std::runtime_error);
  BOOST_CHECK_THROW(
    MatrixProperties p62(MatrixProperties::MINUS_IDENTITY, MatrixProperties::POSITIVE_DEFINITE)
    , std::runtime_error);
  
  
  MatrixProperties p63(MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(p63.shape() == MatrixProperties::MINUS_IDENTITY);
  BOOST_CHECK(p63.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p63.isConstant());
  BOOST_CHECK(p63.isDiagonal());
  BOOST_CHECK(!p63.isIdentity());
  BOOST_CHECK(p63.isInvertible());
  BOOST_CHECK(p63.isMinusIdentity());
  BOOST_CHECK(p63.isMultipleOfIdentity());
  BOOST_CHECK(p63.isNegativeDefinite());
  BOOST_CHECK(p63.isNegativeSemidefinite());
  BOOST_CHECK(p63.isNonZeroIndefinite());
  BOOST_CHECK(!p63.isPositiveDefinite());
  BOOST_CHECK(!p63.isPositiveSemiDefinite());
  BOOST_CHECK(p63.isSymmetric());
  BOOST_CHECK(p63.isIndefinite());
  BOOST_CHECK(!p63.isZero());
  BOOST_CHECK(p63.isTriangular());
  BOOST_CHECK(p63.isLowerTriangular());
  BOOST_CHECK(p63.isUpperTriangular());

  MatrixProperties p64(MatrixProperties::MINUS_IDENTITY, MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p64.shape() == MatrixProperties::MINUS_IDENTITY);
  BOOST_CHECK(p64.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p64.isConstant());
  BOOST_CHECK(p64.isDiagonal());
  BOOST_CHECK(!p64.isIdentity());
  BOOST_CHECK(p64.isInvertible());
  BOOST_CHECK(p64.isMinusIdentity());
  BOOST_CHECK(p64.isMultipleOfIdentity());
  BOOST_CHECK(p64.isNegativeDefinite());
  BOOST_CHECK(p64.isNegativeSemidefinite());
  BOOST_CHECK(p64.isNonZeroIndefinite());
  BOOST_CHECK(!p64.isPositiveDefinite());
  BOOST_CHECK(!p64.isPositiveSemiDefinite());
  BOOST_CHECK(p64.isSymmetric());
  BOOST_CHECK(p64.isIndefinite());
  BOOST_CHECK(!p64.isZero());
  BOOST_CHECK(p64.isTriangular());
  BOOST_CHECK(p64.isLowerTriangular());
  BOOST_CHECK(p64.isUpperTriangular());

  MatrixProperties p65(MatrixProperties::MINUS_IDENTITY, MatrixProperties::INDEFINITE);
  BOOST_CHECK(p65.shape() == MatrixProperties::MINUS_IDENTITY);
  BOOST_CHECK(p65.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p65.isConstant());
  BOOST_CHECK(p65.isDiagonal());
  BOOST_CHECK(!p65.isIdentity());
  BOOST_CHECK(p65.isInvertible());
  BOOST_CHECK(p65.isMinusIdentity());
  BOOST_CHECK(p65.isMultipleOfIdentity());
  BOOST_CHECK(p65.isNegativeDefinite());
  BOOST_CHECK(p65.isNegativeSemidefinite());
  BOOST_CHECK(p65.isNonZeroIndefinite());
  BOOST_CHECK(!p65.isPositiveDefinite());
  BOOST_CHECK(!p65.isPositiveSemiDefinite());
  BOOST_CHECK(p65.isSymmetric());
  BOOST_CHECK(p65.isIndefinite());
  BOOST_CHECK(!p65.isZero());
  BOOST_CHECK(p65.isTriangular());
  BOOST_CHECK(p65.isLowerTriangular());
  BOOST_CHECK(p65.isUpperTriangular());

  MatrixProperties p66(MatrixProperties::MINUS_IDENTITY, MatrixProperties::NON_ZERO_INDEFINITE);
  BOOST_CHECK(p66.shape() == MatrixProperties::MINUS_IDENTITY);
  BOOST_CHECK(p66.positiveness() == MatrixProperties::NEGATIVE_DEFINITE);
  BOOST_CHECK(p66.isConstant());
  BOOST_CHECK(p66.isDiagonal());
  BOOST_CHECK(!p66.isIdentity());
  BOOST_CHECK(p66.isInvertible());
  BOOST_CHECK(p66.isMinusIdentity());
  BOOST_CHECK(p66.isMultipleOfIdentity());
  BOOST_CHECK(p66.isNegativeDefinite());
  BOOST_CHECK(p66.isNegativeSemidefinite());
  BOOST_CHECK(p66.isNonZeroIndefinite());
  BOOST_CHECK(!p66.isPositiveDefinite());
  BOOST_CHECK(!p66.isPositiveSemiDefinite());
  BOOST_CHECK(p66.isSymmetric());
  BOOST_CHECK(p66.isIndefinite());
  BOOST_CHECK(!p66.isZero());
  BOOST_CHECK(p66.isTriangular());
  BOOST_CHECK(p66.isLowerTriangular());
  BOOST_CHECK(p66.isUpperTriangular());
  
  MatrixProperties p71(MatrixProperties::ZERO, MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(p71.shape() == MatrixProperties::ZERO);
  BOOST_CHECK(p71.positiveness() == MatrixProperties::POSITIVE_SEMIDEFINITE);
  BOOST_CHECK(p71.isConstant());
  BOOST_CHECK(p71.isDiagonal());
  BOOST_CHECK(!p71.isIdentity());
  BOOST_CHECK(!p71.isInvertible());
  BOOST_CHECK(!p71.isMinusIdentity());
  BOOST_CHECK(p71.isMultipleOfIdentity());
  BOOST_CHECK(!p71.isNegativeDefinite());
  BOOST_CHECK(p71.isNegativeSemidefinite());
  BOOST_CHECK(!p71.isNonZeroIndefinite());
  BOOST_CHECK(!p71.isPositiveDefinite());
  BOOST_CHECK(p71.isPositiveSemiDefinite());
  BOOST_CHECK(p71.isSymmetric());
  BOOST_CHECK(p71.isIndefinite());
  BOOST_CHECK(p71.isZero());
  BOOST_CHECK(p71.isTriangular());
  BOOST_CHECK(p71.isLowerTriangular());
  BOOST_CHECK(p71.isUpperTriangular());

  
  BOOST_CHECK_THROW(
    MatrixProperties p72(MatrixProperties::ZERO, MatrixProperties::POSITIVE_DEFINITE)
    , std::runtime_error);
  
  MatrixProperties p73(MatrixProperties::ZERO, MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(p73.shape() == MatrixProperties::ZERO);
  BOOST_CHECK(p73.positiveness() == MatrixProperties::NEGATIVE_SEMIDEFINITE);
  BOOST_CHECK(p73.isConstant());
  BOOST_CHECK(p73.isDiagonal());
  BOOST_CHECK(!p73.isIdentity());
  BOOST_CHECK(!p73.isInvertible());
  BOOST_CHECK(!p73.isMinusIdentity());
  BOOST_CHECK(p73.isMultipleOfIdentity());
  BOOST_CHECK(!p73.isNegativeDefinite());
  BOOST_CHECK(p73.isNegativeSemidefinite());
  BOOST_CHECK(!p73.isNonZeroIndefinite());
  BOOST_CHECK(!p73.isPositiveDefinite());
  BOOST_CHECK(p73.isPositiveSemiDefinite());
  BOOST_CHECK(p73.isSymmetric());
  BOOST_CHECK(p73.isIndefinite());
  BOOST_CHECK(p73.isZero());
  BOOST_CHECK(p73.isTriangular());
  BOOST_CHECK(p73.isLowerTriangular());
  BOOST_CHECK(p73.isUpperTriangular());

  BOOST_CHECK_THROW(
    MatrixProperties p74(MatrixProperties::ZERO, MatrixProperties::NEGATIVE_DEFINITE)
    , std::runtime_error);

  MatrixProperties p75(MatrixProperties::ZERO, MatrixProperties::INDEFINITE);
  BOOST_CHECK(p75.shape() == MatrixProperties::ZERO);
  BOOST_CHECK(p75.positiveness() == MatrixProperties::INDEFINITE);
  BOOST_CHECK(p75.isConstant());
  BOOST_CHECK(p75.isDiagonal());
  BOOST_CHECK(!p75.isIdentity());
  BOOST_CHECK(!p75.isInvertible());
  BOOST_CHECK(!p75.isMinusIdentity());
  BOOST_CHECK(p75.isMultipleOfIdentity());
  BOOST_CHECK(!p75.isNegativeDefinite());
  BOOST_CHECK(p75.isNegativeSemidefinite());
  BOOST_CHECK(!p75.isNonZeroIndefinite());
  BOOST_CHECK(!p75.isPositiveDefinite());
  BOOST_CHECK(p75.isPositiveSemiDefinite());
  BOOST_CHECK(p75.isSymmetric());
  BOOST_CHECK(p75.isIndefinite());
  BOOST_CHECK(p75.isZero());
  BOOST_CHECK(p75.isTriangular());
  BOOST_CHECK(p75.isLowerTriangular());
  BOOST_CHECK(p75.isUpperTriangular());

  BOOST_CHECK_THROW(
    MatrixProperties p76(MatrixProperties::ZERO, MatrixProperties::NON_ZERO_INDEFINITE)
    , std::runtime_error);
}


#define buildAndCheck(shouldThrow, ... ) \
  if (shouldThrow) {\
    BOOST_CHECK_THROW(MatrixProperties(__VA_ARGS__), std::runtime_error); \
  } else {\
    BOOST_CHECK_NO_THROW(MatrixProperties(__VA_ARGS__)); }

BOOST_AUTO_TEST_CASE(ConstnessCompatibility)
{
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

BOOST_AUTO_TEST_CASE(InvertibilityCompatibility)
{
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

BOOST_AUTO_TEST_CASE(ArgumentOrderAndRepetition)
{
  MatrixProperties::MatrixShape s = MatrixProperties::GENERAL;
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
