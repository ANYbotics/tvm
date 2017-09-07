#pragma once

#include "tvm/api.h"

namespace tvm
{
  /** This class describes some mathematical properties of a matrix.
    */
  class TVM_DLLAPI MatrixProperties
  {
  public:
    /** Shape of a matrix*/
    enum MatrixShape
    {
      GENERAL,              //general shape. This includes all other shapes.
      LOWER_TRIANGULAR,     //lower triangular matrix. This includes diagonal matrices.
      UPPER_TRIANGULAR,     //upper triangular matrix. This includes diagonal matrices.
      DIAGONAL,             //diagonal matrices. This includes multiple of the identity matrices (including zero matrix)
      MULTIPLE_OF_IDENTITY, //a*I, where a is a real number. This includes the case a=-1, a=0, and a=1.
      IDENTITY,             //identity matrix I
      MINUS_IDENTITY,       //-I
      ZERO                  //zero matrix
    };

    /** Positiveness property of the matrix. Any options other than NA implies
    * that the matrix is symmetric
    */
    enum Positiveness
    {
      NA,                     // not applicable (matrix is not symmetric) / unknown
      POSITIVE_SEMIDEFINITE,  // all eigenvalues are >=0
      POSITIVE_DEFINITE,      // all eigenvalues are >0
      NEGATIVE_SEMIDEFINITE,  // all eigenvalues are <=0
      NEGATIVE_DEFINITE,      // all eigenvalues are <0
      INDEFINITE,             // eigenvalues are a mix of positive, negative and 0
      NON_ZERO_INDEFINITE,    // eigenvalues are a mix of positive, negative but not 0
    };

    /** A wrapper over a boolean representing the constness of a matrix.*/
    class Constness
    {
    public:
      Constness(bool b = false) : b_(b) {}
      operator bool() { return b_; }
    private:
      bool b_;
    };

    /** A wrapper over a boolean representing the invertibility of a matrix.*/
    class Invertibility
    {
    public:
      Invertibility(bool b = false) : b_(b) {}
      operator bool() { return b_; }
    private:
      bool b_;
    };


    /** The data given to the constructors may be redundant. For example an 
      * identity matrix is constant, invertible and positive definite. The 
      * constructors are deducing automatically all what they can from the 
      * shape and the positiveness.
      * The constructors use user-given data when they add information to what
      * they can deduce. If the user-given data are less precise but compatible
      * with what has been deduced, they are discarded. If they are 
      * contradicting the deductions, an assertion is fired.
      *
      * Here are some examples:
      * - a multiple-of-identity matrix can only be said to be symmetric and
      * undefinite. If the user specifies it is positive-semidefinite, this 
      * will be recorded. If additionnally it is specified to be invertible, it
      * will be deduced that the matrix is positive definite.
      * - if a minus-identity matrix is said to be non-zero undefinite, this 
      * caracteristic will be discarded as it can be automatically deduced that
      * the matrix is negative definite. If it is said to be positive definite,
      * non constant or non invertible, an assertion will be fire as this
      * contradicts what can be deduced.
      * - if a matrix is triangular and symmetric, then it is diagonal.
      */
    MatrixProperties(MatrixShape shape = MatrixShape::GENERAL, Positiveness positiveness = Positiveness::NA);
    MatrixProperties(Constness constant, MatrixShape shape = MatrixShape::GENERAL, Positiveness positiveness = Positiveness::NA);
    MatrixProperties(Invertibility invertible, Constness constant, MatrixShape shape = MatrixShape::GENERAL, Positiveness positiveness = Positiveness::NA);

    MatrixShape shape() const;
    Positiveness positiveness() const;

    bool isConstant() const;
    bool isInvertible() const;
    bool isTriangular() const;
    bool isLowerTriangular() const;
    bool isUpperTriangular() const;
    bool isDiagonal() const;
    bool isMultipleOfIdentity() const;
    bool isIdentity() const;
    bool isMinusIdentity() const;
    bool isZero() const;
    bool isSymmetric() const;
    bool isPositiveSemiDefinite() const;
    bool isPositiveDefinite() const;
    bool isNegativeSemidefinite() const;
    bool isNegativeDefinite() const;
    bool isIndefinite() const;
    bool isNonZeroIndefinite() const;

  private:
    bool        constant_;
    bool        invertible_;
    MatrixShape shape_;
    bool        symmetric_;
    Positiveness positiveness_;
  };

  inline MatrixProperties::MatrixShape MatrixProperties::shape() const
  {
    return shape_;
  }

  inline MatrixProperties::Positiveness MatrixProperties::positiveness() const
  {
    return positiveness_;
  }

  inline bool MatrixProperties::isConstant() const
  {
    return constant_;
  }

  inline bool MatrixProperties::isInvertible() const
  {
    return invertible_;
  }

  inline bool MatrixProperties::isTriangular() const
  {
    return shape_ >= MatrixShape::LOWER_TRIANGULAR;
  }

  inline bool MatrixProperties::isLowerTriangular() const
  {
    return shape_ == MatrixShape::LOWER_TRIANGULAR || isDiagonal();
  }

  inline bool MatrixProperties::isUpperTriangular() const
  {
    return shape_ >= MatrixShape::UPPER_TRIANGULAR;
  }

  inline bool MatrixProperties::isDiagonal() const
  {
    return shape_ >= MatrixShape::DIAGONAL;
  }

  inline bool MatrixProperties::isMultipleOfIdentity() const
  {
    return shape_ >= MatrixShape::MULTIPLE_OF_IDENTITY;
  }

  inline bool MatrixProperties::isIdentity() const
  {
    return shape_ == MatrixShape::IDENTITY;
  }

  inline bool MatrixProperties::isMinusIdentity() const
  {
    return shape_ == MatrixShape::MINUS_IDENTITY;
  }

  inline bool MatrixProperties::isZero() const
  {
    return shape_ == MatrixShape::ZERO;
  }

  inline bool MatrixProperties::isSymmetric() const
  {
    return symmetric_;
  }

  inline bool MatrixProperties::isPositiveSemiDefinite() const
  {
    return positiveness_ == Positiveness::POSITIVE_SEMIDEFINITE 
        || isPositiveDefinite() 
        || isZero();
  }

  inline bool MatrixProperties::isPositiveDefinite() const
  {
    return positiveness_ == Positiveness::POSITIVE_DEFINITE;
  }

  inline bool MatrixProperties::isNegativeSemidefinite() const
  {
    return positiveness_ == Positiveness::NEGATIVE_SEMIDEFINITE 
        || isNegativeDefinite()
        || isZero();
  }

  inline bool MatrixProperties::isNegativeDefinite() const
  {
    return positiveness_ == Positiveness::NEGATIVE_DEFINITE;
  }

  inline bool MatrixProperties::isIndefinite() const
  {
    return positiveness_ != Positiveness::NA;
  }

  inline bool MatrixProperties::isNonZeroIndefinite() const
  {
    return positiveness_ == Positiveness::NON_ZERO_INDEFINITE
        || isPositiveDefinite()
        || isNegativeDefinite();
  }

}