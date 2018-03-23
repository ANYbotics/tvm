#pragma once

/* Copyright 2017 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is part of TVM.
 *
 * TVM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * TVM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with TVM.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <tvm/api.h>
#include <tvm/defs.h>
#include <tvm/Variable.h> // Range
#include <tvm/constraint/abstract/LinearConstraint.h>
#include <tvm/hint/internal/Substitutions.h>
#include <tvm/requirements/SolvingRequirements.h>
#include <tvm/scheme/internal/AssignmentTarget.h>
#include <tvm/scheme/internal/CompiledAssignmentWrapper.h>

#include <Eigen/Core>

#include <memory>
#include <type_traits>
#include <vector>

namespace tvm
{

namespace scheme
{

namespace internal
{

  /** A class whose role is to assign efficiently the matrix and vector(s) of a
    * LinearConstraint to a part of matrix and vector(s) specified by a
    * ResolutionScheme and a mapping of variables. This is done while taking
    * into account the possible convention differences between the constraint
    * and the scheme, as well as the requirements on the constraint.
    */
  class TVM_DLLAPI Assignment
  {
  public:
    /** Assignment constructor
      * \param source The linear constraints whose matrix and vector(s) will be
      * assigned.
      * \param req Solving requirements attached to this constraint.
      * \param target The target of the assignment.
      * \param variables The vector of variables corresponding to the target.
      * It must be such that its total dimension is equal to the column size of
      * the target matrix.
      * \param scalarizationWeight An additional scalar weight to apply on the
      * constraint, used by the solver to emulate priority.
      */
    Assignment(LinearConstraintPtr source, 
               std::shared_ptr<requirements::SolvingRequirements> req,
               const AssignmentTarget& target, 
               const VariableVector& variables, 
               const hint::internal::Substitutions& substitutions = {},
               double scalarizationWeight = 1);

    /** Version for bounds
      * \param first wether this is the first assignement of bounds for this
      * variable (first assignment just copy vectors while the following ones
      * need to perform min/max operations).
      */
    Assignment(LinearConstraintPtr source, const AssignmentTarget& target, const VariablePtr& variables, bool first);

    /** To be called when the source has been resized*/
    void onUpdatedSource();
    /** To be called when the target has been resized and/or range has changed*/
    void onUpdatedTarget();
    /** To be called when the variables change.*/
    void onUpdatedMapping(const VariableVector& variables);
    /** Change the weight of constraint. TODO: how to specify the constraint?*/
    void weight(double alpha);
    void weight(const Eigen::VectorXd& w);

    /** Perform the assignment.*/
    void run();

    static double big_;

  private:
    /** Pointer type to a method of LinearConstraint returning a vector.
      * It is used to make a selection between e(), l() and u().
      */
    using RHSFunction = const Eigen::VectorXd& (constraint::abstract::LinearConstraint::*)() const;
   
    /** Pointer type to a method of AssignmentTarget returning a matrix block.
      * It is used to make a selection between A(), AFirstHalf() and ASecondHalf().
      */
    using MatrixFunction = MatrixRef(AssignmentTarget::*)(int, int) const;

    /** Pointer type to a method of AssignementTarget returning a vector segment.
      * It is used to make a selection between b(), bFirstHalf(), bSecondHalf(),
      * l() and u().
      */
    using VectorFunction = VectorRef(AssignmentTarget::*)() const;

    /** A structure grouping a matrix assignment and some of the elements that
      * defined it.
      */
    struct MatrixAssignment
    {
      CompiledAssignmentWrapper<Eigen::MatrixXd> assignment;
      Variable* x;
      Range colRange;
      MatrixFunction getTargetMatrix;
    };

    /** A structure grouping a vector assignment and some of the elements that
    * defined it.
    */
    struct VectorAssignment
    {
      CompiledAssignmentWrapper<Eigen::VectorXd> assignment;
      bool useSource;
      RHSFunction getSourceVector;
      VectorFunction getTargetVector;
    };

    struct VectorSubstitutionAssignement
    {
      CompiledAssignmentWrapper<Eigen::VectorXd> assignment;
      VectorFunction getTargetVector;
    };

    /** Check that the convention and size of the target are compatible with the
      * convention and size of the source.
      */
    void checkTarget(bool bound = false);

    void checkSource(bool bound);

    /** Generates the assignments for the general case.
      * \param variables the set of variables for the problem.
      */
    void build(const VariableVector& variables);
    
    /** Generates the assignments for the bound case.
      * \param variables the set of variables for the problem.
      * \param first true if this is the first assignment for the bounds (first
      * assignment makes copy, the following perform min/max
      */
    void build(const VariablePtr& variable, bool first);
    
    /** Build internal data from the requirements*/
    void processRequirements();
    
    /** Creates a matrix assigment from the jacobian of \p source_ corresponding
      * to variable \p x to the block of matrix described by \p M and \p range.
      * \p flip indicates a sign change if \p true.
      */
    void addMatrixAssignment(Variable& x, MatrixFunction M, const Range& range, bool flip);
    
    /** Creates the assignements due to susbtituing the variable \p x by the
      * linear expression given by \p sub. The target is given by \p M.
      * \p flip indicates a sign change if \p true.
      */
    void addMatrixSubstitutionAssignments(const VariableVector& variables, Variable& x, MatrixFunction M, 
                                          const function::BasicLinearFunction& sub, bool flip);
    
    /** Creates and assigment between the vector given by \p f and the one given
      * by \p v, taking care of the RHS conventions for the source and the
      * target. The assignement type is given by the template parameter \p A.
      * \p flip indicates a sign change if \p true.
      */
    template<AssignType A = AssignType::COPY>
    void addVectorAssignment(RHSFunction f, VectorFunction v, bool flip);
    
    /** Creates the assignements due to the substitution of variable \p x by the
      * linear expression \p sub. The target is given by \p v.
      */
    void addVectorSubstitutionAssignments(const function::BasicLinearFunction& sub, 
                                          VectorFunction v, Variable& x, bool flip);
    
    /** Create a vector assignement where the source is a constant. The target
      * is given by \p v and the type of assignement by \p A.
      */
    template<AssignType A = AssignType::COPY>
    void addConstantAssignment(double d, VectorFunction v);
    
    /** Creates an assignement setting to zero the matrix block given by \p M
      * and \p range. The variable \p x is simply stored in the corresponding
      * \p MatrixAssignment.
      */
    void addZeroAssignment(Variable& x, MatrixFunction M, const Range& range);
    
    /** Calls addAssignments(const VariableVector& variables, MatrixFunction M,
      * RHSFunction f1, VectorFunction v1, RHSFunction f2, VectorFunction v2, 
      * bool flip) for a single-sided case.
      */
    void addAssignments(const VariableVector& variables, MatrixFunction M,
                        RHSFunction f, VectorFunction v, bool flip);

    /** Calls addAssignments(const VariableVector& variables, MatrixFunction M,
      * RHSFunction f1, VectorFunction v1, RHSFunction f2, VectorFunction v2, 
      * bool flip) for a double-sided case.
      */
    void addAssignments(const VariableVector& variables, MatrixFunction M,
                        RHSFunction f1, VectorFunction v1, 
                        RHSFunction f2, VectorFunction v2);

    /** Creates all the matrix assignements between the source and the target,
      * as well as the vector assignements described by \p f1 and \p v1 and
      * optionnally by \p f2 and \p v2 if those are not \p nullptr.
      * This method is called after the constraint::Type conventions of the
      * source and the target have been processed (resulting in the choice of
      * \p M, \p f1, \p v1, \p f2, \p v2 and \p flip). It handles internally the
      * substitutions.
      */
    void addAssignments(const VariableVector& variables, MatrixFunction M,
                        RHSFunction f1, VectorFunction v1, 
                        RHSFunction f2, VectorFunction v2, bool flip);

    /** Create the bounds assignments. It handles the dispatch due to the source
      * Type convention.
      */
    template<AssignType A1 = AssignType::COPY, AssignType A2 = AssignType::COPY>
    void assignBounds(VectorFunction l, VectorFunction u, bool flip);

    /** Create the compiled assignment between \p from and \p to, taking into
      * account the requirements and the possible sign flip indicated by 
      * \p flip.
      */
    template<typename T, AssignType A, typename U>
    CompiledAssignmentWrapper<T> createAssignment(const U& from, const Eigen::Ref<T>& to, bool flip = false);

    /** Create the compiled substitution assignement to = Mult * from (vector 
      * case) or to = from * mult (matrix case) taking into account the 
      * requirements and \p flip
      */
    template<typename T, AssignType A, typename U, typename V>
    CompiledAssignmentWrapper<T> createSubstitutionAssignment(const U& from, const Eigen::Ref<T>& to, const V& Mult, bool flip = false);

    /** The source of the assignment.*/
    LinearConstraintPtr source_;
    /** The target of the assignment.*/
    AssignmentTarget target_;
    /** The weight used to emulate hierarchy in a weight scheme.*/
    double scalarizationWeight_;
    /** The requirements attached to the source.*/
    std::shared_ptr<requirements::SolvingRequirements> requirements_;
    /** All the assignements that are setting the initial values of the targeted blocks*/
    std::vector<MatrixAssignment> matrixAssignments_;
    /** All assignments due to substitution. We separe them from matrixAssignments_
      * because these assignements add to existing values, and we need to be sure
      * that the assignements in matrixAssignments_ have been carried out before.
      */
    std::vector<MatrixAssignment> matrixSubstitutionAssignments_;
    /** All the initial rhs assignments*/
    std::vector<VectorAssignment> vectorAssignments_;
    /** The additional rhs assignments due to substitutions. As for matrix
      * assignments, they need to be carried out after those of vectorAssignments_.
      */
    std::vector<VectorSubstitutionAssignement> vectorSubstitutionAssignments_;

    /** Processed requirements*/
    double scalarWeight_;
    Eigen::VectorXd anisotropicWeight_;
    Eigen::VectorXd minusAnisotropicWeight_;

    /** Data for substitutions */
    VariableVector substitutedVariables_;
    std::vector<std::shared_ptr<function::BasicLinearFunction>> variableSubstitutions_;

    /** Temporary vectors for bound assignements*/
    Eigen::VectorXd tmpl_;
    Eigen::VectorXd tmpl2_;
    Eigen::VectorXd tmpu_;
    Eigen::VectorXd tmpu2_;
  };

  template<AssignType A1, AssignType A2>
  inline void Assignment::assignBounds(VectorFunction l, VectorFunction u, bool flip)
  {
    using constraint::abstract::LinearConstraint;
    switch (source_->type())
    {
    case constraint::Type::EQUAL:
      addVectorAssignment<A1>(&LinearConstraint::e, l, flip);
      addVectorAssignment<A2>(&LinearConstraint::e, u, flip);
      break;
    case constraint::Type::GREATER_THAN:
      addVectorAssignment<A1>(&LinearConstraint::l, l, flip);
      addConstantAssignment<A2>(flip ? -big_ : +big_, u);
      break;
    case constraint::Type::LOWER_THAN:
      addConstantAssignment<A1>(flip ? +big_ : -big_, l);
      addVectorAssignment<A2>(&LinearConstraint::u, u, flip);
      break;
    case constraint::Type::DOUBLE_SIDED:
      addVectorAssignment<A1>(&LinearConstraint::l, l, flip);
      addVectorAssignment<A2>(&LinearConstraint::u, u, flip);
      break;
    }
  }

  template<typename T, AssignType A, typename U>
  inline CompiledAssignmentWrapper<T> Assignment::createAssignment(const U& from, const Eigen::Ref<T>& to, bool flip)
  {
    using Wrapper = CompiledAssignmentWrapper<typename std::conditional<std::is_arithmetic<U>::value, Eigen::VectorXd, T>::type>;
    const Source F = std::is_arithmetic<U>::value ? CONSTANT : EXTERNAL;

    if (requirements_->anisotropicWeight().isDefault())
    {
      if (scalarWeight_ == 1)
      {
        if (flip)
          return Wrapper::template make<A, MINUS, IDENTITY, F>(to, from);
        else
          return Wrapper::template make<A, NONE, IDENTITY, F>(to, from);
      }
      else
      {
        if (flip)
          return Wrapper::template make<A, SCALAR, IDENTITY, F>(to, from, -scalarWeight_);
        else
          return Wrapper::template make<A, SCALAR, IDENTITY, F>(to, from, scalarWeight_);
      }
    }
    else
    {
      if (flip)
        return Wrapper::template make<A, DIAGONAL, IDENTITY, F>(to, from, minusAnisotropicWeight_);
      else
        return Wrapper::template make<A, DIAGONAL, IDENTITY, F>(to, from, anisotropicWeight_);
    }
  }

  template<typename T, AssignType A, typename U, typename V>
  inline CompiledAssignmentWrapper<T> Assignment::createSubstitutionAssignment(const U& from, const Eigen::Ref<T>& to, const V& Mult, bool flip)
  {
    using Wrapper = CompiledAssignmentWrapper<typename std::conditional<std::is_arithmetic<U>::value, Eigen::VectorXd, T>::type>;
    const Source F = std::is_arithmetic<U>::value ? CONSTANT : EXTERNAL;
    const MatrixMult M = GENERAL; //FIXME have a swith on Mult for detecting CUSTOM case

    if (requirements_->anisotropicWeight().isDefault())
    {
      if (scalarWeight_ == 1)
      {
        if (flip)
          return Wrapper::template make<A, MINUS, M, F>(to, from, Mult);
        else
          return Wrapper::template make<A, NONE, M, F>(to, from, Mult);
      }
      else
      {
        if (flip)
          return Wrapper::template make<A, SCALAR, M, F>(to, from, -scalarWeight_, Mult);
        else
          return Wrapper::template make<A, SCALAR, M, F>(to, from, scalarWeight_, Mult);
      }
    }
    else
    {
      if (flip)
        return Wrapper::template make<A, DIAGONAL, M, F>(to, from, minusAnisotropicWeight_, Mult);
      else
        return Wrapper::template make<A, DIAGONAL, M, F>(to, from, anisotropicWeight_, Mult);
    }
  }

  template<AssignType A>
  inline void Assignment::addVectorAssignment(RHSFunction f, VectorFunction v, bool flip)
  {
    bool useSource = source_->rhs() != constraint::RHS::ZERO;
    if (useSource)
    {
      // So far, the sign flip has been deduced only from the ConstraintType of the source
      // and the target. Now we need to take into account the constraint::RHS as well.
      if (source_->rhs() == constraint::RHS::OPPOSITE)
        flip = !flip;
      if (target_.constraintRhs() == constraint::RHS::OPPOSITE)
        flip = !flip;

      const VectorRef& to = (target_.*v)();
      const VectorConstRef& from = (source_.get()->*f)();
      auto w = createAssignment<Eigen::VectorXd, A>(from, to, flip);
      vectorAssignments_.push_back({ w, true, f, v });
    }
    else
    {
      if (target_.constraintRhs() != constraint::RHS::ZERO)
      {
        const VectorRef& to = (target_.*v)();
        auto w = CompiledAssignmentWrapper<Eigen::VectorXd>::make<A, NONE, IDENTITY, ZERO>(to);
        vectorAssignments_.push_back({ w, false, nullptr, v });
      }
    }
  }

  template<AssignType A>
  inline void Assignment::addConstantAssignment(double d, VectorFunction v)
  {
    const VectorRef& to = (target_.*v)();
    auto w = createAssignment<Eigen::VectorXd, A>(d, to, false);
    vectorAssignments_.push_back({ w, false, nullptr, v });
  }



}  // namespace internal

}  // namespace scheme

}  // namespace tvm
