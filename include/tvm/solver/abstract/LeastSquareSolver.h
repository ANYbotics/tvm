/* Copyright 2017-2020 CNRS-AIST JRL and CNRS-UM LIRMM
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <tvm/api.h>
#include <tvm/defs.h>
#include <tvm/hint/internal/Substitutions.h>
#include <tvm/scheme/internal/Assignment.h>
#include <tvm/solver/internal/Option.h>
#include <tvm/utils/internal/map.h>

namespace tvm
{

namespace solver
{

namespace abstract
{
  /** Base class for a (constrained) least-square solver. 
    *
    * The problem to be solved has the general form
    * min. ||Ax-b||^2
    * s.t.      C_e x = d
    *      l <= C_i x <= u
    *         xl <= x <= xu
    *
    * where l or u might be set to -inf or +inf, and the explicit bounds are
    * optionnal.
    *
    * When deriving this class, also remember to derive the factory class
    * LeastSquareConfiguration as well.
    */
  class TVM_DLLAPI LeastSquareSolver
  {
  public:
    LeastSquareSolver(bool verbose = false);
    LeastSquareSolver(const LeastSquareSolver&) = delete;
    LeastSquareSolver& operator=(const LeastSquareSolver&) = delete;
    virtual ~LeastSquareSolver() = default;
    /** Open a build sequence for a problem on the current variables (set 
      * through the inherited ProblemComputationData::addVariable) with the
      * specified dimensions, allocating the memory needed.
      *
      * \param x The variables of the problem. The object need to be valid until ::finalizeBuild is called.
      * \param nObj Row size of A.
      * \param nEq Row size of C_e.
      * \param nIneq Row size of C_i.
      * \param useBounds Presence of explicit bounds in the problem.
      * \param subs Possible substitutions used for solving.
      *
      * Once a build is started, objective, constraints and bounds can be added
      * through ::addObjective, ::addConstraint and ::addBound, until
      * ::finalizeBuild is called.
      */
    void startBuild(const VariableVector& x, int nObj, int nEq, int nIneq, bool useBounds = true, const hint::internal::Substitutions& subs = {});
    /** Finalize the build.*/
    void finalizeBuild();

    /** Add a bound constraint to the solver. If multiple bounds appears on the
      * same variable, their intersection is taken.
      */
    void addBound(LinearConstraintPtr bound);

    /** Add a constraint to the solver. */
    void addConstraint(LinearConstraintPtr cstr);

    /** Add an objective to the solver with given requirements
      * \param obj The linear expression added in least-square form
      * \param req The solving requirements. Only the weight-related requirements
      *   will be taken into account
      * \param additionalWeight An additional factor that will multiply the other weights.
      */
    void addObjective(LinearConstraintPtr obj, const SolvingRequirementsPtr req, double additionalWeight = 1);
    
    /** Set ||x||^2 as the least square objective of the problem.
      * \warning this replace previously added objectives.
      */
    void setMinimumNorm();
    
    /** Solve the problem
      * \return true upon success of the resolution.
      */
    bool solve();

    /** Get the result of the previous call to solve()*/
    const Eigen::VectorXd& result() const;

    /** Return the constraint size for the solver. This can be different from
      * the actual constraint size if the constraint is a double-sided inequality
      * but the solver only accept simple sided constraints
      */
    int constraintSize(const LinearConstraintPtr& c) const;

  protected:
    virtual void initializeBuild_(int nObj, int nEq, int nIneq, bool useBounds) = 0;
    virtual void addBound_(LinearConstraintPtr bound, RangePtr range, bool first) = 0;
    virtual void addEqualityConstraint_(LinearConstraintPtr cstr) = 0;
    virtual void addIneqalityConstraint_(LinearConstraintPtr cstr) = 0;
    virtual void addObjective_(LinearConstraintPtr obj, SolvingRequirementsPtr req, double additionalWeight=1) = 0;
    virtual void setMinimumNorm_() = 0;
    virtual void preAssignmentProcess_() {}
    virtual void postAssignmentProcess_() {}
    virtual bool solve_() = 0;
    virtual const Eigen::VectorXd& result_() const = 0;
    virtual bool handleDoubleSidedConstraint_() const = 0;

    virtual void printProblemData_() const = 0;
    virtual void printDiagnostic_() const = 0;

    const VariableVector& variables() const { return *variables_; }
    const hint::internal::Substitutions* substitutions() const { return subs_; }

    template<typename ... Args>
    void addAssignement(Args&& ... args);

  public:
    template<typename K, typename T> using map = utils::internal::map<K, T>;
    using AssignmentVector = std::vector<scheme::internal::Assignment>;
    using AssignmentPtrVector = std::vector<scheme::internal::Assignment*>;
    using MapToAssignment = map<constraint::abstract::LinearConstraint*, AssignmentPtrVector>;

  protected:
    int nEq_;
    int nIneq_;
    int nObj_;
    int objSize_;
    int eqSize_;
    int ineqSize_;

  private:
    bool buildInProgress_;
    bool verbose_;
    VariableVector const* variables_;
    /** Used to track if this is the first time bounds are applied to a given variable. */
    map<Variable*, bool> first_;
    /** List of assignments used for assembling the problem data. */
    std::vector<scheme::internal::Assignment> assignments_;
    /** Keeping tracks of which assignments are associated to a constraint. 
      * \todo most of the times, there will be a single assignment per constraint.
      * This would be a good place to use small vector-like container.
      */
    MapToAssignment objectiveToAssigments_;
    MapToAssignment constraintToAssigments_;
    MapToAssignment boundToAssigments_;
    const hint::internal::Substitutions* subs_;
  };


  /** A base class for LeastSquareSolver factory.
   *
   * The goal of this class is to be passed to a resolution scheme to specify
   * its underlying solver.
   */
  class TVM_DLLAPI LeastSquareConfiguration
  {
  protected:
    LeastSquareConfiguration(const std::string& solverName) 
      : solverName_(solverName) {}

  public:
    virtual ~LeastSquareConfiguration() = default;

    virtual std::unique_ptr<LeastSquareConfiguration> clone() const = 0;
    virtual std::unique_ptr<LeastSquareSolver> createSolver() const = 0;

  private:
    std::string solverName_;
  };


  template<typename ...Args>
  inline void LeastSquareSolver::addAssignement(Args&& ... args)
  {
    assignments_.emplace_back(std::forward<Args>(args)...);
  }

}

}

}