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

#include <tvm/solver/abstract/LeastSquareSolver.h>

#include <eigen-lssol/LSSOL_LS.h>

namespace tvm
{

namespace solver
{
  class LSSOLLSSolverFactory;

  /** A set of options for LSSOLLeastSquareSolver */
  class TVM_DLLAPI LSSOLLSSolverOptions
  {
    TVM_ADD_NON_DEFAULT_OPTION  (big_number,          constant::big_number)
    TVM_ADD_DEFAULT_OPTION      (crashTol,            double)
    TVM_ADD_DEFAULT_OPTION      (feasibilityMaxIter,  int)
    TVM_ADD_NON_DEFAULT_OPTION  (feasibilityTol,      1e-6)
    TVM_ADD_DEFAULT_OPTION      (infiniteBnd,         double)
    TVM_ADD_DEFAULT_OPTION      (infiniteStep,        double)
    TVM_ADD_DEFAULT_OPTION      (optimalityMaxIter,   int)
    TVM_ADD_DEFAULT_OPTION      (persistence,         bool)
    TVM_ADD_DEFAULT_OPTION      (printLevel,          int)
    TVM_ADD_DEFAULT_OPTION      (rankTol,             double)
    TVM_ADD_NON_DEFAULT_OPTION  (verbose,             false)
    TVM_ADD_NON_DEFAULT_OPTION  (warm,                true)

  public:
    using Factory = LSSOLLSSolverFactory;
  };


  /** An encapsulation of the LSSOL solver, to solve linear least-squares problems. */
  class TVM_DLLAPI LSSOLLeastSquareSolver : public abstract::LeastSquareSolver
  {
  public:
    LSSOLLeastSquareSolver(const LSSOLLSSolverOptions& options = {});

  protected:
    void initializeBuild_(int nObj, int nEq, int nIneq, bool useBounds) override;
    void addBound_(LinearConstraintPtr bound, RangePtr range, bool first) override;
    void addEqualityConstraint_(LinearConstraintPtr cstr) override;
    void addIneqalityConstraint_(LinearConstraintPtr cstr) override;
    void addObjective_(LinearConstraintPtr cstr, SolvingRequirementsPtr req, double additionalWeight) override;
    void setMinimumNorm_() override;
    void preAssignmentProcess_() override;
    void postAssignmentProcess_() override;
    bool solve_() override;
    virtual const Eigen::VectorXd& result_() const override;
    bool handleDoubleSidedConstraint_() const override { return true; }

    void printProblemData_() const override;
    void printDiagnostic_() const override;

  private:
    using VectorXdTail = decltype(Eigen::VectorXd().tail(1));

    Eigen::MatrixXd A_;
    Eigen::MatrixXd C_;
    Eigen::VectorXd b_;
    Eigen::VectorXd l_;
    Eigen::VectorXd u_;

    VectorXdTail    cl_;   // part of l_ corresponding to general constraints
    VectorXdTail    cu_;   // part of u_ corresponding to general constraints

    Eigen::LSSOL_LS ls_;

    bool autoMinNorm_;
    double big_number_;
  };

  
  /** A factory class to create LSSOLLeastSquareSolver instances with a given
    * set of options.
    */
  class TVM_DLLAPI LSSOLLSSolverFactory : public abstract::LSSolverFactory
  {
  public:
    /** Creation of a configuration from a set of options*/
    LSSOLLSSolverFactory(const LSSOLLSSolverOptions& options = {});

    std::unique_ptr<abstract::LSSolverFactory> clone() const override;
    std::unique_ptr<abstract::LeastSquareSolver> createSolver() const override;

  private:
    LSSOLLSSolverOptions options_;
  };

}

}