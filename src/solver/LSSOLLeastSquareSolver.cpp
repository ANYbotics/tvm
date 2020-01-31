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

#include <tvm/solver/LSSOLLeastSquareSolver.h>

#include <tvm/scheme/internal/AssignmentTarget.h>

namespace tvm
{

namespace solver
{
  LSSOLLeastSquareSolver::LSSOLLeastSquareSolver(double big_number)
    : LeastSquareSolver()
    , cl_(l_.tail(0))
    , cu_(u_.tail(0))
    , big_number_(big_number)
  {
  }

  void LSSOLLeastSquareSolver::initializeBuild_(int m1, int me, int mi, bool useBounds)
  {
    int n = variables().totalSize();
    int m0 = me + mi;
    A_.resize(m1, n);
    A_.setZero();
    C_.resize(m0, n);
    C_.setZero();
    b_.resize(m1);
    b_.setZero();
    l_ = Eigen::VectorXd::Constant(m0 + n, -big_number_);
    u_ = Eigen::VectorXd::Constant(m0 + n, +big_number_);
    cl_ = l_.tail(m0);
    cu_ = u_.tail(m0);
    ls_.resize(n, m0, Eigen::lssol::eType::LS1);
  }

  void LSSOLLeastSquareSolver::addBound_(LinearConstraintPtr bound, RangePtr range, bool first)
  {
    scheme::internal::AssignmentTarget target(range, l_, u_);
    addAssignement(bound, target, bound->variables()[0], first);
  }

  void LSSOLLeastSquareSolver::addEqualityConstraint_(LinearConstraintPtr cstr)
  {
    RangePtr r = std::make_shared<Range>(eqSize_+ineqSize_, cstr->size());
    scheme::internal::AssignmentTarget target(r, C_, cl_, cu_, constraint::RHS::AS_GIVEN);
    addAssignement(cstr, nullptr, target, variables(), *substitutions());
  }

  void LSSOLLeastSquareSolver::addIneqalityConstraint_(LinearConstraintPtr cstr)
  {
    RangePtr r = std::make_shared<Range>(eqSize_ + ineqSize_, cstr->size());
    scheme::internal::AssignmentTarget target(r, C_, cl_, cu_, constraint::RHS::AS_GIVEN);
    addAssignement(cstr, nullptr, target, variables(), *substitutions());
  }

  void LSSOLLeastSquareSolver::addObjective_(LinearConstraintPtr cstr, SolvingRequirementsPtr req, double additionalWeight)
  {
    RangePtr r = std::make_shared<Range>(objSize_, cstr->size());
    scheme::internal::AssignmentTarget target(r, A_, b_, constraint::Type::EQUAL, constraint::RHS::AS_GIVEN);
    addAssignement(cstr, req, target, variables(), *substitutions(), additionalWeight);
  }

  bool LSSOLLeastSquareSolver::solve_()
  {
    return ls_.solve(A_, b_, C_, l_, u_);
  }


  LSSOLLeastSquareSolverConfiguration::LSSOLLeastSquareSolverConfiguration(double big_number)
    : LeastSquareSolverConfiguration("lssol")
    , big_number_(big_number)
  {
  }
  
  std::unique_ptr<abstract::LeastSquareSolver> LSSOLLeastSquareSolverConfiguration::createSolver() const
  {
    return std::make_unique<LSSOLLeastSquareSolver>(big_number_);
  }
}

}