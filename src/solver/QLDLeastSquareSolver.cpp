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

#include <tvm/solver/QLDLeastSquareSolver.h>

#include <tvm/scheme/internal/AssignmentTarget.h>

#include <iostream>

namespace tvm
{

namespace solver
{
  QLDLeastSquareSolver::QLDLeastSquareSolver(const QLDLeastSquareOptions& options)
    : LeastSquareSolver(options.verbose().value())
    , Aineq_(A_.bottomRows(0))
    , bineq_(b_.tail(0))
    , big_number_(options.big_number().value())
    , cholesky_(options.cholesky().value())
    , choleskyDamping_(options.choleskyDamping().value())
    , eps_(options.eps().value())
    , autoMinNorm_(false)
  {
  }

  void QLDLeastSquareSolver::initializeBuild_(int m1, int me, int mi, bool)
  {
    int n = variables().totalSize();
    int m0 = me + mi;
    underspecifiedObj_ = m1 < n;
    if (cholesky_ && underspecifiedObj_)
    {
      D_.resize(m1 + n, n);
      D_.bottomRows(n).setZero();
      D_.bottomRows(n).diagonal().setConstant(choleskyDamping_);
    }
    else
      D_.resize(m1, n);
    e_.resize(m1);
    if (!cholesky_)
      Q_.resize(n, n);
    c_.resize(n);
    A_.resize(m0, n);
    b_.resize(m0);
    xl_ = Eigen::VectorXd::Constant(n, -big_number_);
    xu_ = Eigen::VectorXd::Constant(n, +big_number_);
    new(&Aineq_) MatrixXdBottom(A_.bottomRows(mi));
    new(&bineq_) VectorXdTail(b_.tail(mi));
    if (underspecifiedObj_)
      qld_.problem(n, me, mi, n+m1);
    else
      qld_.problem(n, me, mi);
    if (cholesky_)
    {
      if (underspecifiedObj_)
        new(&qr_) Eigen::HouseholderQR<Eigen::MatrixXd>(m1+n, n);
      else
        new(&qr_) Eigen::HouseholderQR<Eigen::MatrixXd>(m1, n);
    }

    autoMinNorm_ = false;
  }

  void QLDLeastSquareSolver::addBound_(LinearConstraintPtr bound, RangePtr range, bool first)
  {
    scheme::internal::AssignmentTarget target(range, xl_, xu_);
    addAssignement(bound, target, bound->variables()[0], first);
  }

  void QLDLeastSquareSolver::addEqualityConstraint_(LinearConstraintPtr cstr)
  {
    RangePtr r = std::make_shared<Range>(eqSize_, cstr->size());
    scheme::internal::AssignmentTarget target(r, A_, b_, constraint::Type::EQUAL, constraint::RHS::OPPOSITE);
    addAssignement(cstr, nullptr, target, variables(), *substitutions());
  }

  void QLDLeastSquareSolver::addIneqalityConstraint_(LinearConstraintPtr cstr)
  {
    RangePtr r = std::make_shared<Range>(ineqSize_, constraintSize(cstr));
    scheme::internal::AssignmentTarget target(r, Aineq_, bineq_, constraint::Type::GREATER_THAN, constraint::RHS::OPPOSITE);
    addAssignement(cstr, nullptr, target, variables(), *substitutions());
  }

  void QLDLeastSquareSolver::addObjective_(LinearConstraintPtr cstr, SolvingRequirementsPtr req, double additionalWeight)
  {
    RangePtr r = std::make_shared<Range>(objSize_, cstr->size());
    scheme::internal::AssignmentTarget target(r, D_, e_, constraint::Type::EQUAL, constraint::RHS::OPPOSITE);
    addAssignement(cstr, req, target, variables(), *substitutions(), additionalWeight);
  }

  void QLDLeastSquareSolver::setMinimumNorm_()
  {
    autoMinNorm_ = true;
    Q_.setIdentity();
    c_.setZero();
  }

  void QLDLeastSquareSolver::preAssignmentProcess_()
  {
  }

  void QLDLeastSquareSolver::postAssignmentProcess_()
  {
    if (!autoMinNorm_)
    {
      c_.noalias() = D_.topRows(m1_).transpose() * e_;

      if (cholesky_)
      {
        qr_.compute(D_);
      }
      else
      {
        Q_.noalias() = D_.transpose() * D_;   //TODO check if this can be optimized: QLD might need only half the matrix
      }
    }
  }

  bool QLDLeastSquareSolver::solve_()
  {
    if (cholesky_ && !autoMinNorm_)
    {
      int n = variables().totalSize();
      return qld_.solve(qr_.matrixQR().topRows(n), c_,
                        A_, b_,
                        xl_, xu_,
                        me_,
                        true, eps_);
    }
    else
    {
      return qld_.solve(Q_, c_,
                        A_, b_,
                        xl_, xu_,
                        me_,
                        false, eps_);
    }
  }

  const Eigen::VectorXd& QLDLeastSquareSolver::result_() const
  {
    return qld_.result();
  }

  void QLDLeastSquareSolver::printProblemData_() const
  {
    if (cholesky_)
    {
      int n = variables().totalSize();
      std::cout << "R =\n" << qr_.matrixQR().topRows(n).template triangularView<Eigen::Upper>().toDenseMatrix() << std::endl;
    }
    else
      std::cout << "`Q =\n" << Q_ << std::endl;
    std::cout << "c = " << c_.transpose() << std::endl;
    std::cout << "A =\n" << A_ << std::endl;
    std::cout << "b = " << b_.transpose() << std::endl;
    std::cout << "xl = " << xl_.transpose() << std::endl;
    std::cout << "xu = " << xu_.transpose() << std::endl;
  }

  void QLDLeastSquareSolver::printDiagnostic_() const
  {
    std::cout << "QLD fail code = " << qld_.fail() << " (0 is success)" << std::endl;
  }

  QLDLeastSquareConfiguration::QLDLeastSquareConfiguration(const QLDLeastSquareOptions& options)
    : LeastSquareConfiguration("qld")
    , options_(options)
  {
  }

  std::unique_ptr<abstract::LeastSquareSolver> QLDLeastSquareConfiguration::createSolver() const
  {
    return std::make_unique<QLDLeastSquareSolver>(options_);
  }

}

}