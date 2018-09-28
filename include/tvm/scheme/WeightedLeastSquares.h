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

#include <tvm/scheme/abstract/ResolutionScheme.h>
#include <tvm/scheme/internal/Assignment.h>
#include <tvm/scheme/internal/ProblemComputationData.h>

#include <EigenQP/LSSOL_LS.h>

namespace tvm
{

namespace scheme
{
  /** This class implements the classic weighted least square scheme
    */
  class TVM_DLLAPI WeightedLeastSquares : public abstract::LinearResolutionScheme<WeightedLeastSquares>
  {
  private:
    struct Memory : public internal::ProblemComputationData
    {
      Memory(int solverId);
      void resize(int m0, int m1, double big_number);

      Eigen::MatrixXd A;
      Eigen::MatrixXd C;
      Eigen::VectorXd b;
      Eigen::VectorXd l;
      Eigen::VectorXd u;

      std::vector<internal::Assignment> assignments;
      Eigen::LSSOL_LS ls;

    protected:
      void setVariablesToSolution_(VariableVector& x) override;
    };

  public:
    using ComputationDataType = Memory;

    // FIXME temporary verbose parameter
    WeightedLeastSquares(bool verbose = true, double scalarizationWeight = 1000);

    /** Private interface for CRTP*/
    bool solve_(LinearizedControlProblem& problem, internal::ProblemComputationData* data) const;
    std::unique_ptr<Memory> createComputationData_(const LinearizedControlProblem& problem) const;

  protected:
    bool verbose_;
    double scalarizationWeight_;
  };

}  // namespace scheme

}  // namespace tvm
