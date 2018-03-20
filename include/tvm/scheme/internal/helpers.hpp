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

namespace tvm
{
namespace scheme
{
namespace internal
{
  class ProblemComputationData;

  template<typename Problem, typename Scheme>
  inline ProblemComputationData& getComputationData(Problem& problem, const Scheme& resolutionScheme)
  {
    auto id = resolutionScheme.id();
    auto it = problem.computationData_.find(id);
    if (it != problem.computationData_.end())
    {
      return *(it->second);
    }
    else
    {
      problem.finalize();
      auto p = problem.computationData_.insert(std::move(std::make_pair(id, resolutionScheme.createComputationData(problem))));
      return *(p.first->second);
    }
  }

}
}
}