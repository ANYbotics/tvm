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

#include "AbstractNode.h"

namespace tvm
{

namespace data
{

/** Node is the concrete base for AbstractNode.
 *
 * It represents an entity that uses input signals to compute output
 * signals using update functions and the dependencies between these
 * quantitites.
 *
 */
template<typename T>
class Node : public AbstractNode
{
protected:
  /** Register updates
   *
   * An update is formed by its unique id (from an enum type) and the
   * related object method.
   *
   */
  template<typename EnumT, typename U, typename ... Args>
  void registerUpdates(EnumT u, void(U::*fn)(), Args ... args);

  /** Register a single update */
  template<typename EnumT, typename U>
  void registerUpdates(EnumT u, void(U::*fn)());

  /** Add a dependency of an output to an update call
   *
   * It is not possible for an output to have a direct and some update
   * dependencies at the same time.
   *
   */
  template<typename U = T, typename EnumO, typename EnumU>
  void addOutputDependency(EnumO o, EnumU u);

  /** Add a dependency of multiple outputs to an update call */
  template<typename U = T, typename EnumO, typename EnumU>
  void addOutputDependency(std::initializer_list<EnumO> os, EnumU u);

  /** Add a dependency between two update calls.
   *
   * The first argument of this function depends on the second argument.
   *
   */
  template<typename U = T, typename EnumU1, typename EnumU2>
  void addInternalDependency(EnumU1 uDependent, EnumU2 u);

  /** Add a dependency of an update function to an input signal and its source */
  template<typename U = T, typename EnumU, typename S, typename EnumO>
  void addInputDependency(EnumU u, std::shared_ptr<S> source, EnumO i);

  /** Add a dependency of an update function to multiple input signals from a source */
  template<typename U = T, typename EnumU, typename S, typename EnumO, typename ... Args>
  void addInputDependency(EnumU u, std::shared_ptr<S> source, EnumO i, Args ... args);

  /** Add a dependency of an output to an input signal and its source
   *
   * This is used when an output directly use the input signal without
   * requiring an update, and a cache. This is the case when an output directly
   * forward an input, and it should be the only use-case.
   *
   * It is not possible for an output to have a direct and some update
   * dependencies at the same time.
   *
   */
  template<typename U = T, typename EnumO, typename S, typename EnumI>
  void addDirectDependency(EnumO o, std::shared_ptr<S> source, EnumI i);
};

} // namespace data

} // namespace tvm

#include "Node.hpp"
