/* Copyright 2017-2018 CNRS-AIST JRL and CNRS-UM LIRMM
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

#include <tvm/Variable.h>
#include <tvm/VariableVector.h>
#include <tvm/defs.h>
#include <tvm/graph/abstract/Node.h>
#include <tvm/internal/MatrixWithProperties.h>
#include <tvm/utils/internal/map.h>

#include <Eigen/Core>

#include <algorithm>
#include <map>
#include <vector>

namespace tvm
{

namespace internal
{

/** Describes an entity that can provide a value and its jacobian
 *
 * \dot
 * digraph "update graph" {
 *   rankdir="LR";
 *   {
 *     rank = same;
 *     node [shape=hexagon];
 *     Value; Jacobian;
 *   }
 *   {
 *     rank = same;
 *     node [style=invis, label=""];
 *     outValue; outJacobian;
 *   }
 *   Value -> outValue [label="value()"];
 *   Jacobian -> outJacobian [label="jacobian(x_i)"];
 * }
 * \enddot
 */
class TVM_DLLAPI FirstOrderProvider : public graph::abstract::Node<FirstOrderProvider>
{
public:
  SET_OUTPUTS(FirstOrderProvider, Value, Jacobian)

  /** \internal by default, these methods return the cached value.
   * However, they are virtual in case the user might want to bypass the cache.
   * This would be typically the case if he/she wants to directly return the
   * output of another method, e.g. return the jacobian of an other Function.
   */
  /** Return the value of this entity*/
  virtual const Eigen::VectorXd & value() const;
  /** Return the jacobian matrix of this entity corresponding to the variable
   * \p x
   */
  virtual const MatrixWithProperties & jacobian(const Variable & x) const;

  /** Linearity w.r.t \p x*/
  bool linearIn(const Variable & x) const;

  const Space & imageSpace() const;

  /** Return the image space size.*/
  int size() const;
  /** Size of the output value (representation size of the image space).*/
  int rSize() const;
  /** Size of the tangent space to the image space (or equivalently row size of the jacobian matrices).*/
  int tSize() const;

  /** Return the variables*/
  const VariableVector & variables() const;

protected:
  /** Constructor for a function/constraint with value in \f$ \mathbb{R}^m \f$.
   *
   * \param m the size of the function/constraint image space, i.e. the row
   * size of the jacobians (or equivalently in this case the size of the
   * output value).
   */
  FirstOrderProvider(int m);

  /** Constructor for a function/constraint with value in a specified space.
   *
   * \param image Description of the image space
   */
  FirstOrderProvider(Space image);

  /** Resize all cache members corresponding to active outputs.
   *
   * This can be overriden in case you do not need all of the default
   * mechanism (typically if you will not use part of the cache).
   * If you override to perform additional operations, do not forget to
   * call this base version in the derived classes.
   */
  virtual void resizeCache();

  /** Sub-methods of resizeCache resizing the value cache vector (if used).
   * To be used by derived classes that need this level of granularity.
   */
  void resizeValueCache();
  /** Sub-methods of resizeCache resizing the jacobian cache matrices (if
   * used).
   * To be used by derived classes that need this level of granularity.
   */
  void resizeJacobianCache();

  /** Add a variable. Cache is automatically updated.
   * \param v The variable to add
   * \param linear Specify that the entity is depending linearly on the
   * variable or not.
   */
  void addVariable(VariablePtr v, bool linear);
  /** Remove variable \p v. Cache is automatically updated. */
  void removeVariable(VariablePtr v);

  /** Add a variable vector.
   *
   * Convenience function similar to adding each elements of the vector individually with the same linear parameter
   *
   * \see addVariable(VariablePtr, bool)
   *
   */
  void addVariable(const VariableVector & v, bool linear);

  /** To be overriden by derived classes that need to react to
   * the addition of a variable. Called at the end of addVariable();
   */
  virtual void addVariable_(VariablePtr);

  /** To be overriden by derived classes that need to react to
   * the removal of a variable. Called at the end of removeVariable();
   */
  virtual void removeVariable_(VariablePtr);

  /** Split a jacobian matrix J into its components Ji corresponding to the
   * provided variables.
   *
   * \param J The matrix to be split
   * \param vars The vector of variables giving the layout of J. It is the
   * user's responsibility to ensure these variables are part of variables_
   * and that J has the correct size.
   * \param keepProperties If true, the properties associated with matrices
   * Ji are kept, if not they are reset to default.
   */
  void splitJacobian(const MatrixConstRef & J, const std::vector<VariablePtr> & vars, bool keepProperties = false);

  /** Overload for VariableVector operations */
  inline void splitJacobian(const MatrixConstRef & J, const VariableVector & vars, bool keepProperties = false)
  {
    splitJacobian(J, vars.variables(), keepProperties);
  }

  // cache
  Eigen::VectorXd value_;
  utils::internal::map<Variable const *, MatrixWithProperties> jacobian_;

protected:
  /** Resize the function */
  void resize(int m);

  Space imageSpace_; // output space
  VariableVector variables_;
  utils::internal::map<Variable const *, bool> linear_;
};

inline const Eigen::VectorXd & FirstOrderProvider::value() const { return value_; }

inline const MatrixWithProperties & FirstOrderProvider::jacobian(const Variable & x) const { return jacobian_.at(&x); }

inline bool FirstOrderProvider::linearIn(const Variable & x) const { return linear_.at(&x); }

inline const Space & FirstOrderProvider::imageSpace() const { return imageSpace_; }

inline int FirstOrderProvider::size() const { return imageSpace_.size(); }

inline int FirstOrderProvider::rSize() const { return imageSpace_.rSize(); }
inline int FirstOrderProvider::tSize() const { return imageSpace_.tSize(); }

inline const VariableVector & FirstOrderProvider::variables() const { return variables_; }

} // namespace internal

} // namespace tvm
