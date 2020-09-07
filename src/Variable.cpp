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

#include <tvm/Variable.h>

#include <tvm/VariableVector.h>

#include <sstream>

namespace tvm
{

  VariablePtr dot(VariablePtr var, int ndiff)
  {
    assert(ndiff > 0 && "you cannot derive less than 1 time.");
    int i;
    Variable* derivative = var.get();

    //find the ndiff-th derivative of var, or the largest i such that the i-th
    //derivative exists.
    for (i = 0; i < ndiff; ++i)
    {
      if (!derivative->derivative_)
        break;
      else
        derivative = derivative->derivative_.get();
    }

    if (i == ndiff)                 //the ndiff-th derivative already exists
      return { var, derivative };
    else                            //we need to create the derivatives from i+1 to ndiff
    {
      for (; i < ndiff; ++i)
      {
        auto primitive = derivative;
        primitive->derivative_.reset(new Variable(derivative));
        derivative = primitive->derivative_.get();
      }
      return { var, derivative };
    }
  }



  VariablePtr Variable::duplicate(const std::string& n) const
  {
    VariablePtr newPrimitive;
    if (n.empty())
    {
      newPrimitive = space_.createVariable(basePrimitive()->name() + "'");
    }
    else
    {
      newPrimitive = space_.createVariable(n);
    }
    if (derivativeNumber_)
    {
      auto r = dot(newPrimitive, derivativeNumber_);
      r->value(value_);
      return r;
    }
    else
    {
      newPrimitive->value(value_);
      return newPrimitive;
    }
  }

  const std::string & Variable::name() const
  {
    return name_;
  }

  int Variable::size() const
  {
    return static_cast<int>(value_.size());
  }

  const Space & Variable::space() const
  {
    return space_;
  }

  bool Variable::isEuclidean() const
  {
    return space_.isEuclidean() || !isBasePrimitive();
  }

  const Eigen::VectorXd & Variable::value() const
  {
    return value_;
  }

  void Variable::value(const VectorConstRef& x)
  {
    if (x.size() == size())
      value_ = x;
    else
      throw std::runtime_error("x has not the correct size.");
  }

  void Variable::setZero()
  {
    value_.setZero();
  }

  int Variable::derivativeNumber() const
  {
    return derivativeNumber_;
  }

  bool Variable::isDerivativeOf(const Variable & v) const
  {
    return basePrimitive() == v.basePrimitive() && derivativeNumber_ > v.derivativeNumber();
  }

  bool Variable::isPrimitiveOf(const Variable & v) const
  {
    return basePrimitive() == v.basePrimitive() && derivativeNumber_ < v.derivativeNumber();
  }

  bool Variable::isBasePrimitive() const
  {
    return derivativeNumber_ == 0;
  }

  VariablePtr Variable::basePrimitive() const
  {
    if (isBasePrimitive())
    {
      // while it does not seem ideal to cast the constness away, it is coherent
      // with the general case: from a const derived variable, we can get a non
      // const primitive. This is equivalent to have primitive_ be a shared_ptr
      // to this in the case of a base primitive, what we can obviously not do.
      return const_cast<Variable*>(this)->shared_from_this();
    }
    else
    {
      const Variable* ptr = this;
      for (int i = 0; i < derivativeNumber_ - 1; ++i)
        ptr = ptr->primitive_;

      return ptr->primitive_->shared_from_this();
    }
  }

  Range Variable::getMappingIn(const VariableVector& variables) const
  {
    if (mappingHelper_.stamp == variables.stamp())
      return{ mappingHelper_.start, size() };
    else
    {
      if (variables.contains(*this))
      {
        variables.computeMapping();
        return{ mappingHelper_.start, size() };
      }
      else
        throw std::runtime_error("This variable is not part of the vector of variables.");
    }
  }

  Variable::Variable(const Space & s, const std::string & name)
    : name_(name)
    , space_(s)
    , value_(s.rSize())
    , derivativeNumber_(0)
    , primitive_(nullptr)
    , derivative_(nullptr)
    , mappingHelper_()
  {
    value_.setZero();
  }

  Variable::Variable(Variable* var)
    : space_(var->space_)
    , value_(var->space_.tSize())
    , derivativeNumber_(var->derivativeNumber_ + 1)
    , primitive_(var)
    , derivative_(nullptr)
    , mappingHelper_()
  {
    std::stringstream ss;
    if (derivativeNumber_ == 1)
    {
      ss << "d " << basePrimitive()->name_ << " / dt";
    }
    else
    {
      ss << "d" << derivativeNumber_ << " " << basePrimitive()->name_ << " / dt" << derivativeNumber_;
    }
    name_ = ss.str();
    value_.setZero();
  }

}  // namespace tvm
