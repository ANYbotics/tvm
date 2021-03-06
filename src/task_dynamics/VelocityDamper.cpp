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

#include <tvm/task_dynamics/VelocityDamper.h>

#include <tvm/function/abstract/Function.h>

namespace tvm
{

namespace task_dynamics
{
  VelocityDamperConfig::VelocityDamperConfig(double di, double ds, double xsi, double xsiOff)
    : di_(di), ds_(ds), xsi_(xsi), xsiOff_(xsiOff)
  {
    if (di_ <= ds_)
    {
      throw std::runtime_error("di needs to be greater than ds");
    }
    if (ds_ < 0)
    {
      throw std::runtime_error("ds should be non-negative.");
    }
    if (xsi_ < 0)
    {
      throw std::runtime_error("xsi should be non-negative.");
    }
    if (xsiOff_ < 0)
    {
      throw std::runtime_error("xsiOff should be positive.");
    }
  }

  VelocityDamper::VelocityDamper(const VelocityDamperConfig& config, double big)
    : dt_(0)
    , xsi_(0)
    , ds_(config.ds_)
    , di_(config.di_)
    , big_(big)
    , autoXsi_(config.xsi_==0)
  {
    if (autoXsi_)
    {
      xsi_ = config.xsiOff_;
    }
    else
    {
      xsi_ = config.xsi_;
    }
    if (big <= 0)
    {
      throw std::runtime_error("big should be non-negative.");
    }
  }

  VelocityDamper::VelocityDamper(double dt, const VelocityDamperConfig& config, double big)
    : dt_(dt)
    , xsi_(0)
    , ds_(config.ds_)
    , di_(config.di_)
    , big_(big)
    , autoXsi_(config.xsi_ == 0)
  {
    if (autoXsi_)
    {
      xsi_ = config.xsiOff_;
    }
    else
    {
      xsi_ = config.xsi_;
    }
    if (dt <= 0)
    {
      throw std::runtime_error("Time increment should be non-negative.");
    }
    if (big <= 0)
    {
      throw std::runtime_error("big should be non-negative.");
    }
  }

  std::unique_ptr<abstract::TaskDynamicsImpl> VelocityDamper::impl_(FunctionPtr f, constraint::Type t, const Eigen::VectorXd & rhs) const
  {
    if (dt_ > 0)
    {
      return std::unique_ptr<abstract::TaskDynamicsImpl>(new Impl(f, t, rhs, dt_, autoXsi_, di_, ds_, xsi_, big_));
    }
    else
    {
      return std::unique_ptr<abstract::TaskDynamicsImpl>(new Impl(f, t, rhs, autoXsi_, di_, ds_, xsi_, big_));
    }
  }

  VelocityDamper::Impl::Impl(FunctionPtr f, constraint::Type t, const Eigen::VectorXd & rhs, bool autoXsi, double di, double ds, double xsi, double big)
    : TaskDynamicsImpl(Order::One, f, t, rhs)
    , dt_(0)
    , ds_(ds)
    , di_(di)
    , xsiOff_(0)
    , a_(-1 / (di - ds))
    , big_(big)
    , autoXsi_(autoXsi)
    , d_(f->size())
    , axsi_(f->size())
    , active_(f->size(), false)
  {
    if (autoXsi)
    {
      axsi_.setOnes();
      xsiOff_ = xsi;
      addInputDependency<TaskDynamicsImpl>(Update::UpdateValue, f, function::abstract::Function::Output::Velocity);
    }
    else
    {
      axsi_.setConstant(a_*xsi);
    }
  }

  VelocityDamper::Impl::Impl(FunctionPtr f, constraint::Type t, const Eigen::VectorXd & rhs, double dt, bool autoXsi, double di, double ds, double xsi, double big)
    : TaskDynamicsImpl(Order::Two, f, t, rhs)
    , dt_(dt)
    , ds_(ds)
    , di_(di)
    , xsiOff_(xsi)
    , a_(-1 / (di - ds))
    , big_(big)
    , autoXsi_(autoXsi)
    , d_(f->size())
    , axsi_(f->size())
    , active_(f->size(), false)
  {
    if (autoXsi)
    {
      axsi_.setOnes();
      xsiOff_ = xsi;
    }
    else
    {
      axsi_.setConstant(a_*xsi);
    }
  }

  void VelocityDamper::Impl::updateValue()
  {
    if (type() == constraint::Type::LOWER_THAN)
    {
      d_ = rhs() - function().value(); //turn f<=rhs into d = rhs-f >= 0
      updateValue_(-1);
      if (order() == Order::Two)
      {
        value_ += function().velocity();
        value_ /= dt_;
      }
      value_ = (d_.array() > di_).select(big_, -value_);
    }
    else
    {
      d_ = function().value() - rhs(); //turn f>=rhs into d = f-rhs >= 0
      updateValue_(+1);
      if (order() == Order::Two)
      {
        value_ -= function().velocity();
        value_ /= dt_;
      }
      value_ = (d_.array() > di_).select(-big_, value_);
    }
  }

  void VelocityDamper::Impl::updateValue_(double s)
  {
    if (autoXsi_)
    {
      const auto& dv = function().velocity();
      for (int i = 0; i < function().size(); ++i)
      {
        if (d_[i] <= di_ && !active_[static_cast<size_t>(i)])
        {
          active_[static_cast<size_t>(i)] = true;
          axsi_[i] = a_ * (s * dv[i] * (ds_ - di_) / (d_[i] - ds_) + xsiOff_);
        }
        else if (d_[i] > di_ && active_[static_cast<size_t>(i)])
        {
          active_[static_cast<size_t>(i)] = false;
        }
      }
    }
    value_.array() = d_.array() - ds_;
    value_.array() *= axsi_.array();
  }

} // task_dynamics

} // tvm
