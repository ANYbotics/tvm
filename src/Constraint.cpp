#include "Constraint.h"
#include "errors.h"

namespace tvm
{
  ConstraintBase::ConstraintBase(int m)
    : FirstOrderProvider(m)
  {
  }

  Constraint::Constraint(ConstraintType ct, RHSType rt, int m)
    : data::OutputSelector<ConstraintBase>(m)
    , cstrType_(ct)
    , rhsType_(rt)
    , usel_((ct == ConstraintType::GREATER_THAN || ct == ConstraintType::DOUBLE_SIDED) && rt != RHSType::ZERO)
    , useu_((ct == ConstraintType::LOWER_THAN || ct == ConstraintType::DOUBLE_SIDED) && rt != RHSType::ZERO)
    , usee_(ct == ConstraintType::EQUAL && rt != RHSType::ZERO)
  {
    if (ct == ConstraintType::DOUBLE_SIDED && rt == RHSType::ZERO)
      throw std::runtime_error("The combination (ConstraintType::DOUBLE_SIDED, RHSType::ZERO) is forbidden. Please use (ConstraintType::EQUAL, RHSType::ZERO) instead.");
    //FIXME: we make the choice here to have no "rhs" output when the RHSType is zero. 
    //An alternative is to use and set to zero the relevant vectors, but then we need 
    //to prevent a derived class to change their value.
    resizeCache();
    if (!usel_)
      disableOutput(Output::L);
    if (!useu_)
      disableOutput(Output::U);
    if (!usee_)
      disableOutput(Output::E);
  }

  const Eigen::VectorXd& Constraint::l() const
  {
    if (isOutputEnabled(Output::L))
      return lNoCheck();
    else
      throw UnusedOutput(/*description*/); //TODO add description of the error
  }

  const Eigen::VectorXd& Constraint::u() const
  {
    if (isOutputEnabled(Output::U))
      return uNoCheck();
    else
      throw UnusedOutput(/*description*/); //TODO add description of the error
  }

  const Eigen::VectorXd& Constraint::e() const
  {
    if (isOutputEnabled(Output::E))
      return eNoCheck();
    else
      throw UnusedOutput(/*description*/); //TODO add description of the error
  }

  const Eigen::VectorXd& Constraint::lNoCheck() const
  {
    return l_;
  }

  const Eigen::VectorXd& Constraint::uNoCheck() const
  {
    return u_;
  }

  const Eigen::VectorXd& Constraint::eNoCheck() const
  {
    return e_;
  }

  void Constraint::resizeCache()
  {
    if (usel_)
      l_.resize(size());

    if (useu_)
      u_.resize(size());

    if (usee_)
      e_.resize(size());
  }

  ConstraintType Constraint::constraintType() const
  {
    return cstrType_;
  }

  RHSType Constraint::rhsType() const
  {
    return rhsType_;
  }
}
