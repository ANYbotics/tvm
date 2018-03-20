#include <tvm/LinearizedControlProblem.h>

#include <tvm/constraint/internal/LinearizedTaskConstraint.h>
#include <tvm/scheme/internal/helpers.h>

namespace tvm
{
  LinearizedControlProblem::LinearizedControlProblem()
  {
  }

  LinearizedControlProblem::LinearizedControlProblem(const ControlProblem& pb)
  {
    for (auto tr : pb.tasks())
      add(tr);
  }

  TaskWithRequirementsPtr LinearizedControlProblem::add(const Task& task, const requirements::SolvingRequirements& req)
  {
    auto tr = std::make_shared<TaskWithRequirements>(task, req);
    add(tr);
    return tr;
  }

  void LinearizedControlProblem::add(TaskWithRequirementsPtr tr)
  {
    ControlProblem::add(tr);

    //FIXME A lot of work can be done here based on the properties of the task's jacobian.
    //In particular, we could detect bounds, pairs of tasks forming a double-sided constraints...
    LinearConstraintWithRequirements lcr;
    lcr.constraint = std::make_shared<constraint::internal::LinearizedTaskConstraint>(tr->task);
    //we use the aliasing constructor of std::shared_ptr to ensure that
    //lcr.requirements points to and doesn't outlive tr->requirements.
    lcr.requirements = std::shared_ptr<requirements::SolvingRequirements>(tr, &tr->requirements);
    lcr.bound = scheme::internal::isBound(lcr.constraint);
    constraints_[tr.get()] = lcr;

    using CstrOutput = internal::FirstOrderProvider::Output;
    updater_.addInput(lcr.constraint, CstrOutput::Jacobian);
    switch (tr->task.type())
    {
    case constraint::Type::EQUAL: updater_.addInput(lcr.constraint, constraint::abstract::Constraint::Output::E); break;
    case constraint::Type::GREATER_THAN: updater_.addInput(lcr.constraint, constraint::abstract::Constraint::Output::L); break;
    case constraint::Type::LOWER_THAN: updater_.addInput(lcr.constraint, constraint::abstract::Constraint::Output::U); break;
    case constraint::Type::DOUBLE_SIDED: updater_.addInput(lcr.constraint, constraint::abstract::Constraint::Output::L, constraint::abstract::Constraint::Output::U); break;
    }
  }

  void LinearizedControlProblem::remove(TaskWithRequirements* tr)
  {
    ControlProblem::remove(tr);
    // if the above line did not throw, tr exists in the problem and in bounds_ or constraints_
    updater_.removeInput(constraints_[tr].constraint.get());
    constraints_.erase(tr);
  }

  void LinearizedControlProblem::add(const hint::Substitution & s)
  {
    substitutions_.add(s);
  }

  const hint::internal::Substitutions & LinearizedControlProblem::substitutions() const
  {
    return substitutions_;
  }

  std::vector<LinearConstraintWithRequirements> LinearizedControlProblem::constraints() const
  {
    std::vector<LinearConstraintWithRequirements> constraints;
    constraints.reserve(constraints_.size());
    for (auto c: constraints_)
      constraints.push_back(c.second);

    return constraints;
  }

  LinearConstraintPtr LinearizedControlProblem::constraint(TaskWithRequirements * t) const
  {
    return constraints_.at(t).constraint;
  }

  void LinearizedControlProblem::update()
  {
    updater_.refresh();
    updater_.run();
    substitutions_.updateSubstitutions();
  }

  LinearizedControlProblem::Updater::Updater()
    : upToDate_(false)
    , inputs_(new graph::internal::Inputs)
  {
  }

  void LinearizedControlProblem::Updater::refresh()
  {
    if (!upToDate_)
    {
      updateGraph_.clear();
      updateGraph_.add(inputs_);
      updateGraph_.update();
      upToDate_ = true;
    }
  }

  void LinearizedControlProblem::Updater::run()
  {
    updateGraph_.execute();
  }
}  // namespace tvm
