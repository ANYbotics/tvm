#pragma once

#include <memory>

#include <Eigen/Core>

namespace tvm
{
  //forward declarations
  namespace constraint
  {
    namespace abstract
    {
      class Constraint;
      class LinearConstraint;
    }
  }
  namespace function
  {
    namespace abstract
    {
      class Function;
    }
  }
  namespace requirements
  {
    class SolvingRequirements;
  }
  namespace task_dynamics
  {
    namespace abstract
    {
      class TaskDynamics;
    }
  }
  class Range;
  class Variable;
  class VariableVector;

  //definitions
  using MatrixConstRef = Eigen::Ref<const Eigen::MatrixXd>;
  using MatrixRef = Eigen::Ref<Eigen::MatrixXd>;
  using VectorConstRef = Eigen::Ref<const Eigen::VectorXd>;
  using VectorRef = Eigen::Ref<Eigen::VectorXd>;

  using MatrixPtr = std::shared_ptr<Eigen::MatrixXd>;
  using VectorPtr = std::shared_ptr<Eigen::VectorXd>;

  using ConstraintPtr = std::shared_ptr<constraint::abstract::Constraint>;
  using FunctionPtr = std::shared_ptr<function::abstract::Function>;
  using LinearConstraintPtr = std::shared_ptr<constraint::abstract::LinearConstraint>;
  using RangePtr = std::shared_ptr<Range>;
  using SolvingRequirementsPtr = std::shared_ptr<requirements::SolvingRequirements>;
  using TaskDynamicsPtr = std::shared_ptr<task_dynamics::abstract::TaskDynamics>;
  using VariablePtr = std::shared_ptr<Variable>;
}  // namespace tvm
