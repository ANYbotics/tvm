/* Copyright 2017-2020 CNRS-AIST JRL and CNRS-UM LIRMM */

#pragma once

#include <tvm/api.h>
#include <tvm/defs.h>

#include <vector>

namespace tvm::solver::internal
{
  /** An agregation of all the events to be handled by a solver*/
  class SolverEvents
  {
  public:
    struct WeightEvent
    {
      constraint::abstract::LinearConstraint* c;
      bool scalar;
      bool vector;
    };

    struct Objective
    {
      LinearConstraintPtr c;
      SolvingRequirementsPtr req;
      double scalarizationWeight;
    };


    void addScalarWeigthEvent(constraint::abstract::LinearConstraint* c);
    void addVectorWeigthEvent(constraint::abstract::LinearConstraint* c);

    const std::vector<WeightEvent>& weightEvents() const { return weightEvents_; }


    std::vector<LinearConstraintPtr> addedConstraints_;
    std::vector<LinearConstraintPtr> addedBounds_;
    std::vector<Objective> addedObjectives_;
    std::vector<LinearConstraintPtr> removedConstraints_;
    std::vector<LinearConstraintPtr> removedBounds_;
    std::vector<LinearConstraintPtr> removedObjectives_;

    std::vector<VariablePtr> addedVariables_;
    std::vector<VariablePtr> removedVariables_;

  private:
    /** \internal We don't anticipate to have many events at the same time so 
      * that searching in the vector will be fast. If it was not the case, we can
      * change the data structure, or add one to speed up search.*/
    std::vector<WeightEvent> weightEvents_;
  };

  inline void SolverEvents::addScalarWeigthEvent(constraint::abstract::LinearConstraint* c)
  {
    for (auto& e : weightEvents_)
    {
      if (e.c == c)
      {
        e.scalar = true;
        return;
      }
    }

    weightEvents_.push_back({ c, true, false });
  }

  inline void SolverEvents::addVectorWeigthEvent(constraint::abstract::LinearConstraint* c)
  {
    for (auto& e : weightEvents_)
    {
      if (e.c == c)
      {
        e.vector = true;
        return;
      }
    }

    weightEvents_.push_back({ c, false, true });
  }
}