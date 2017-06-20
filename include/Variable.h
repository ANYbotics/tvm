#pragma once

#include <Eigen/Core>

#include <memory>
#include <vector>
#include <string>

#include "DataGraph.h"
#include "tvm_api.h"

namespace taskvm
{
  class Variable;

  class TVM_API Space
  {
  public:
    Space(int size);
    Space(int size, int representationSize);
    Space(int size, int representationSize, int tangentRepresentationSize);

    std::unique_ptr<Variable> createVariable(const std::string& name) const;

    int size() const;
    int rSize() const;
    int tSize() const;

  private:
    int mSize_;   //size of this space (as a manifold)
    int rSize_;   //size of a vector representing a point in this space
    int tSize_;   //size of a vector representing a velocity in this space
  };

  std::shared_ptr<Variable> TVM_API dot(std::shared_ptr<Variable> var, int ndiff=1);

  class TVM_API Variable: public DataSource
  {
  public:
    enum Output {Value};

    const std::string& name() const;
    int size() const;
    const Space& space() const;
    const Eigen::VectorXd& value() const;
    int derivativeNumber() const;
    bool isBasePrimitive() const;
    std::shared_ptr<Variable> primitive() const;
    std::shared_ptr<Variable> basePrimitive() const;

  protected:

  private:
    /** Constructor for a new variable */
    Variable(const Space& s, const std::string& name);

    /** Constructor for the derivative of var */
    Variable(std::shared_ptr<Variable> var);


    /** name */
    std::string name_;

    /** data of the space from which the variable was created */
    Space space_;

    /** Value of the variable */
    Eigen::VectorXd value_;

    /** Number of derivation since the base primitive. 0 a variable which is 
      * not a derivative.
      */
    int derivativeNumber_;

    /** If the variable is the time derivative of another one, primitive_ is a 
      * reference to the latter, otherwise it is uninitialized.
      */
    std::shared_ptr<Variable> primitive_;

    /** If the variable has a time derivative, keep a pointer on it */
    std::weak_ptr<Variable> derivative_;


    /** friendship declaration */
    friend class TVM_API Space;
    friend std::shared_ptr<Variable> TVM_API dot(std::shared_ptr<Variable>, int);
  };
}