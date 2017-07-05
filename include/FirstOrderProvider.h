#pragma once

#include <algorithm>
#include <map>
#include <vector>

#include <Eigen/Core>

#include  <tvm/data/Node.h>

namespace tvm
{
  class Variable;

  namespace internal
  {
    /* Describes an entity that can provide a value and its jacobian*/

    class TVM_DLLAPI FirstOrderProvider : public data::Node<FirstOrderProvider>
    {
    public:
      SET_OUTPUTS(FirstOrderProvider, Value, Jacobian)

      const Eigen::VectorXd& value() const;
      const Eigen::MatrixXd& jacobian(const Variable& x) const;

      /** Note: by default, these methods return the cached value.
      * However, they are virtual in case the user might want to bypass the cache.
      * This would be typically the case if he/she wants to directly return the
      * output of another method, e.g. return the jacobian of an other Function.
      *
      * Question: should they be made protected or stay public
      */
      virtual const Eigen::VectorXd& valueNoCheck() const;
      virtual const Eigen::MatrixXd& jacobianNoCheck(const Variable& x) const;

      /** Return the output size m*/
      int size() const;

      /** Return the variables*/
      const std::vector<std::shared_ptr<Variable>>& variables() const;

    protected:
      FirstOrderProvider(int m);

      /** Resize all cache members corresponding to active output.
        * Do not forget to call it if you override it in derived classes.
        */
      virtual void resizeCache();

      /** Add or remove variables. Cache is automatically updated*/
      void addVariable(std::shared_ptr<Variable>);
      void removeVariable(std::shared_ptr<Variable>);

      /** To be overriden by derived classes that need to react to
        * the addition of a variable. Called at the end of addVariable();
        */
      virtual void addVariable_(std::shared_ptr<Variable>);
      virtual void removeVariable_(std::shared_ptr<Variable>);

      // cache
      //FIXME should this be made private with protected accessor? 
      //In particular, we would like to be sure that elements of jacobian are
      //accessed with the at() method, not the [] operator, so as to avoid adding
      //accidentally a variable.
      Eigen::VectorXd value_;
      std::map<Variable const*, Eigen::MatrixXd> jacobian_;

    private:
      int m_; //output size
      std::vector<std::shared_ptr<Variable>> variables_;
    };
  }
}