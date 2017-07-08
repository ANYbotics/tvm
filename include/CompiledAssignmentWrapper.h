#pragma once

#include <functional>

#include <Eigen/Core>

#include "CompiledAssignment.h"

namespace tvm
{
  namespace utils
  {
    template<typename MatrixType>
    struct CompiledAssignmentWrapper
    {
      CompiledAssignmentWrapper() {
        std::cout << "default constructor" << std::endl;
      }
      CompiledAssignmentWrapper(const CompiledAssignmentWrapper<MatrixType>& other);
      CompiledAssignmentWrapper(CompiledAssignmentWrapper<MatrixType>&& other);
      CompiledAssignmentWrapper<MatrixType>& operator=(const CompiledAssignmentWrapper<MatrixType>& other);
      CompiledAssignmentWrapper<MatrixType>& operator=(CompiledAssignmentWrapper<MatrixType>&& other);
      ~CompiledAssignmentWrapper();

      template<AssignType A, ScalarMult S, MatrixMult M, MultPos P>
      static CompiledAssignmentWrapper make(const Eigen::Ref<const MatrixType>& from, const Eigen::Ref<MatrixType>& to,
        double s = 1, const typename MatrixMultBase<M, P>::MultType* const m = nullptr);

      template<AssignType A>
      static CompiledAssignmentWrapper makeNoFrom(const Eigen::Ref<MatrixType>& to);

      std::function<void()> run;
      std::function<void(const Eigen::Ref<const MatrixType>&)> setFrom;
      std::function<void(const Eigen::Ref<MatrixType>&)> setTo;

    private:
      template<AssignType A, ScalarMult S, MatrixMult M, MultPos P>
      void construct(const Eigen::Ref<const MatrixType>& from, const Eigen::Ref<MatrixType>& to,
        double s, const typename MatrixMultBase<M, P>::MultType* const m);

      template<AssignType A, ScalarMult S, MatrixMult M, MultPos P>
      void construct(const Eigen::Ref<MatrixType>& to);

      template<AssignType A, ScalarMult S, MatrixMult M, MultPos P, bool UseFrom>
      void constructFunctions();

      /** We store the wrapped CompiledAssignment as a void*. This allows us to
        * have a non templated wrapper.
        * We of course need to keep trace of the real type. This is done through
        * the various std::function member of the class.
        */
      void* compiledAssignment_ = nullptr;

      std::function<void()> delete_;
      std::function<void(CompiledAssignmentWrapper*)> copy_;
      std::function<void(CompiledAssignmentWrapper*)> move_;
      std::function<void(CompiledAssignmentWrapper*)> build_;
    };

    template<typename MatrixType>
    inline CompiledAssignmentWrapper<MatrixType>::CompiledAssignmentWrapper(const CompiledAssignmentWrapper<MatrixType>& other)
    {
      std::cout << "copy constructor" << std::endl;
      other.copy_(this);
      this->build_ = other.build_;
      other.build_(this);
    }

    template<typename MatrixType>
    inline CompiledAssignmentWrapper<MatrixType>::CompiledAssignmentWrapper(CompiledAssignmentWrapper<MatrixType>&& other)
    {
      std::cout << "move constructor" << std::endl;
      other.move_(this);
      this->build_ = other.build_;
      other.build_(this);
    }

    template<typename MatrixType>
    inline CompiledAssignmentWrapper<MatrixType>& CompiledAssignmentWrapper<MatrixType>::operator=(const CompiledAssignmentWrapper<MatrixType>& other)
    {
      std::cout << "assign operator" << std::endl;

      if (&other == this) { return *this; }
      delete_();
      other.copy_(this);
      other.build_(this);
      return *this;

    }
    template<typename MatrixType>
    inline CompiledAssignmentWrapper<MatrixType>& CompiledAssignmentWrapper<MatrixType>::operator=(CompiledAssignmentWrapper<MatrixType>&& other)
    {
      std::cout << "move assign operator" << std::endl;
      
      if (&other == this) { return *this; }
      delete_();
      other.move_(this);
      other.build_(this);
      return *this;
    }

    template<typename MatrixType>
    inline CompiledAssignmentWrapper<MatrixType>::~CompiledAssignmentWrapper()
    {
      delete_();
    }

    template<typename MatrixType>
    template<AssignType A, ScalarMult S, MatrixMult M, MultPos P>
    inline CompiledAssignmentWrapper<MatrixType> CompiledAssignmentWrapper<MatrixType>::make(const Eigen::Ref<const MatrixType>& from, const Eigen::Ref<MatrixType>& to, double s, const typename MatrixMultBase<M, P>::MultType * const m)
    {
      CompiledAssignmentWrapper<MatrixType> wrapper;
      wrapper.construct<A, S, M, P>(from, to, s, m);
      return wrapper;
    }

    template<typename MatrixType>
    template<AssignType A>
    inline CompiledAssignmentWrapper<MatrixType> CompiledAssignmentWrapper<MatrixType>::makeNoFrom(const Eigen::Ref<MatrixType>& to)
    {
      CompiledAssignmentWrapper<MatrixType> wrapper;
      wrapper.construct<A, NONE, IDENTITY, PRE>(to);
      return wrapper;
    }

    template<typename MatrixType>
    template<AssignType A, ScalarMult S, MatrixMult M, MultPos P>
    inline void CompiledAssignmentWrapper<MatrixType>::construct(const Eigen::Ref<const MatrixType>& from, const Eigen::Ref<MatrixType>& to,
      double s, const typename MatrixMultBase<M, P>::MultType* const m)
    {
      compiledAssignment_ = new CompiledAssignment<MatrixType, A, S, M, P, true>(from, to, s, m);
      constructFunctions<A, S, M, P, true>();
      build_(this);
    }

    template<typename MatrixType>
    template<AssignType A, ScalarMult S, MatrixMult M, MultPos P>
    inline void CompiledAssignmentWrapper<MatrixType>::construct(const Eigen::Ref<MatrixType>& to)
    {
      compiledAssignment_ = new CompiledAssignment<MatrixType, A, S, M, P, false>(to);
      constructFunctions<A, S, M, P, false>();
      build_(this);
    }

    template<typename MatrixType>
    template<AssignType A, ScalarMult S, MatrixMult M, MultPos P, bool UseFrom>
    inline void CompiledAssignmentWrapper<MatrixType>::constructFunctions()
    {
      typedef CompiledAssignment<MatrixType, A, S, M, P, UseFrom> CA;
      typedef CompiledAssignmentWrapper<MatrixType> Wrapper;

      build_ = [](Wrapper* wrapper)
      {
        wrapper->delete_ = [wrapper]()
        {
          std::cout << "delete lambda" << std::endl;
          delete static_cast<CA*>(wrapper->compiledAssignment_);
        };
        wrapper->copy_ = [wrapper](Wrapper* other)
        {
          std::cout << "copy lambda" << std::endl;
          other->compiledAssignment_ = static_cast<void*>(new CA(*(static_cast<CA*>(wrapper->compiledAssignment_))));
        };
        wrapper->move_ = [wrapper](Wrapper* other)
        {
          std::cout << "move lambda" << std::endl;
          other->compiledAssignment_ = wrapper->compiledAssignment_;
          wrapper->compiledAssignment_ = nullptr;
        };

        wrapper->run = [wrapper]()
        {
          static_cast<CA*>(wrapper->compiledAssignment_)->run();
        };
        wrapper->setFrom = [wrapper](const Eigen::Ref<const MatrixType>& from)
        {
          static_cast<CA*>(wrapper->compiledAssignment_)->setFrom(from);
        };
        wrapper->setTo = [wrapper](const Eigen::Ref<MatrixType>& to)
        {
          static_cast<CA*>(wrapper->compiledAssignment_)->setTo(to);
        };
      };
    }
  } // utils
} // tvm