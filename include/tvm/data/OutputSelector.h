#pragma once

#include <stdexcept>

#include <tvm/data/Outputs.h>


namespace tvm
{
  namespace data
  {
    /** A structure holding the members used in OutputSelector if add = true,
      * and nothing otherwise.
      */
    template<bool add = true>
    struct SelectorMembers
    {
      /** Used to store and test whether an output is dynamically enabled or not.*/
      std::vector<bool> dynamicallyEnabled_;
      /** track if the enabling/disabling of outputs is locked or not*/
      bool locked_ = false;
    };

    template<>
    struct SelectorMembers<false>
    {

    };


    //forward declaration
    template <typename T> class OutputSelector;


    /** detail functions, see is_output_selector*/
    template< typename T>
    std::true_type is_output_selector_impl(OutputSelector<T> const volatile&);

    /** detail functions, see is_output_selector*/
    std::false_type is_output_selector_impl(...);

    /** Check at compile-time if T derives from OutputSelector
      *
      * We cannot use std::is_base_of because OutputSelector is a template class.
      * Instead, we rely on (the declaration only of the two overloads of 
      * is_output_selector_impl. If (and only if T derives from OutputSelector 
      * the overload returning std::true_type will be selected, and thus the 
      * following function returns true.
      */
    template <typename T>
    constexpr bool is_output_selector() {
      return decltype(is_output_selector_impl(std::declval<T>()))::value;
    }

    /** This class adds to its template argument the capability to enable or
      * disable some of its outputs.
      *
      * We use here a bit of template metaprogramming for taking care of the
      * following problem:
      * imagine we have the following inheritance structure
      * class A
      * class B : public OutputSelector<A>
      * class C : public B
      * class D : public OutputSelector<C>
      * We want D to hold a single vector of bool for tracking enabled outputs
      * and a single bool for lock. In our example, this means that 
      * OutputSelector<A> holds the data, and OutputSelector<C> reuse the same
      * data. We implement this by checking if the template parameter is a
      * derived class of OutputSelector. If it is not (case of A in our example)
      * we make OutputSelector inherit from SelectorMembers<true> so that it 
      * inherits the data. If it is (case of C), we make OutputSelector inherit
      * from SelectorMembers<true> so that no data is added.
      */
    template <typename OutputProvider>
    class OutputSelector : public OutputProvider, protected SelectorMembers<!is_output_selector<OutputProvider>()>
    {
    public:
      /** Constructor. Simply forward the arguments for constructing OutputProvider*/
      template <typename ... Args>
      OutputSelector(Args&&... args)
        : OutputProvider(std::forward<Args>(args)...)
      {
        static_assert(std::is_base_of<Outputs, OutputProvider>::value, "Cannot build for a type that is not derived of Outputs");
        dynamicallyEnabled_.resize(OutputProvider::OutputSize, true);
      }

      /** Lock the outputs, preventing them to be enabled or disabled. */
      void lock() { locked_ = true; }

      /** Unlock the outputs, allowing them to be enabled or disabled. */
      void unlock() { locked_ = false; }

      /** Check if the outputs are locked. */
      bool isLocked() const { return locked_; }

    protected:
      /** Disable an output. The enum must refer to an existing output. */
      template<typename EnumT>
      void disableOutput(EnumT e)
      {
        if (!is_valid_output<OutputProvider>(e))
          throw std::runtime_error("You can only disable outputs that are part of the class.");
        
        if (!locked_)
          dynamicallyEnabled_[static_cast<size_t>(e)] = false;
        else
          throw std::runtime_error("Outputs have been locked, you cannot disable one.");
      }

      /** Multi-argument version of disableOutput. */
      template<typename EnumT, typename ... Args>
      void disableOutput(EnumT e, Args ... args)
      {
        disableOutput(e);
        disableOutput(args...);
      }

      /** Enable an output. The enum must refer to an existing output. */
      template<typename EnumT>
      void enableOutput(EnumT e)
      {
        if (!is_valid_output<OutputProvider>(e))
          throw std::runtime_error("You can only disable outputs that are part of the class.");

        if (!isOutputStaticallyEnabled(static_cast<int>(e)))
          throw std::runtime_error("You can not enable outputs that are statically disabled.");

        if (!locked_)
          dynamicallyEnabled_[static_cast<size_t>(e)] = true;
        else
          throw std::runtime_error("Outputs have been locked, you cannot enable one.");
      }

      /** Multi-argument version of enableOutput. */
      template<typename EnumT, typename ... Args>
      void enableOutput(EnumT e, Args ... args)
      {
        enableOutput(e);
        enableOutput(args...);
      }

      /** Override the method of Outputs to get the desired enable/disable behavior. */
      virtual bool isOutputCustomEnabled(int e) const override
      {
        if (e < 0 || e >= static_cast<int>(dynamicallyEnabled_.size()))
          throw std::runtime_error("Enum value is invalid");
        return dynamicallyEnabled_[e];
      }
    };
  }
}
