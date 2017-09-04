#include "exceptions.h"
#include "Function.h"
#include "Variable.h"

namespace tvm
{
  void Function::resizeCache()
  {
    FirstOrderProvider::resizeCache();

    if (isOutputEnabled((int)Output::Velocity))
      velocity_.resize(size());

    if (isOutputEnabled((int)Output::NormalAcceleration))
      normalAcceleration_.resize(size());

    if (isOutputEnabled((int)Output::JDot))
    {
      for (auto v : variables())
        JDot_[v.get()].resize(size(), v->size());
    }
  }

  void Function::addVariable_(VariablePtr v)
  {
    JDot_[v.get()].resize(size(), v->size());
  }

  void Function::removeVariable_(VariablePtr v)
  {
    JDot_.erase(v.get());
  }
}
