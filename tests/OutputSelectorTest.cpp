/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2012-2019, CNRS-UM LIRMM, CNRS-AIST JRL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <gtest/gtest.h>

#include <tvm/graph/abstract/Node.h>
#include <tvm/graph/abstract/OutputSelector.h>

#include <iostream>

using namespace tvm::graph::abstract;

class Provider1 : public Node<Provider1>
{
public:
  SET_OUTPUTS(Provider1, O0, O1, O2)

  Provider1()
  {
  }
};

class Provider2 : public Provider1
{
public:
  SET_OUTPUTS(Provider2, O3, O4)
  DISABLE_OUTPUTS(Provider1::Output::O1)

  Provider2()
  {
  }
};

class Provider3 : public OutputSelector<Provider2>
{
public:
  Provider3()
  {
    disableOutput(Provider1::Output::O1, Provider2::Output::O3);
  }
};

class Provider4 : public Provider3
{
public:
  SET_OUTPUTS(Provider4, O5, O6, O7)

  Provider4()
  {
  }
};

class Provider5 : public OutputSelector<Provider4>
{
public:
  Provider5()
  {
    disableOutput(Provider1::Output::O0, Provider4::Output::O6);
  }

  template<typename EnumT>
  void manualEnable(EnumT e) { enableOutput(e); }

  template<typename EnumT>
  void manualDisable(EnumT e) { disableOutput(e); }
};

class Provider6 : public OutputSelector<Provider6, Provider4>
{
public:
  SET_OUTPUTS(Provider6, O8, O9)

  Provider6()
  {
    disableOutput(Provider6::Output::O9, Provider4::Output::O5, Provider2::Output::O3, Provider1::Output::O1);
  }
};


TEST(OutputSelectorTest, outputsSelector) {  // NOLINT
  Provider3 p3;
  EXPECT_TRUE(p3.isOutputEnabled(Provider1::Output::O0));
  EXPECT_FALSE(p3.isOutputEnabled(Provider1::Output::O1));
  EXPECT_TRUE(p3.isOutputEnabled(Provider1::Output::O2));
  EXPECT_FALSE(p3.isOutputEnabled(Provider2::Output::O3));
  EXPECT_TRUE(p3.isOutputEnabled(Provider2::Output::O4));

  Provider5 p5;
  for (int i = 0; i < 8; ++i)
  {
    if(i != 0 && i != 1 && i != 3 && i != 6)
    {
      EXPECT_TRUE(p5.isOutputEnabled(i));
    }
    else
    {
      EXPECT_FALSE(p5.isOutputEnabled(i));
    }
  }

  Provider6 p6;
  for(int i = 0; i < 10; ++i)
  {
    if(i != 1 && i != 3 && i != 5 && i != 9)
    {
      EXPECT_TRUE(p6.isOutputEnabled(i));
    }
    else
    {
      EXPECT_FALSE(p6.isOutputEnabled(i));
    }
  }

  p5.manualEnable(Provider4::Output::O6);
  EXPECT_THROW(p5.manualEnable(Provider1::Output::O1), std::runtime_error);
  p5.lock();
  EXPECT_THROW(p5.manualDisable(Provider1::Output::O2), std::runtime_error);
}
