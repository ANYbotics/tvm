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

#include <tvm/internal/meta.h>

#include <Eigen/Core>

#include <tvm/utils/AffineExpr.h>

using namespace tvm::internal;
using namespace tvm::utils;


//---------------------- derives_from ------------------------\\

static_assert(derives_from<Eigen::MatrixXd, Eigen::MatrixBase>());
static_assert(!derives_from<double, Eigen::MatrixBase>());
static_assert(derives_from<LinearExpr<Eigen::MatrixXd>, LinearExpr>());
static_assert(!derives_from<Eigen::MatrixXd, LinearExpr>());
static_assert(derives_from<AffineExpr<::internal::NoConstant, Eigen::MatrixXd>, AffineExpr>());
static_assert(!derives_from<Eigen::MatrixXd, AffineExpr>());
static_assert(derives_from<Eigen::MatrixXd, Eigen::MatrixXd>());
static_assert(!derives_from<int, int>()); //derives_from only work with classes

//---------- enable_for_t and enable_for_templated_t ---------\\

// Dummy classes for test purposes
class A {};
template<typename U, typename V> class TemplatedClass {};
template<typename U> class TemplatedClassD1 : public TemplatedClass<U, int> {};
class TemplatedClassD2 : public TemplatedClass<double, int> {};
class TemplatedClassD3 : public TemplatedClassD1<int> {};

//function accepting int and Eigen::MatrixXd
template<typename T, enable_for_t<T, int, Eigen::MatrixXd> = 0>
constexpr std::true_type testSimple(const T&) { return {}; }
// Fallback version
constexpr std::false_type testSimple(...) { return {}; }

//function accepting Eigen::MatrixBase, tvm::utils::LinearExpr, tvm::utils::AffineExpr
template<typename T, enable_for_templated_t<T, Eigen::MatrixBase, TemplatedClass> = 0>
constexpr std::true_type testTemplate(const T&) { return {}; }
// Fallback version
constexpr std::false_type testTemplate(...) { return {}; }

static_assert(testSimple(3));
static_assert(!testSimple(6.));
static_assert(decltype(testSimple(Eigen::MatrixXd()))::value);
static_assert(!decltype(testSimple(Eigen::Matrix4d()))::value);
static_assert(!decltype(testSimple(A()))::value);

static_assert(decltype(testTemplate(Eigen::MatrixXd()))::value);
static_assert(decltype(testTemplate(Eigen::Matrix4d()))::value);
static_assert(!decltype(testTemplate(A()))::value);
static_assert(decltype(testTemplate(TemplatedClass<int, double>()))::value);
static_assert(decltype(testTemplate(TemplatedClassD1<int>()))::value);
static_assert(decltype(testTemplate(TemplatedClassD2()))::value);
static_assert(decltype(testTemplate(TemplatedClassD3()))::value);


//---------------- always_true, always_false -----------------\\

static_assert(always_true<int>::value);
static_assert(always_true<A>::value);
static_assert(!always_false<int>::value);
static_assert(!always_false<A>::value);

//------------------ has_member_type_XXX ---------------------\\

TVM_CREATE_HAS_MEMBER_TYPE_TRAIT_FOR(Foo)

class B { using Foo = int; };
static_assert(!has_member_type_Foo<int>::value);
static_assert(!has_member_type_Foo<A>::value);
static_assert(has_member_type_Foo<B>::value);
