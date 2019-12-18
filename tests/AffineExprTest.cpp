#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#define DOCTEST_CONFIG_SUPER_FAST_ASSERTS
#include "doctest/doctest.h"

#include<tvm/Variable.h>
#include<tvm/function/BasicLinearFunction.h>
#include<tvm/utils/AffineExpr.h>
#include<tvm/internal/enums.h>

#include <iostream>

using namespace tvm;
using namespace tvm::utils;
using namespace tvm::function;
using namespace Eigen;

#define CREATE_VEC(x, vx) VectorXd x = vx;
#define CREATE_VAR(x, vx) \
  Space R##x(static_cast<int>(vx.size())); \
  VariablePtr x = R##x.createVariable(#x); \
  x << vx;

#define EVAL_EIG(res, expr) res = expr;
#define EVAL_AFF(res, expr) \
  BasicLinearFunction f(expr); \
  f.updateValue(); \
  res = f.value();

#define GENERATE(create, eval, ...) \
  PP_ID(PP_APPLY(CHOOSE_GEN_START, PP_NARG(__VA_ARGS__)) \
          (create, eval, __VA_ARGS__))

#define CHOOSE_GEN_START(count) GENERATE##count

#define GENERATE2(create, eval, res, expr) eval(res, expr)
#define GENERATE4(create, eval, res, expr, x, vx, ...) create(x,vx) PP_ID(GENERATE2(create, eval, res, expr))
#define GENERATE6(create, eval, res, expr, x, vx, ...) create(x,vx) PP_ID(GENERATE4(create, eval, res, expr, __VA_ARGS__))
#define GENERATE8(create, eval, res, expr, x, vx, ...) create(x,vx) PP_ID(GENERATE6(create, eval, res, expr, __VA_ARGS__))
#define GENERATE10(create, eval, res, expr, x, vx, ...) create(x,vx) PP_ID(GENERATE8(create, eval, res, expr, __VA_ARGS__))
#define GENERATE12(create, eval, res, expr, x, vx, ...) create(x,vx) PP_ID(GENERATE10(create, eval, res, expr, __VA_ARGS__))

#define TEST_AFFINE_EXPR(expr, ...) \
{ \
  VectorXd res1, res2; \
  { \
    PP_ID(GENERATE(CREATE_VEC, EVAL_EIG, res1, expr, __VA_ARGS__)) \
  } \
  { \
    PP_ID(GENERATE(CREATE_VAR, EVAL_AFF, res2, expr, __VA_ARGS__)) \
  } \
  FAST_CHECK_UNARY((res1-res2).isZero()); \
}


TEST_CASE("Test for AffineExpr")
{
  VectorXd vx = VectorXd::Random(5);
  VectorXd vy = VectorXd::Random(5);
  VectorXd vz = VectorXd::Random(3);
  VectorXd vw = VectorXd::Random(3);
  MatrixXd A = MatrixXd::Random(6, 5);
  MatrixXd B = MatrixXd::Random(6, 5);
  MatrixXd C = MatrixXd::Random(6, 3);
  MatrixXd M = MatrixXd::Random(6, 6);
  VectorXd d = VectorXd::Random(6);
  VectorXd e = VectorXd::Random(6);
  
  
  TEST_AFFINE_EXPR(A * x,                                     x, vx)                      //LinearExpr, simple case
  TEST_AFFINE_EXPR(M * A * x,                                 x, vx)                      //LinearExpr, simple matrix expression
  TEST_AFFINE_EXPR((d.asDiagonal() * A + M * B) * x,          x, vx)                      // complex matrix expression
  TEST_AFFINE_EXPR(A * x + d,                                 x, vx)                      // AffineExpr = LinearExpr + vector
  TEST_AFFINE_EXPR(d + A * x,                                 x, vx)                      // AffineExpr = vector + LinearExpr
  TEST_AFFINE_EXPR(A * x + (3*d+M*e),                         x, vx)                      // AffineExpr = LinearExpr + vector(complex expr)
  TEST_AFFINE_EXPR(A * x + B * y,                             x, vx, y, vy)               // AffineExpr = LinearExpr + LinearExpr
  TEST_AFFINE_EXPR(A * x + B * y + C * z,                     x, vx, y, vy, z, vz)        // AffineExpr = AffineExpr + LinearExpr
  TEST_AFFINE_EXPR(A * x + (B * y + C * z),                   x, vx, y, vy, z, vz)        // AffineExpr = LinearExpr + AffineExpr
  TEST_AFFINE_EXPR(A * x + B * y + d,                         x, vx, y, vy)               // AffineExpr = AffineExpr(NoConstant) + vector
  TEST_AFFINE_EXPR(d + (A * x + B * y),                       x, vx, y, vy)               // AffineExpr = vector + AffineExpr(NoConstant)
  TEST_AFFINE_EXPR(A * x + d + e,                             x, vx)                      // AffineExpr = AffineExpr(with constant) + vector
  TEST_AFFINE_EXPR(A * x + d + B * y + e,                     x, vx, y, vy)               // AffineExpr = AffineExpr(with constant) + vector
  TEST_AFFINE_EXPR(e + (A * x + d),                           x, vx)                      // AffineExpr = vector + AffineExpr(with constant)
  TEST_AFFINE_EXPR(e + (A * x + d + B * y),                   x, vx, y, vy)               // AffineExpr = vector + AffineExpr(with constant)
  TEST_AFFINE_EXPR((A * x + B * y) + (C * z + C * w),         x, vx, y, vy, z, vz, w, vw) // AffineExpr = AffineExpr(NoConstant) + AffineExpr(NoConstant)
  TEST_AFFINE_EXPR((A * x + B * y) + (C * z + C * w + e),     x, vx, y, vy, z, vz, w, vw) // AffineExpr = AffineExpr(NoConstant) + AffineExpr(with constant)
  TEST_AFFINE_EXPR((A * x + B * y + d) + (C * z + C * w),     x, vx, y, vy, z, vz, w, vw) // AffineExpr = AffineExpr(with constant) + AffineExpr(with constant)
  TEST_AFFINE_EXPR((A * x + B * y + d) + (C * z + C * w + e), x, vx, y, vy, z, vz, w, vw) // AffineExpr = AffineExpr(with complex constant) + AffineExpr(with complex constant)

}
