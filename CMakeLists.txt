# BSD 3-Clause License
#
# Copyright (c) 2012-2019, CNRS-UM LIRMM, CNRS-AIST JRL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Set up the CMake project.
cmake_minimum_required(VERSION 3.1.3)
project(tvm)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wno-sign-conversion -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Create a list of catkin package dependencies used in header files of this package.
set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
  eigen-quadprog
  eigen-qld
)

## Create a list of catkin package dependencies, now for both header and source files.
set(CATKIN_PACKAGE_DEPENDENCIES
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

## Find catkin dependencies for building and execution.
find_package(catkin REQUIRED
  COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

###########
## Build ##
###########
## Declare a C++ library.
add_library(${PROJECT_NAME}
  src/Clock.cpp
  src/ControlProblem.cpp
  src/LinearizedControlProblem.cpp
  src/Space.cpp
  src/Task.cpp
  src/Variable.cpp
  src/VariableVector.cpp

  src/constraint/BasicLinearConstraint.cpp
  src/constraint/Constraint.cpp
  src/constraint/LinearConstraint.cpp
  src/constraint/LinearizedTaskConstraint.cpp
  src/constraint/RHSVectors.cpp

  src/event/Listener.cpp
  src/event/Source.cpp

  src/function/BasicLinearFunction.cpp
  src/function/Function.cpp
  src/function/IdentityFunction.cpp
  src/function/LinearFunction.cpp

  src/geometry/Plane.cpp

  src/graph/CallGraph.cpp
  src/graph/Inputs.cpp
  src/graph/Log.cpp
  src/graph/Logger.cpp

  src/hint/AutoCalculator.cpp
  src/hint/DiagonalCalculator.cpp
  src/hint/GenericCalculator.cpp
  src/hint/Substitution.cpp
  src/hint/SubstitutionCalculator.cpp
  src/hint/SubstitutionCalculatorImpl.cpp
  src/hint/Substitutions.cpp
  src/hint/SubstitutionUnit.cpp

  src/internal/FirstOrderProvider.cpp
  src/internal/MatrixProperties.cpp
  src/internal/ObjWithId.cpp

  src/requirements/SolvingRequirements.cpp

  src/scheme/Assignment.cpp
  src/scheme/AssignmentTarget.cpp
  src/scheme/helpers.cpp
  src/scheme/ResolutionScheme.cpp
  src/scheme/SchemeAbilities.cpp
  src/scheme/WeightedLeastSquares.cpp

  src/solver/defaultLeastSquareSolver.cpp
  src/solver/LeastSquareSolver.cpp

  src/task_dynamics/Constant.cpp
  src/task_dynamics/None.cpp
  src/task_dynamics/Proportional.cpp
  src/task_dynamics/ProportionalDerivative.cpp
  src/task_dynamics/TaskDynamics.cpp
  src/task_dynamics/TaskDynamicsImpl.cpp
  src/task_dynamics/VelocityDamper.cpp

  src/utils/UpdatelessFunction.cpp
  src/utils/checkFunction.cpp

  src/solver/QLDLeastSquareSolver.cpp
  src/solver/QuadprogLeastSquareSolver.cpp
)

## Add CMake target dependencies of the library.
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Specify your packages header files.
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)
# Header directories listed after 'SYSTEM' are marked as external, and don't generate warnings.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Specify libraries to link executable targets against.
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC -DTVM_USE_QLD -DTVM_USE_QUADPROG )

#############
## Install ##
#############
install(
  TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
  include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

## GTest.
if(CATKIN_ENABLE_TESTING)
  ## Find catkin dependencies, including test dependencies.
  find_package(catkin REQUIRED
    COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  ## Add gtest based cpp test target and link libraries.
  ## Note that the tests are executed in the order of the list of source files.
  ## Therefore, put basic tests in the top and more advanced tests in the bottom.
  catkin_add_gtest(test_${PROJECT_NAME}
    tests/AffineExprTest.cpp
    tests/AssignmentTest.cpp
    tests/CompiledAssignmentTest.cpp
    tests/ConstraintTest.cpp
    tests/LeastSquareSolverTest.cpp
    tests/SolverTestFunctions.cpp
    tests/LinearizedTaskConstraintTest.cpp
    tests/MatrixPropertiesTest.cpp
    tests/MetaTest.cpp
    tests/OutputSelectorTest.cpp
    tests/SolverTest.cpp
    tests/SolvingRequirementsTest.cpp
    tests/SubstitutionTest.cpp
    tests/TaskDynamicsTest.cpp
    tests/UtilsTest.cpp
    tests/VariableTest.cpp
  )
endif()

## Link GTest.
if(TARGET test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
    tests
  )

  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )

  # Generate test coverage report -> ${PROJECT_BINARY_DIR}/cmake_code_coverage/${PROJECT_NAME}/index.html
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS test_${PROJECT_NAME}
  )
  endif(cmake_code_coverage_FOUND)
endif()

###########
## Clang ##
###########

#find_package(cmake_clang_tools QUIET)
#if(cmake_clang_tools_FOUND)
#  add_default_clang_tooling()
#endif(cmake_clang_tools_FOUND)