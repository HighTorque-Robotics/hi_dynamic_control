/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/********************************************************************************
Modified Copyright (c) 2023-2024, hightorque Robotics.Co.Ltd. All rights reserved.

For further information, contact: service@hightorque.cn or visit our website
at https://www.hightorque.cn/.
********************************************************************************/

#include "hi_interface/constraint/XYReferenceConstraintCppAd.h"
#include "hi_interface/hiRobotPreComputation.h"

namespace ocs2
{
namespace legged_robot
{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
XYReferenceConstraintCppAd::XYReferenceConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                       const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                       size_t contactPointIndex)
  : StateInputConstraint(ConstraintOrder::Linear)
  , referenceManagerPtr_(&referenceManager)
  , eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 2))
  , contactPointIndex_(contactPointIndex)
{
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
XYReferenceConstraintCppAd::XYReferenceConstraintCppAd(const XYReferenceConstraintCppAd& rhs)
  : StateInputConstraint(rhs)
  , referenceManagerPtr_(rhs.referenceManagerPtr_)
  , eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone())
  , contactPointIndex_(rhs.contactPointIndex_)
{
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool XYReferenceConstraintCppAd::isActive(scalar_t time) const
{
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t XYReferenceConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                              const PreComputation& preComp) const
{
  const auto& preCompLegged = cast<piRobotPreComputation>(preComp);
  eeLinearConstraintPtr_->configure(preCompLegged.getEeXYReferenceConstraintConfigs()[contactPointIndex_]);

  return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation XYReferenceConstraintCppAd::getLinearApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const
{
  const auto& preCompLegged = cast<piRobotPreComputation>(preComp);
  eeLinearConstraintPtr_->configure(preCompLegged.getEeXYReferenceConstraintConfigs()[contactPointIndex_]);

  return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

}  // namespace legged_robot
}  // namespace ocs2
