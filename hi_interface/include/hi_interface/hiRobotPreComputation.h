/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include <hi_interface/common/ModelSettings.h>

#include "hi_interface/constraint/EndEffectorLinearConstraint.h"
#include "hi_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <mutex>

namespace ocs2
{
namespace legged_robot
{
/** Callback for caching and reference update */
class piRobotPreComputation : public PreComputation
{
public:
  piRobotPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                            const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings);
  ~piRobotPreComputation() override = default;

  piRobotPreComputation* clone() const override
  {
    return new piRobotPreComputation(*this);
  }

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const
  {
    return eeNormalVelConConfigs_;
  }
  const std::vector<EndEffectorLinearConstraint::Config>& getEeXYReferenceConstraintConfigs() const
  {
    return eeXYRefConConfigs_;
  }
  const std::vector<EndEffectorLinearConstraint::Config>& getEeXYLimitConstraintConfigs() const
  {
    return eeXYLimitConConfigs_;
  }

  PinocchioInterface& getPinocchioInterface()
  {
    return pinocchioInterface_;
  }
  const PinocchioInterface& getPinocchioInterface() const
  {
    return pinocchioInterface_;
  }

protected:
  piRobotPreComputation(const piRobotPreComputation& other);

private:
  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;
  const ModelSettings settings_;

  std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;
  std::vector<EndEffectorLinearConstraint::Config> eeXYRefConConfigs_;
  std::vector<EndEffectorLinearConstraint::Config> eeXYLimitConConfigs_;
};

}  // namespace legged_robot
}  // namespace ocs2
