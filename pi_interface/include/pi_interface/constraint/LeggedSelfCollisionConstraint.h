//
// Created by qiayuan on 23-1-29.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, hightorque Robotics.Co.Ltd. All rights reserved.

For further information, contact: service@hightorque.cn or visit our website
at https://www.hightorque.cn/.
********************************************************************************/

#pragma once

#include <ocs2_self_collision/SelfCollisionConstraint.h>

#include "pi_interface/piRobotPreComputation.h"

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

class LeggedSelfCollisionConstraint final : public SelfCollisionConstraint
{
public:
  LeggedSelfCollisionConstraint(const CentroidalModelPinocchioMapping& mapping,
                                PinocchioGeometryInterface pinocchioGeometryInterface, scalar_t minimumDistance)
    : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance)
  {
  }
  ~LeggedSelfCollisionConstraint() override = default;
  LeggedSelfCollisionConstraint(const LeggedSelfCollisionConstraint& other) = default;
  LeggedSelfCollisionConstraint* clone() const override
  {
    return new LeggedSelfCollisionConstraint(*this);
  }

  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override
  {
    return cast<piRobotPreComputation>(preComputation).getPinocchioInterface();
  }
};

}  // namespace legged
