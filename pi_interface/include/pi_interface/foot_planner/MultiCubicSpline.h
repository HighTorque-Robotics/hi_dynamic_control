/********************************************************************************
Modified Copyright (c) 2023-2024, hightorque Robotics.Co.Ltd. All rights reserved.

For further information, contact: service@hightorque.cn or visit our website
at https://www.hightorque.cn/.
********************************************************************************/

#pragma once

#include "pi_interface/foot_planner/CubicSpline.h"

namespace ocs2
{
namespace legged_robot
{
class MultiCubicSpline
{
public:
  MultiCubicSpline()
  {
  }
  MultiCubicSpline(std::vector<CubicSpline::Node> nodes);

  scalar_t position(scalar_t time) const;

  scalar_t velocity(scalar_t time) const;

  scalar_t acceleration(scalar_t time) const;

  const std::vector<CubicSpline::Node>& getNodes() const
  {
    return nodes_;
  }

private:
  std::vector<CubicSpline::Node> nodes_;
  std::vector<CubicSpline> cubic_splines_;
};

}  // namespace legged_robot
}  // namespace ocs2
