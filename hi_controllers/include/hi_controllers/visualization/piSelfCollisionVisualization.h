//
// Created by qiayuan on 23-1-30.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, hightorque Robotics.Co.Ltd. All rights reserved.

For further information, contact: service@hightorque.cn or visit our website
at https://www.hightorque.cn/.
********************************************************************************/

#pragma once
#include <ros/ros.h>

#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>

#include <utility>

namespace legged
{
using namespace ocs2;

class piSelfCollisionVisualization : public GeometryInterfaceVisualization
{
public:
  piSelfCollisionVisualization(PinocchioInterface pinocchioInterface, PinocchioGeometryInterface geometryInterface,
                                   const CentroidalModelPinocchioMapping& mapping, ros::NodeHandle& nh,
                                   scalar_t maxUpdateFrequency = 50.0)
    : mappingPtr_(mapping.clone())
    , GeometryInterfaceVisualization(std::move(pinocchioInterface), std::move(geometryInterface), nh, "odom")
    , lastTime_(std::numeric_limits<scalar_t>::lowest())
    , minPublishTimeDifference_(1.0 / maxUpdateFrequency)
  {
  }
  void update(const SystemObservation& observation)
  {
    if (observation.time - lastTime_ > minPublishTimeDifference_)
    {
      lastTime_ = observation.time;

      publishDistances(mappingPtr_->getPinocchioJointPosition(observation.state));
    }
  }

private:
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace legged
