/********************************************************************************
Modified Copyright (c) 2023-2024, hightorque Robotics.Co.Ltd. All rights reserved.

For further information, contact: service@hightorque.cn or visit our website
at https://www.hightorque.cn/.
********************************************************************************/

#pragma once

#include <ocs2_core/Types.h>
#include "std_msgs/Float64MultiArray.h"

namespace legged
{
using namespace ocs2;

std_msgs::Float64MultiArray createFloat64MultiArrayFromVector(const vector_t& data);

}  // namespace legged