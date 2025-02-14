//
// Created by qiayuan on 22-12-23.
//
/********************************************************************************
Modified Copyright (c) 2023-2024, hightorque Robotics.Co.Ltd. All rights reserved.

For further information, contact: service@hightorque.cn or visit our website
at https://www.hightorque.cn/.
********************************************************************************/

#pragma once

#include "hi_wbc/WbcBase.h"

namespace legged
{
class HierarchicalWbc : public WbcBase
{
public:
  using WbcBase::WbcBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                  size_t mode, scalar_t period) override;
};

}  // namespace legged
