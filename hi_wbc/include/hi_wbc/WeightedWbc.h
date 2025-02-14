//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, hightorque Robotics.Co.Ltd. All rights reserved.

For further information, contact: service@hightorque.cn or visit our website
at https://www.hightorque.cn/.
********************************************************************************/

#include "hi_wbc/WbcBase.h"

namespace legged
{
class WeightedWbc : public WbcBase
{
public:
  using WbcBase::WbcBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                  size_t mode, scalar_t period) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

protected:
  virtual Task formulateConstraints();
  virtual Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  Task formulateStanceBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);

private:
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;

  vector_t last_qpSol;
};

}  // namespace legged