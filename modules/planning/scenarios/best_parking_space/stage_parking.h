

#pragma once

#include <memory>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/scenarios/best_parking_space/best_parking_space.h"
namespace apollo {
namespace planning {

class StageBestParking : public Stage {
public:
    StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

private:
    StageResult FinishStage();

    BestParkingSpaceConfig scenario_config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::StageBestParking, Stage)

}  // namespace planning
}  // namespace apollo
