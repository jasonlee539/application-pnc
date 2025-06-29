/****************************************************************************
 * Copyright 2025 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ****************************************************************************/

/**
 * @file
 * @brief Unprotected right‑turn scenario at signalised intersections.
 *
 * Major change: introduce a robust right‑turn detection routine that relies
 * first on HDMap lane semantics and second on geometric heading change along
 * the reference line.  The older |tan|>20 heuristic has been removed.
 */

#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_creep.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_intersection_cruise.h"
#include "modules/planning/scenarios/traffic_light_unprotected_right_turn/stage_stop.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;
namespace math = apollo::common::math;

namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Determine whether the path at @p check_s represents a right‑turn.
 *
 * Strategy:
 *   1. Use HDMap lane semantics (RIGHT_TURN).
 *   2. Fall back to geometric heading change: look 10 m ahead; if the heading
 *      decreases more than 45 ° (right‑hand traffic), we classify as right‑turn.
 */
bool IsRightTurnOnPath(const ReferenceLineInfo &reference_line_info, const double check_s) {
    // (1) Lane semantics (most reliable) --------------------------------------
    const auto lane_turn = reference_line_info.GetPathTurnType(check_s);
    if (lane_turn == hdmap::Lane::RIGHT_TURN) {
        return true;
    }

    // (2) Geometric fallback ---------------------------------------------------
    const auto &ref_line = reference_line_info.reference_line();
    const double kLookAhead = 10.0;  // metres
    const double s_ahead = check_s + kLookAhead;

    if (s_ahead >= ref_line.Length()) {
        return false;  // Not enough reference line to evaluate.
    }

    const auto &pt_now = ref_line.GetReferencePoint(check_s);
    const auto &pt_ahead = ref_line.GetReferencePoint(s_ahead);

    const double heading_now = pt_now.heading();
    const double heading_ahead = pt_ahead.heading();
    const double heading_diff = math::NormalizeAngle(heading_ahead - heading_now);

    constexpr double kRightTurnThresh = -M_PI_4;  // −45 ° threshold
    return heading_diff < kRightTurnThresh;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Scenario methods
// ─────────────────────────────────────────────────────────────────────────────

bool TrafficLightUnprotectedRightTurnScenario::Init(
        std::shared_ptr<DependencyInjector> injector,
        const std::string &name) {
    if (init_) {
        return true;
    }

    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<ScenarioTrafficLightUnprotectedRightTurnConfig>(&context_.scenario_config)) {
        AERROR << "fail to get specific config of scenario " << Name();
        return false;
    }

    init_ = true;
    return true;
}

bool TrafficLightUnprotectedRightTurnScenario::IsTransferable(
        const Scenario *const other_scenario,
        const Frame &frame) {
    // Must have a lane‑follow command.
    if (!frame.local_view().planning_command->has_lane_follow_command()) {
        return false;
    }
    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        return false;
    }

    const auto &reference_line_info = frame.reference_line_info().front();

    // ① Find first encountered traffic‑light overlap ---------------------------
    const hdmap::PathOverlap *tl_overlap_ptr = nullptr;
    for (const auto &ov : reference_line_info.FirstEncounteredOverlaps()) {
        if (ov.first == ReferenceLineInfo::STOP_SIGN || ov.first == ReferenceLineInfo::YIELD_SIGN) {
            return false;  // Different scenario.
        }
        if (ov.first == ReferenceLineInfo::SIGNAL) {
            tl_overlap_ptr = &ov.second;
            break;
        }
    }
    if (!tl_overlap_ptr) {
        return false;
    }

    // ② Ensure the manoeuvre is a right‑turn -----------------------------------
    if (!IsRightTurnOnPath(reference_line_info, tl_overlap_ptr->start_s)) {
        return false;
    }

    // ③ Group neighbouring traffic lights & check colour -----------------------
    const auto &ref_line = reference_line_info.reference_line();
    const auto &all_tl_overlaps = ref_line.map_path().signal_overlaps();
    static constexpr double kGroupDist = 2.0;  // m
    std::vector<hdmap::PathOverlap> grouped;
    for (const auto &ov : all_tl_overlaps) {
        if (std::fabs(ov.start_s - tl_overlap_ptr->start_s) <= kGroupDist) {
            grouped.push_back(ov);
        }
    }

    const double check_range = context_.scenario_config.start_traffic_light_scenario_distance();
    const double adc_front_s = reference_line_info.AdcSlBoundary().end_s();

    bool need_scenario = false;
    for (const auto &ov : grouped) {
        const double dist = ov.start_s - adc_front_s;
        if (dist <= 0.0 || dist > check_range) {
            continue;  // Passed or too far.
        }
        const auto color = frame.GetSignal(ov.object_id).color();
        if (color != perception::TrafficLight::GREEN && color != perception::TrafficLight::BLACK) {
            need_scenario = true;
            break;
        }
    }
    if (!need_scenario) {
        return false;
    }

    // ④ Populate context -------------------------------------------------------
    context_.current_traffic_light_overlap_ids.clear();
    for (const auto &ov : grouped) {
        context_.current_traffic_light_overlap_ids.push_back(ov.object_id);
    }

    return true;
}

bool TrafficLightUnprotectedRightTurnScenario::Enter(Frame *frame) {
    const auto &reference_line_info = frame->reference_line_info().front();
    std::string first_tl_id;
    for (const auto &ov : reference_line_info.FirstEncounteredOverlaps()) {
        if (ov.first == ReferenceLineInfo::SIGNAL) {
            first_tl_id = ov.second.object_id;
            break;
        }
    }
    if (first_tl_id.empty()) {
        injector_->planning_context()->mutable_planning_status()->mutable_traffic_light()->Clear();
        AERROR << "Cannot find traffic-light overlap!";
        return false;
    }

    const auto &all_tl_overlaps = reference_line_info.reference_line().map_path().signal_overlaps();
    auto base_it = std::find_if(
            all_tl_overlaps.begin(), all_tl_overlaps.end(), [&first_tl_id](const hdmap::PathOverlap &ov) {
                return ov.object_id == first_tl_id;
            });
    if (base_it == all_tl_overlaps.end()) {
        injector_->planning_context()->mutable_planning_status()->mutable_traffic_light()->Clear();
        return true;
    }

    static constexpr double kGroupDist = 2.0;  // m
    const double base_s = base_it->start_s;
    for (const auto &ov : all_tl_overlaps) {
        if (std::fabs(ov.start_s - base_s) <= kGroupDist) {
            injector_->planning_context()
                    ->mutable_planning_status()
                    ->mutable_traffic_light()
                    ->add_current_traffic_light_overlap_id(ov.object_id);
            ADEBUG << "Add traffic_light " << ov.object_id << " at s " << ov.start_s;
        }
    }
    return true;
}

bool TrafficLightUnprotectedRightTurnScenario::Exit(Frame * /*frame*/) {
    injector_->planning_context()->mutable_planning_status()->mutable_traffic_light()->Clear();
    return true;
}

}  // namespace planning
}  // namespace apollo
