#ifndef SWARM_PLANNER_H_
#define SWARM_PLANNER_H_

#define DOUBLE_PRECISION

#include <Eigen/Dense>
#include <Eigen/Core>
#include <shared_mutex>
#include <thread>
#include <future>

// #include "base.hpp"
// #include "workspace.hpp"
#include "swarm_planner_deps/state_validity_checker.hpp"
#include "swarm_planner_deps/swarm_config_tracker.hpp"

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/Path.h"

#include "ompl/geometric/planners/fmt/FMT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/InformedRRTstar.h"
#include "ompl/geometric/planners/rrt/SORRTstar.h"
#include "ompl/geometric/planners/est/EST.h"

#include "ompl/geometric/PathGeometric.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// using current_planner = og::FMT;
using current_planner = og::InformedRRTstar;
// using current_planner = og::SORRTstar;
// using current_planner = og::RRTConnect;
// using current_planner = og::EST;

namespace swarm_planner {
    class SwarmPlannerSE2 {
    private:
        std::shared_ptr<ob::SE2StateSpace> space;
        std::vector<std::shared_ptr<ob::SpaceInformation>> si_vector;
        std::vector<std::shared_ptr<SwarmStateValidityChecker>> state_validity_checker_vector;
        std::vector<std::shared_ptr<current_planner>> planner_vector;

        std::shared_ptr<SwarmConfigTracker> swarm_config_tracker_;
        std::shared_ptr<std::vector<std::vector<Eigen::Vector2d>>> drone_paths;
        std::shared_ptr<std::vector<bool>> drones_path_found;

        void initialize_planners();

    public:
        std::tuple<std::vector<bool>, std::vector<std::vector<Eigen::Vector2d>>> get_paths();
        SwarmPlannerSE2(std::vector<Eigen::Vector2d> bounds, std::shared_ptr<SwarmConfigTracker> swarm_config_tracker);
        bool plan_paths();
        bool write_states_and_goals(std::vector<Eigen::Vector4d> drone_states,
                                    std::vector<Eigen::Vector2d> drone_goals);
    };

} // namespace swarm_planner

#endif // SWARM_PLANNER_H_
