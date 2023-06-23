#ifndef STATE_VALIDITY_CHECKER_H_
#define STATE_VALIDITY_CHECKER_H_

#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "swarm_planner_deps/swarm_config_tracker.hpp"
#include <cmath>

namespace ob = ompl::base;

namespace swarm_planner {
    class SwarmStateValidityChecker: public ob::StateValidityChecker {
    private:
        int drone_index_;
        std::shared_ptr<SwarmConfigTracker> swarm_config_tracker_;
    public:
        SwarmStateValidityChecker(const ob::SpaceInformationPtr &si): ob::StateValidityChecker(si) {}

        virtual bool isValid(const ob::State *state) const override;

        void set_swarm_config_tracker(std::shared_ptr<SwarmConfigTracker> swarm_config_tracker);
        void set_drone_index(int drone_index);
    };

} // namespace swarm_planner

#endif // STATE_VALIDITY_CHECKER_H_
