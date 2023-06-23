#include "swarm_planner.hpp"

namespace swarm_planner {
    void SwarmPlannerSE2::initialize_planners() {
        std::cout << "[SwarmPlannerSE2::initialize_planners] there are " << this->swarm_config_tracker_->num_drones << " drones\n";
        for (int i=0; i < this->swarm_config_tracker_->num_drones; i++) {
            std::cout << "initializtion conditions met, initializing stuff\n";
            this->si_vector.push_back(std::make_shared<ob::SpaceInformation>(this->space));
            this->state_validity_checker_vector.push_back(std::make_shared<SwarmStateValidityChecker>(this->si_vector[i]));
            this->state_validity_checker_vector[i]->set_drone_index(i);
            this->state_validity_checker_vector[i]->set_swarm_config_tracker(this->swarm_config_tracker_);
            this->si_vector[i]->setStateValidityChecker(this->state_validity_checker_vector[i]);
            this->planner_vector.push_back(std::make_shared<current_planner>(this->si_vector[i]));
        }

        std::cout << this->si_vector.size() << " initialized [n] drones\n";
    }

    SwarmPlannerSE2::SwarmPlannerSE2(std::vector<Eigen::Vector2d> workspace_bounds, std::shared_ptr<SwarmConfigTracker> swarm_config_tracker) {
        this->swarm_config_tracker_ = swarm_config_tracker;
        this->drone_paths = std::make_shared<std::vector<std::vector<Eigen::Vector2d>>>();
        this->drones_path_found = std::make_shared<std::vector<bool>>();

        this->space = std::make_shared<ob::SE2StateSpace>();
        ob::RealVectorBounds bounds(2);
        bounds.setHigh(0, workspace_bounds[0][0]);
        bounds.setLow(0, workspace_bounds[0][1]);
        bounds.setHigh(1, workspace_bounds[1][0]);
        bounds.setLow(1, workspace_bounds[1][1]);

        this->space->setBounds(bounds);
        if (this->swarm_config_tracker_->num_drones != -1) {
            this->initialize_planners();
        }
    }

    std::tuple<std::vector<bool>, std::vector<std::vector<Eigen::Vector2d>>> SwarmPlannerSE2::get_paths() {
        std::vector<bool> paths_found = *this->drones_path_found;
        std::vector<std::vector<Eigen::Vector2d>> paths = *this->drone_paths;
        return std::make_tuple(paths_found, paths);
    }

    bool SwarmPlannerSE2::plan_paths() {
        std::cout << "planning initiated\n";
        if (this->swarm_config_tracker_->num_drones == -1) {
            std::cout << "drones haven't been added\n";
            return false;
        } else if (this->si_vector.size() != this->swarm_config_tracker_->num_drones) {
            std::cout << "num drones don't match the number of elements in vectors\n";
            std::cout << "there are " << this->swarm_config_tracker_->num_drones << "drones and " << this->si_vector.size() << "vector elememts\n";
            return false; // for now. TODO keep track of which drones are joining and separating and maintain si and planners accordingly
        } else {
            this->drone_paths->resize(this->swarm_config_tracker_->num_drones);
            this->drones_path_found->resize(this->swarm_config_tracker_->num_drones);

            // std::cout << "resized the drone paths\n";
            std::vector<std::vector<Eigen::Vector2d>> temp_paths(this->swarm_config_tracker_->num_drones);
            std::vector<bool> temp_path_founds(this->swarm_config_tracker_->num_drones, false);

            std::vector<std::jthread> planning_threads;
            for (int i = 0; i < this->swarm_config_tracker_->num_drones; i++) {
                // std::cout << "start position " << (*this->swarm_config_tracker_->drone_states_)[i] << std::endl;
                planning_threads.push_back(std::jthread([&temp_paths, &temp_path_founds, i, this]() {
                    auto planner(std::make_shared<current_planner>(this->si_vector[i]));
                    // planner->setRange(2.1);
                    // planner->setNearestNeighbors();
                    // planner->setKNearest(2);
                    // planner->setGoalBias(0.1); // Adjust the goalBias parameter
                    // planner->setPruneThreshold(5.1); // Adjust the pruneThreshold parameter

                    auto pdef(std::make_shared<ob::ProblemDefinition>(this->si_vector[i]));
                    std::cout << "initialized pdef in thread " << i << std::endl;

                    ob::ScopedState<> start(this->space);
                    ob::ScopedState<> goal(this->space);
                    std::cout << "initializing start and goal\n";

                    Eigen::Vector4d temp_drone_state;
                    Eigen::Vector2d temp_drone_goal;
                    std::shared_lock<std::shared_mutex> current_lock = this->swarm_config_tracker_->read_swarm_config();

                    temp_drone_state = (*this->swarm_config_tracker_->drone_states_)[i];
                    temp_drone_goal = (*this->swarm_config_tracker_->drone_goals_)[i];

                    current_lock.unlock();

                    start->as<ompl::base::SE2StateSpace::StateType>()->setXY(temp_drone_state[0], temp_drone_state[1]);
                    start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(0);

                    goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(temp_drone_goal[0], temp_drone_goal[1]);
                    goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(0);

                    pdef->setStartAndGoalStates(start, goal);
                    // std::cout << temp_drone_state[0];

                    // this->planner_vector[i]->setProblemDefinition(pdef);
                    planner->setProblemDefinition(pdef);

                    planner->setup();
                    // this->planner_vector[i]->setup();

                    std::cout << "attempting to solve for a path for drone " << i << std::endl;
                    // std::cout << "the start point is " << start << std::endl;
                    // ob::PlannerStatus solved = this->planner_vector[i]->ob::Planner::solve(0.015);
                    ob::PlannerStatus solved = planner->ob::Planner::solve(0.4);
                    std::cout << "attempt complete for drone " << i << std::endl;
                    if (solved) {
                        std::cout << "path for drone " << i << " found\n";
                        temp_path_founds[i] = true;
                        ob::PathPtr base_path = pdef->getSolutionPath();

                        og::PathGeometric path( dynamic_cast< const og::PathGeometric& >( *base_path ));

                        unsigned int path_len = path.getStateCount();
                        std::vector<ob::State*> states = path.getStates();
                        temp_paths[i].clear();
                        for (int j=0; j < path_len; j++) {
                            const ob::State* state = states[j];
                            const auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
                            double x = se2_state->getX();
                            double y = se2_state->getY();
                            std::cout << "Drone " << i << " path idx " << j << ": (" << x << ", " << y << ")\n";
                            temp_paths[i].push_back(Eigen::Vector2d(x, y));
                        }
                    } else {
                        temp_path_founds[i] = false;
                    }
                }));
            }

            for (int i=0; i<planning_threads.size(); i++) {
                planning_threads[i].join();
                *(this->drones_path_found) = temp_path_founds;
                *(this->drone_paths) = temp_paths;
            }

        }
        return true;
    }
}
