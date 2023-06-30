#include "swarm_planner_deps/swarm_config_tracker.hpp"

namespace swarm_planner {
    SwarmConfigTracker::SwarmConfigTracker() {
        this->drone_states_ = std::make_shared<std::vector<Eigen::Vector4d>>();
        this->drone_goals_ = std::make_shared<std::vector<Eigen::Vector2d>>();
        this->drone_active_ = std::make_shared<std::vector<bool>>();
        this->drone_radii_ = std::make_shared<std::vector<double>>();
    }

    bool SwarmConfigTracker::num_drones_is_set() {
        if (this->num_drones != -1) {
            return true;
        } else {
            std::cout << "num_drones is not set\n";
            return false;
        }
    }

    bool SwarmConfigTracker::set_num_drones(int drones) {
        if (drones > 0) {
            this->num_drones = drones;
            return true;
        } else {
            return false;
        }
    }

    bool SwarmConfigTracker::write_drone_states(std::vector<Eigen::Vector4d> drone_states) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_states.size()) {
                (*this->drone_states_) = drone_states;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_drone_goals(std::vector<Eigen::Vector2d> drone_goals) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_goals.size()) {
                (*this->drone_goals_) = drone_goals;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_drone_active_vector(std::vector<bool> drone_active) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_active.size()) {
                (*this->drone_active_) = drone_active;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_drone_radii(std::vector<double> drone_radii) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_radii.size()) {
                (*this->drone_radii_) = drone_radii;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_drone_capacities(std::vector<double> drone_capacities) {
        if (this->num_drones_is_set()) {
            if (this->num_drones == drone_capacities.size()) {
                (*this->drone_capacities_) = drone_capacities;
                return true;
            }
        }
        return false;
    }

    bool SwarmConfigTracker::write_swarm_config(std::vector<Eigen::Vector4d> drone_states,
                                                std::vector<Eigen::Vector2d> drone_goals) {
        if (this->num_drones == -1) {
            return false;
        }

        if (drone_states.size() != drone_goals.size()) {
            return false;
        }

        if (this->num_drones != drone_states.size()) {
            return false;
        }

        std::cout << "assigning values to drone_states_ and drone_goals_\n";

        (*this->drone_states_) = drone_states;
        (*this->drone_goals_) = drone_goals;

        return true;
    }

    std::shared_lock<std::shared_mutex> SwarmConfigTracker::read_swarm_config() const {
        return std::shared_lock<std::shared_mutex>(this->swarm_config_mut);
    }
} // namespace swarm_planner
