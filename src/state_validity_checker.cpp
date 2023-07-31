#include "swarm_planner_deps/state_validity_checker.hpp"

double calculate_distance(double x1, double y1, double x2, double y2) {
    double distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    return distance;
}

namespace swarm_planner {
    void SwarmStateValidityChecker::set_swarm_config_tracker(std::shared_ptr<SwarmConfigTracker> swarm_config_tracker) {
        this->swarm_config_tracker_ = swarm_config_tracker;
    }

    void SwarmStateValidityChecker::set_drone_index(int drone_index) {
        this->drone_index_ = drone_index;
    }

    // TODO
    bool SwarmStateValidityChecker::isValid(const ob::State *state) const {
        const auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
        double x = se2_state->getX();
        double y = se2_state->getY();
        // this is the x and y of the sampled state, we are ignoring yaw for now

        std::shared_lock<std::shared_mutex> read_lock = this->swarm_config_tracker_->read_swarm_config();

        bool collision = false;

        Eigen::Vector4d current_drone_state = (*this->swarm_config_tracker_->drone_states_)[this->drone_index_];
        double dist_to_sampled_point = calculate_distance(x, y, current_drone_state[0], current_drone_state[1]);
        double drone_speed = calculate_distance(current_drone_state[2], 0, 0, current_drone_state[3]);
        // check function definition to clarify this, drone_state[2] is x_vel and drone_state[3] is y_vel

        if (drone_speed == 0) {
            drone_speed = 0.1; // to prevent divide-by-zero errors
        }

        double time_to_reach_sampled_state = dist_to_sampled_point / drone_speed;
        // TODO propagate the state of each drone in the for loop by this time and avoid those regions as well.

        for (int i=0; i < this->swarm_config_tracker_->drone_states_->size(); i++) {
            if (i == this->drone_index_) {
                continue;
            } else if ((*this->swarm_config_tracker_->drone_active_)[i] != true) {
                continue;
            }
            // TODO check if drone is active, continue the loop if it isn't
            Eigen::Vector4d drone_state = (*this->swarm_config_tracker_->drone_states_)[i];
            const double drone_x = drone_state[0];
            const double drone_y = drone_state[1];

            const double drone_vx = drone_state[2];
            const double drone_vy = drone_state[3];

            Eigen::Vector2d drone_position = Eigen::Vector2d(drone_x, drone_y);
            Eigen::Vector2d drone_velocity = Eigen::Vector2d(drone_vx, drone_vy);
            Eigen::Vector2d projected_position = drone_position + drone_velocity * time_to_reach_sampled_state;

            double dist_to_future_state = calculate_distance(drone_position[0], drone_position[1], projected_position[0], projected_position[1]);
            double line_slope = (projected_position[1] - drone_position[1]) / (projected_position[0] - drone_position[0]);
            double perpendicular_line_slope = -1/line_slope;

            double A = perpendicular_line_slope;
            double B = -1;

            double Cl1 = -perpendicular_line_slope * drone_position.x() + drone_position.y();
            double Cl2 = -perpendicular_line_slope * projected_position.x() + projected_position.y();
            double l1_dist = std::abs(A*x + B*y + Cl1) / std::sqrt(A*A + B*B);
            double l2_dist = std::abs(A*x + B*y + Cl2) / std::sqrt(A*A + B*B);

            if (l1_dist+l2_dist - dist_to_future_state <= 0.01) {
                A = line_slope;
                double C = -line_slope * drone_position.x() + drone_position.y();
                double l0_dist = std::abs(A*x + B*y + C) / std::sqrt(A*A + B*B);
                if (l0_dist < 0.3) {
                    collision = true;
                    return !collision;
                }
            } else if (calculate_distance(x, y, drone_x, drone_y) < 0.3) {
                // ENHANCEMENT check if the drone will be there in `time_to_reach_sampled_state`
                collision = true;
                return !collision;
            }
        }
        read_lock.unlock();

        return !collision;
    }
}
