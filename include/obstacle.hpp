#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sp_const.hpp>
#include <trajectory.hpp>
#include <polynomial.hpp>
#include <utility>
#include <random>

namespace MATP {
    struct Obstacle {
        ros::Time update_time;
        ObstacleType type = ObstacleType::DEFAULT;
        int id = -1;
        double radius = 0;
        double downwash = 0;
        double max_acc = 0;
        point3d position;
        point3d velocity;
        point3d goal_point;
        point3d observed_position;
        Trajectory <point3d> prev_traj; //trajectory of obstacles planned at the previous step

        bool isCollided(const point3d &point, double agent_radius, double horizon, double uncertainty_horizon) const {
            for (double t = 0; t <= horizon; t += std::min(0.1 * horizon, 0.1)) {
                point3d obs_point = position + velocity * t;
                double t_min = std::min(t, uncertainty_horizon);
                if (obs_point.distance(point) < agent_radius + radius + 0.5 * max_acc * t_min * t_min) {
                    return true;
                }
            }

            return false;
        }
    };
    typedef std::vector<Obstacle> Obstacles;

    struct CollisionAlert{
        point3d agent_position;
        Obstacles obstacles;
        void initialize() { obstacles.clear(); }
        bool activated() const { return !obstacles.empty(); }
    };
    typedef std::vector<CollisionAlert> CollisionAlerts;

    class ObstacleBase {
    public:
        //DynamicObstacle has ellipsoidal shape
        ObstacleBase(double _radius, double _max_acc, double _downwash)
                : radius(_radius), max_acc(_max_acc), downwash(_downwash) {
            type = ObstacleType::DEFAULT;
        }

        Obstacle getObstacle(double t) {
            Obstacle obstacle = getObstacle_impl(t);
            obstacle.max_acc = max_acc;
            obstacle.radius = radius;
            obstacle.downwash = downwash;
            return obstacle;
        }

        virtual Obstacle getObstacle_impl(double t) {
            Obstacle obstacle;
            return obstacle;
        }

        [[nodiscard]] double getRadius() const {
            return radius;
        }

        [[nodiscard]] double getMaxAcc() const {
            return max_acc;
        }

        [[nodiscard]] ObstacleType getType() const {
            return type;
        }

        [[nodiscard]] double getDownwash() const {
            return downwash;
        }

        void setRadius(double _radius) {
            radius = _radius;
        }

    protected:
        ObstacleType type;
        double radius;
        double max_acc;
        double downwash;
    };

    class SpinObstacle : public ObstacleBase {
    public:
        SpinObstacle(geometry_msgs::PoseStamped _axis, geometry_msgs::Point _start, double _radius, double _speed,
                     double _max_acc, double _downwash)
                : ObstacleBase(_radius, _max_acc, _downwash), axis(std::move(_axis)), start(_start), speed(_speed) {
            type = ObstacleType::DYN_SPIN;
        }

        Obstacle getObstacle_impl(double t) override {
            return getSpinObstacle(t);
        }

    private:
        geometry_msgs::PoseStamped axis;
        geometry_msgs::Point start;
        double speed;

        Obstacle getSpinObstacle(double t) {
            Obstacle spinObstacle;
            spinObstacle.type = type;
            Eigen::Vector3d a(start.x - axis.pose.position.x, start.y - axis.pose.position.y,
                              start.z - axis.pose.position.z);
            Eigen::Vector3d n(axis.pose.orientation.x, axis.pose.orientation.y, axis.pose.orientation.z);
            n.normalize();
            Eigen::Vector3d r = a - a.dot(n) * n;
            double spin_radius = r.norm();
            double w = speed / spin_radius;
            double theta = w * t;

            if (max_acc < speed * speed / spin_radius) {
                ROS_WARN("[Obstacle] max_acc is smaller than real acc!");
//                throw(PlanningReport::INVALIDARGS);
            }

            // position
            Eigen::Quaterniond q(cos(theta / 2), sin(theta / 2) * n.x(), sin(theta / 2) * n.y(),
                                 sin(theta / 2) * n.z());
            Eigen::Quaterniond p(0, a.x(), a.y(), a.z());
            p = q * p * q.inverse();
            spinObstacle.position.x() = axis.pose.position.x + p.x();
            spinObstacle.position.y() = axis.pose.position.y + p.y();
            spinObstacle.position.z() = axis.pose.position.z + p.z();

            // velocity
            theta = M_PI / 2;
            Eigen::Quaterniond q_vel(cos(theta / 2), sin(theta / 2) * n.x(), sin(theta / 2) * n.y(),
                                     sin(theta / 2) * n.z());
            p = q_vel * p * q_vel.inverse();
            spinObstacle.velocity.x() = w * p.x();
            spinObstacle.velocity.y() = w * p.y();
            spinObstacle.velocity.z() = w * p.z();

            return spinObstacle;
        }
    };

    class StraightObstacle : public ObstacleBase {
    public:
        StraightObstacle() : ObstacleBase(0, 0, 1) {
            type = ObstacleType::DYN_STRAIGHT;
            dist_to_goal = 0;
            dist_acc = 0;
            flight_time = 0;
            speed = 0;
        }

        StraightObstacle(const point3d& _start, const point3d& _goal, double _radius, double _speed,
                         double _max_acc, double _downwash)
                : ObstacleBase(_radius, _max_acc, _downwash), start(_start), goal(_goal), speed(_speed) {
            type = ObstacleType::DYN_STRAIGHT;
            dist_to_goal = start.distance(goal);
            dist_acc = 0.5 * speed * speed / max_acc;
            n = (goal - start).normalized();

            if(dist_to_goal > 2 * dist_acc){
                flight_time = (dist_to_goal - 2 * dist_acc) / speed + 2 * speed / max_acc;
            }
            else{
                flight_time = 2 * sqrt(dist_to_goal / max_acc);
            }
        }

        Obstacle getObstacle_impl(double t) override {
            return getStraightObstacle(t);
        }

        [[nodiscard]] double getFlightTime() const{
            return flight_time;
        }

    private:
        point3d start, goal, n;
        double speed, dist_to_goal, dist_acc, flight_time;

        Obstacle getStraightObstacle(double t) {
            Obstacle straightObstacle;
            straightObstacle.type = type;
            point3d obs_position, obs_velocity;

            if (dist_to_goal > 2 * dist_acc) {
                double t1 = speed / max_acc;
                double t2 = t1 + (dist_to_goal - 2 * dist_acc) / speed;
                double t3 = t1 + t2;
                if (t < t1) {
                    obs_position = start + n * 0.5 * max_acc * t * t;
                    obs_velocity = n * max_acc * t;
                } else if (t < t2) {
                    obs_position = start + n * (0.5 * max_acc * t1 * t1 + speed * (t - t1));
                    obs_velocity = n * speed;
                } else if (t < t3) {
                    obs_position = goal - n * 0.5 * max_acc * (t3 - t) * (t3 - t);
                    obs_velocity = n * (speed - max_acc * (t - t2));
                } else {
                    obs_position = goal;
                }
            } else {
                double t1 = sqrt(dist_to_goal / max_acc);
                double t2 = 2 * t1;
                if (t < t1) {
                    obs_position = start + n * 0.5 * max_acc * t * t;
                    obs_velocity = n * max_acc * t;
                } else if (t < t2) {
                    obs_position = start + n * (0.5 * dist_to_goal + max_acc * t1 * (t - t1) -
                                                0.5 * max_acc * (t - t1) * (t - t1));
                    obs_velocity = n * max_acc * (t2 - t);
                } else {
                    obs_position = goal;
                }
            }

            straightObstacle.position = obs_position;
            straightObstacle.velocity = obs_velocity;

            return straightObstacle;
        }
    };

    class PatrolObstacle : public ObstacleBase {
    public:
        PatrolObstacle(const point3ds& _waypoints, double _radius, double _speed, double _max_acc,
                       double _downwash)
                : ObstacleBase(_radius, _max_acc, _downwash), waypoints(_waypoints), speed(_speed) {
            type = ObstacleType::DYN_PATROL;
            setWaypoints(_waypoints);
        }

        Obstacle getObstacle_impl(double t) override {
            return getPatrolObstacle(t);
        }

        point3ds getWaypoints() {
            return waypoints;
        }

        void setWaypoints(const point3ds& cand_points){
            waypoints.clear();
            for(int i = 0; i < cand_points.size() - 1; i++) {
                point3d prev_direction;
                if(!waypoints.empty()){
                    point3d prev_delta = cand_points[i] - waypoints.back();
                    if(prev_delta.norm_sq() > SP_EPSILON_FLOAT){
                        prev_direction = (cand_points[i] - waypoints.back()).normalized();
                    }
                }

                point3d future_delta = cand_points[i + 1] - cand_points[i];
                point3d future_direction;
                if(!waypoints.empty() and future_delta.norm_sq() > SP_EPSILON_FLOAT) {
                    future_direction = future_delta.normalized();
                }

                // Skip unnecessary waypoints
                if(!waypoints.empty() and future_direction.dot(prev_direction) > 1 - SP_EPSILON_FLOAT) {
                    continue;
                }

                waypoints.emplace_back(cand_points[i]);
                prev_direction = future_direction;
            }
            waypoints.emplace_back(cand_points.back());

            flight_time.resize(waypoints.size());
            straightObstacles.resize(waypoints.size());
            for (size_t i = 0; i < waypoints.size(); i++) {
                if (i == waypoints.size() - 1) {
                    straightObstacles[i] = StraightObstacle(waypoints[i], waypoints[0],
                                                            radius, speed, max_acc, downwash);
                } else {
                    straightObstacles[i] = StraightObstacle(waypoints[i], waypoints[i + 1],
                                                            radius, speed, max_acc, downwash);
                }
                flight_time[i] = straightObstacles[i].getFlightTime();
            }
        }

    private:
        point3ds waypoints;
        std::vector<double> flight_time;
        std::vector<StraightObstacle> straightObstacles;
        double speed;

        Obstacle getPatrolObstacle(double t) {
            Obstacle patrolObstacle;
            patrolObstacle.type = type;
            double current_time = t;
            int current_idx = 0;
            while (current_time >= flight_time[current_idx]) {
                current_time -= flight_time[current_idx];
                if (current_idx == waypoints.size() - 1) {
                    current_idx = 0;
                } else {
                    current_idx++;
                }
            }

            patrolObstacle = straightObstacles[current_idx].getObstacle(current_time);
            return patrolObstacle;
        }
    };

    //Note: cannot used in oracle prediction
    class ChasingObstacle : public ObstacleBase {
    public:
        ChasingObstacle() : ObstacleBase(0, 0, 1) {
            type = ObstacleType::DYN_CHASING;
            t_last_called = 0;
            max_vel = 0;
            gamma_target = 0;
            gamma_obs = 0;
            chasing_target_id = -1;
        }

        ChasingObstacle(const Obstacle &_start_state, double _radius, double _max_vel, double _max_acc,
                        double _gamma_target, double _gamma_obs, double _downwash)
                : ObstacleBase(_radius, _max_acc, _downwash) {
            type = ObstacleType::DYN_CHASING;
            current_state.position = _start_state.position;
            current_state.velocity.x() = 0;
            current_state.velocity.y() = 0;
            current_state.velocity.z() = 0;
            max_vel = _max_vel;
            gamma_target = _gamma_target;
            gamma_obs = _gamma_obs;
            goal_point = current_state.position;
            t_last_called = 0;
            chasing_target_id = -1;
        }

        void setGoalPoint(const point3d &_goal_point) {
            goal_point = _goal_point;
        }

        void setObstacles(const Obstacles &_obstacles) {
            obstacles = _obstacles;
        }

        Obstacle getObstacle_impl(double t) override {
            return getChasingObstacle(t);
        }

    private:
        Obstacle current_state;
        double t_last_called, max_vel, gamma_target, gamma_obs;
        point3d goal_point;
        Obstacles obstacles;
        int chasing_target_id;

        Obstacle getChasingObstacle(double t) {
            Obstacle chasingObstacle;
            chasingObstacle.type = type;
            Eigen::Vector3d delta_goal(goal_point.x() - current_state.position.x(),
                                       goal_point.y() - current_state.position.y(),
                                       goal_point.z() - current_state.position.z());

            Eigen::Vector3d a = gamma_target * delta_goal;
            Eigen::Vector3d v(current_state.velocity.x(),
                              current_state.velocity.y(),
                              current_state.velocity.z());
            double dt = t - t_last_called;

            for (const auto &obstacle: obstacles) {
                Eigen::Vector3d delta_obstacle(obstacle.position.x() - current_state.position.x(),
                                               obstacle.position.y() - current_state.position.y(),
                                               obstacle.position.z() - current_state.position.z());
                double dist_to_obs = delta_obstacle.norm();
                if (dist_to_obs < SP_EPSILON_FLOAT) {
                    continue;
                }
                double Q_star = 2 * (radius + obstacle.radius);
                if (dist_to_obs < Q_star) {
//                    a += gamma_obs * (1/Q_star - 1/dist_to_obs) / (1/(dist_to_obs * dist_to_obs)) * delta_obstacle.normalized();
                    a += gamma_obs * (1 - dist_to_obs / Q_star) * (1 / (dist_to_obs * Q_star)) *
                         (-delta_obstacle.normalized());
                }
            }

            if (a.norm() > (max_acc - 0.01)) {
                a = a / a.norm() * (max_acc - 0.01);
            }
            v = v + a * dt;
            if (v.norm() > max_vel) {
                v = v / v.norm() * max_vel;
            }

            //update current_state
            current_state.position.x() = current_state.position.x() + v(0) * dt;
            current_state.position.y() = current_state.position.y() + v(1) * dt;
            current_state.position.z() = current_state.position.z() + v(2) * dt;
            current_state.velocity.x() = v(0);
            current_state.velocity.y() = v(1);
            current_state.velocity.z() = v(2);

            chasingObstacle.position = current_state.position;
            chasingObstacle.velocity = current_state.velocity;
            chasingObstacle.radius = radius;

            t_last_called = t;
            return chasingObstacle;
        }
    };

    class GaussianObstacle : public ObstacleBase {
    public:
        GaussianObstacle() : ObstacleBase(0, 0, 1) {
            type = ObstacleType::DYN_GAUSSIAN;
            acc_history_horizon = 0;
        }

        GaussianObstacle(point3d _start, double _radius,
                         point3d _initial_vel, double _max_vel,
                         double _stddev_acc, double _max_acc, double _acc_update_cycle,
                         double _downwash)
                : ObstacleBase(_radius, _max_acc, _downwash),
                  start(_start), initial_vel(_initial_vel), max_vel(_max_vel),
                  stddev_acc(_stddev_acc), max_acc(_max_acc), acc_update_cycle(_acc_update_cycle)
        {
            type = ObstacleType::DYN_GAUSSIAN;
            acc_history_horizon = 0;
            update_acc_history(10);
        }

        Obstacle getObstacle_impl(double t) override {
            return getGaussianObstacle(t);
        }

        void set_acc_history(std::vector<Eigen::Vector3d> _acc_history) {
            acc_history = std::move(_acc_history);
        }

    private:
        point3d start;
        std::vector<Eigen::Vector3d> acc_history;
        point3d initial_vel;
        double max_vel;
        double stddev_acc, max_acc, acc_update_cycle;
        double acc_history_horizon;

        void update_acc_history(double desired_horizon) {
            if (acc_history_horizon < desired_horizon) {
                std::random_device rd;
                std::mt19937 generator(rd());
                std::normal_distribution<double> distribution(0.0, stddev_acc);
                int n = ceil((desired_horizon - acc_history_horizon) / acc_update_cycle);
                acc_history_horizon += n * acc_update_cycle;

                for (int i = 0; i < n; i++) {
                    Eigen::Vector3d acc;
                    for (int j = 0; j < 3; j++) {
                        acc(j) = distribution(generator);
                    }
                    if (acc.norm() > max_acc) {
                        acc = acc.normalized() * max_acc;
                    }

                    acc_history.emplace_back(acc);
                }
            }
        }

        Obstacle getGaussianObstacle(double t) {
            Obstacle gaussianObstacle;
            gaussianObstacle.type = type;
            gaussianObstacle.position = start;
            gaussianObstacle.velocity = initial_vel;

            // update acc history
            if (t >= acc_history_horizon) {
                double horizon_update = 10.0;
                update_acc_history(acc_history_horizon + horizon_update);
                acc_history_horizon += horizon_update;
            }

            // backtrack to find current state
            int n = floor((t + SP_EPSILON_FLOAT) / acc_update_cycle);
            if (acc_history.size() < n + 1) {
                ROS_ERROR("acc history size error");
            }
            Eigen::Vector3d v(initial_vel.x(), initial_vel.y(), initial_vel.z());
            Eigen::Vector3d v_next;
            double dt = acc_update_cycle;
            for (int i = 0; i < n + 1; i++) {
                if (i == n) {
                    dt = t - n * acc_update_cycle;
                }

                Eigen::Vector3d acc = acc_history[i];
                v_next = v + acc * dt;
                if (v_next.norm() > max_vel) {
                    // If v is over max_vel then ignore acc
                    gaussianObstacle.position.x() += v(0) * dt;
                    gaussianObstacle.position.y() += v(1) * dt;
                    gaussianObstacle.position.z() += v(2) * dt;
                } else {
                    gaussianObstacle.position.x() += v(0) * dt + 0.5 * acc(0) * dt * dt;
                    gaussianObstacle.position.y() += v(1) * dt + 0.5 * acc(1) * dt * dt;
                    gaussianObstacle.position.z() += v(2) * dt + 0.5 * acc(2) * dt * dt;
                    gaussianObstacle.velocity.x() += acc(0) * dt;
                    gaussianObstacle.velocity.y() += acc(1) * dt;
                    gaussianObstacle.velocity.z() += acc(2) * dt;
                    v = v_next;
                }
            }

            return gaussianObstacle;
        }
    };

    class RealObstacle : public ObstacleBase {
    public:
        RealObstacle() : ObstacleBase(0, 0, 1) {
            type = ObstacleType::DYN_REAL;
        }

        RealObstacle(double _radius, double _max_acc, double _downwash)
            : ObstacleBase(_radius, _max_acc, _downwash)
        {
            type = ObstacleType::DYN_REAL;
        }

        Obstacle getObstacle_impl(double t) override {
            return getRealObstacle();
        }

    private:
        Obstacle getRealObstacle() {
            Obstacle realObstacle;
            realObstacle.type = type;
            return realObstacle;
        }
    };
}
