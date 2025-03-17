#include "agent_manager.hpp"

namespace MATP {
    AgentManager::AgentManager(const ros::NodeHandle &nh, const Param &_param, const Mission &_mission, int agent_id)
            : param(_param), mission(_mission) {
        // Initialize agent
        agent = mission.agents[agent_id];
        agent.current_state.position = mission.agents[agent_id].start_point;
        agent.current_goal_point = agent.current_state.position;
        agent.next_waypoint = agent.current_state.position;

        // Initialize planner states
        planner_state = PlannerState::WAIT;
        collision_alert.initialize();

        // Initialize flags
        has_current_state = false;
        has_obstacles = false;
        has_local_map = false;
        is_disturbed = false;

        //Trajectory Planner
        traj_planner = std::make_unique<TrajPlanner>(nh, param, mission, agent);

        //Map manager
        map_manager = std::make_unique<MapManager>(nh, param, mission, agent_id);

        // Command publisher
        if (param.multisim_experiment) {
            cmd_publisher = std::make_unique<CmdPublisher>(nh, param, mission, agent_id);
        }
    }

    void AgentManager::doStep(double time_step) {
        bool do_step_ideal;
        if (param.multisim_experiment) {
            // Check whether the agent is disturbed
            is_disturbed = cmd_publisher->isDisturbed();
            if (is_disturbed) {
                agent.current_state.position = cmd_publisher->getObservedAgentPosition();
                agent.current_state.velocity = point3d(0, 0, 0);
                agent.current_state.acceleration = point3d(0, 0, 0);
                do_step_ideal = false;
            } else {
                do_step_ideal = true;
            }
        } else {
            do_step_ideal = true;
        }

        if (do_step_ideal) {
            // Update agent's current state with ideal future state
            State ideal_future_state = getFutureState(time_step);
            agent.current_state.position = ideal_future_state.position;
            agent.current_state.velocity = ideal_future_state.velocity;
            agent.current_state.acceleration = ideal_future_state.acceleration;
        }

        if (param.world_dimension == 2) {
            agent.current_state.position.z() = (float) param.world_z_2d;
        }

        // update local map
        if (not param.world_use_global_map) {
            map_manager->updateVirtualLocalMap(agent.current_state.position);
        }

        has_current_state = true;
    }

    PlanningReport AgentManager::plan(ros::Time sim_current_time) {
        // Input check
        if (!has_obstacles || !has_current_state) {
            return PlanningReport::WAITFORROSMSG;
        }

        if (param.multisim_experiment and planner_state == PlannerState::LAND) {
            cmd_publisher->landingCallback();
        } else {
            // Change the desired goal position by the agent's state
            planningStateTransition();

            if (is_disturbed) {
                ROS_WARN_STREAM("[AgentManager] agent " << agent.id << " disturbance detected");
            }

            // Start planning
            TrajOptResult result = traj_planner->plan(agent,
                                                      map_manager->getOctomap(),
                                                      map_manager->getDistmap(),
                                                      sim_current_time,
                                                      is_disturbed);
            desired_traj = result.desired_traj;
            collision_alert = result.collision_alert;
            agent.current_goal_point = traj_planner->getCurrentGoalPosition();

            // Update trajectory in Command publisher
            if (param.multisim_experiment) {
                cmd_publisher->updateTraj(desired_traj, sim_current_time);
            }
        }

        // Re-initialization for replanning
        has_obstacles = false;
        has_current_state = false;

        return PlanningReport::SUCCESS;
    }

    void AgentManager::publish() {
        traj_planner->publish();
        map_manager->publish();
    }

    void AgentManager::publishMap() {
        map_manager->publish();
    }

    void AgentManager::obstacleCallback(const Obstacles &msg_obstacles) {
        Obstacles obstacles = msg_obstacles;
        if (param.multisim_experiment) {
            for (auto &obstacle: obstacles) {
                if (obstacle.type == ObstacleType::DYN_REAL and
                    cmd_publisher->isObsPoseUpdated(obstacle.id)) {
                    State obs_state = cmd_publisher->getObservedObstacleState(obstacle.id);
                    obstacle.position = obs_state.position;
                    obstacle.velocity = obs_state.velocity;
                }
            }
        }

        traj_planner->setObstacles(obstacles);
        has_obstacles = true;
    }

    void AgentManager::mergeMapCallback(const octomap_msgs::Octomap &msg_merge_map) {
        map_manager->mergeMapCallback(msg_merge_map);
    }

    void AgentManager::setCurrentState(const State &msg_current_state) {
        // update agent.current_state
        agent.current_state.position = msg_current_state.position;
        if (param.world_dimension == 2) {
            agent.current_state.position.z() = (float) param.world_z_2d;
        }
        agent.current_state.velocity = msg_current_state.velocity;
        agent.current_state.acceleration = msg_current_state.acceleration;

        // update flag
        has_current_state = true;
    }

    void AgentManager::setPlannerState(const PlannerState &new_planner_state) {
        if (param.multisim_experiment and planner_state == PlannerState::LAND and
            not cmd_publisher->landingFinished()) {
            return;
        }

        planner_state = new_planner_state;
    }

    void AgentManager::setStartPosition(const point3d &_start_position) {
        mission.agents[agent.id].start_point = _start_position;
        agent.start_point = _start_position;
    }

    void AgentManager::setDesiredGoal(const point3d &new_desired_goal_position) {
        mission.agents[agent.id].desired_goal_point = new_desired_goal_position;
        agent.desired_goal_point = new_desired_goal_position;
    }

    void AgentManager::setGlobalMap(){
        map_manager->setGlobalMap();
    }

    void AgentManager::setGlobalMap(const sensor_msgs::PointCloud2 &global_map) {
        map_manager->setGlobalMap(global_map);
    }

    void AgentManager::setNextWaypoint(const point3d& next_waypoint) {
        agent.next_waypoint = next_waypoint;
    }

    void AgentManager::setCollisionAlert(const CollisionAlert& _collision_alert) {
        collision_alert = _collision_alert;
    }

    bool AgentManager::isInitialStateValid() const {
        bool is_valid;
        point3d observed_position;
        if (param.multisim_experiment and cmd_publisher->isAgentPoseUpdated()) {
            observed_position = cmd_publisher->getObservedAgentPosition();
            double dist = observed_position.distance(agent.current_state.position);
            is_valid = dist < param.reset_threshold;
            if (not is_valid) {
                ROS_WARN_STREAM(
                        "observed: " << observed_position <<
                                     ", ideal: " << agent.current_state.position <<
                                     ", dist:" << dist);
            }
        } else {
            is_valid = true; // Use ideal state instead -> valid
        }

        return is_valid;
    }

    bool AgentManager::isAgentPoseUpdated() const {
        return !param.multisim_experiment or cmd_publisher->isAgentPoseUpdated();
    }

    point3d AgentManager::getCurrentPosition() const {
        return agent.current_state.position;
    }

    State AgentManager::getCurrentState() const {
        State msg_current_state;
        msg_current_state.position = agent.current_state.position;
        msg_current_state.velocity = agent.current_state.velocity;
        msg_current_state.acceleration = agent.current_state.acceleration;
        return msg_current_state;
    }

    State AgentManager::getFutureState(double future_time) const {
        State msg_current_state = desired_traj.getStateAt(future_time);
        return msg_current_state;
    }

    PlanningStatistics AgentManager::getPlanningStatistics() const {
        return traj_planner->getPlanningStatistics();
    }

    traj_t AgentManager::getTraj() const {
        return desired_traj;
    }

    int AgentManager::getPlannerSeq() const {
        return traj_planner->getPlannerSeq();
    }

    point3d AgentManager::getCurrentGoalPoint() const {
        return traj_planner->getCurrentGoalPosition();
    }

    point3d AgentManager::getDesiredGoalPoint() const {
        return agent.desired_goal_point;
    }

    Obstacle AgentManager::getAgent() const {
        Obstacle msg_obstacle;
        msg_obstacle.id = agent.id;
        msg_obstacle.type = ObstacleType::AGENT;
        msg_obstacle.position = agent.current_state.position;
        msg_obstacle.velocity = agent.current_state.velocity;
        msg_obstacle.goal_point = agent.current_goal_point;
        msg_obstacle.radius = (float) agent.radius;
        msg_obstacle.downwash = (float) agent.downwash;
        msg_obstacle.max_acc = (float) agent.max_acc;
        if (not desired_traj.empty()) {
            msg_obstacle.prev_traj = desired_traj;
        }
        return msg_obstacle;
    }

    octomap_msgs::Octomap AgentManager::getOctomapMsg() const {
        return map_manager->getLocalOctomapMsg();
    }

    point3d AgentManager::getNextWaypoint() const {
        return agent.next_waypoint;
    }

    std::shared_ptr<DynamicEDTOctomap> AgentManager::getDistmap() const {
        return map_manager->getDistmap();
    }

    point3d AgentManager::getStartPoint() const {
        return agent.start_point;
    }

    CollisionAlert AgentManager::getCollisionAlert() const {
        return collision_alert;
    }

    point3d AgentManager::getObservedAgentPosition() const {
        return cmd_publisher->getObservedAgentPosition();
    }

    point3d AgentManager::getObservedObstaclePosition(int obs_id) const {
        State state = cmd_publisher->getObservedObstacleState(obs_id);
        return state.position;
    }

    State AgentManager::getObservedObstacleState(int obs_id) const {
        return cmd_publisher->getObservedObstacleState(obs_id);
    }

    void AgentManager::planningStateTransition() {
        if (planner_state == PlannerState::GOTO) {
            agent.desired_goal_point = mission.agents[agent.id].desired_goal_point;
        } else if (planner_state == PlannerState::PATROL and
                   agent.desired_goal_point.distance(agent.current_state.position) < param.goal_threshold) {
            // Swap start and goal position
            point3d temp = agent.desired_goal_point;
            agent.desired_goal_point = agent.start_point;
            agent.start_point = temp;
        } else if (planner_state == PlannerState::GOBACK) {
            // Go back to start position
            agent.desired_goal_point = mission.agents[agent.id].start_point;
        }

        // if planner_state == PlannerState::WAIT, use previous desired goal position
    }
}