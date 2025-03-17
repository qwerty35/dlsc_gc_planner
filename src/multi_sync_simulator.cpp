#include <multi_sync_simulator.hpp>

namespace MATP {
    MultiSyncSimulator::MultiSyncSimulator(const ros::NodeHandle &_nh, const Param &_param, const Mission &_mission)
            : nh(_nh), param(_param), mission(_mission), obstacle_generator(_nh, _param, _mission) {
        // Initialize ROS publisher, service, messages
        pub_agent_trajectories = nh.advertise<visualization_msgs::MarkerArray>("/agent_trajectories_history", 1);
        pub_obstacle_trajectories = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_trajectories_history", 1);
        pub_collision_model = nh.advertise<visualization_msgs::MarkerArray>("/collision_model", 1);
        pub_agent_velocities_x = nh.advertise<std_msgs::Float64MultiArray>("/agent_velocities_x", 1);
        pub_agent_velocities_y = nh.advertise<std_msgs::Float64MultiArray>("/agent_velocities_y", 1);
        pub_agent_velocities_z = nh.advertise<std_msgs::Float64MultiArray>("/agent_velocities_z", 1);
        pub_agent_accelerations_x = nh.advertise<std_msgs::Float64MultiArray>("/agent_accelerations_x", 1);
        pub_agent_accelerations_y = nh.advertise<std_msgs::Float64MultiArray>("/agent_accelerations_y", 1);
        pub_agent_accelerations_z = nh.advertise<std_msgs::Float64MultiArray>("/agent_accelerations_z", 1);
        pub_agent_vel_limits = nh.advertise<std_msgs::Float64MultiArray>("/agent_vel_limits", 1);
        pub_agent_acc_limits = nh.advertise<std_msgs::Float64MultiArray>("/agent_acc_limits", 1);
        pub_start_goal_points_vis = nh.advertise<visualization_msgs::MarkerArray>("/start_goal_points", 1);
        pub_world_boundary = nh.advertise<visualization_msgs::MarkerArray>("/world_boundary", 1);
        pub_collision_alert = nh.advertise<visualization_msgs::MarkerArray>("/collision_alert", 1);
        pub_desired_trajs_vis = nh.advertise<visualization_msgs::MarkerArray>("/desired_trajs_vis", 1);
        pub_grid_map = nh.advertise<visualization_msgs::MarkerArray>("/grid_map", 1);
        pub_communication_range = nh.advertise<visualization_msgs::MarkerArray>("/communication_range", 1);
        pub_communication_group = nh.advertise<visualization_msgs::MarkerArray>("/communication_group", 1);
        sub_global_map = nh.subscribe("/octomap_point_cloud_centers", 1, &MultiSyncSimulator::globalMapCallback, this);
        service_start_planning = nh.advertiseService("/start_planning", &MultiSyncSimulator::startPlanningCallback,
                                                     this);
        service_land = nh.advertiseService("/stop_planning", &MultiSyncSimulator::landCallback, this);
        service_patrol = nh.advertiseService("/start_patrol", &MultiSyncSimulator::patrolCallback, this);
        service_stop_patrol = nh.advertiseService("/stop_patrol", &MultiSyncSimulator::stopPatrolCallback, this);

        // ROS messages
        initializeROSMessage();

        // Timer
        sim_start_time = ros::Time::now();
        sim_current_time = sim_start_time;

        // Flags
        is_collided = false;
        has_global_map = false;
        initial_update = true;
        mission_changed = true;
        safety_ratio_agent = SP_INFINITY;
        safety_ratio_obs = SP_INFINITY;
        total_flight_time = SP_INFINITY;
        total_distance = 0;

        // Parameters related to log files
        mission_start_time = std::to_string(sim_start_time.toSec());
        file_name_param = param.getPlannerModeStr() + "_" + std::to_string(mission.qn) + "agents";
        if (param.multisim_save_mission) {
            mission.saveMission(
                    param.package_path + "/missions/previous_missions/mission_" + mission_start_time + ".json");
        }

        // Planner state
        initializePlannerState();

        // Grid based planner
        grid_based_planner = std::make_unique<GridBasedPlanner>(param, mission);

        // Agent
        initializeAgents();

        // Obstacle
        initializeObstacles();
    }

    void MultiSyncSimulator::run() {
        // Main Loop
        for (int iter = 0;
             (iter < param.multisim_max_planner_iteration or param.multisim_experiment) and ros::ok(); iter++) {
            ros::spinOnce();

            // Wait until map is loaded and start signal is arrived
            if (not isPlannerReady()) {
                ros::Rate(10).sleep();
                iter--;
                continue;
            }

            // Check mission finished
            if (isFinished() or (not param.multisim_experiment and iter == param.multisim_max_planner_iteration - 1)) {
                // Save result in csv file
                summarizeResult();

                if (param.multisim_experiment) {
                    // Wait for another mission
                    planner_state = PlannerState::WAIT;
                    iter = 0;
                    initial_update = true;
                    continue;
                } else {
                    // Planning finished
                    break;
                }
            }

            if (initial_update) {
                initializeSimTime();
                initial_update = false;
            } else if (param.multisim_experiment and (sim_current_time - ros::Time::now()).toSec() > 0) {
                // Wait until next planning period
                // Note: planning_start_time = traj_start_time - multisim_time_step
                continue;
            } else {
                doStep();
            }

            // Update collision alert
            updateCollisionAlert();

            // Waypoint planning
            decentralizedMAPP();

            // Update and broadcast agent and obstacle state
            broadcastMsgs();

            // Trajectory planning
            bool success = plan();
            if (not success) {
                break;
            }

            // Publish planning result
            publish();

            if (param.multisim_experiment) {
                // (For experiment) Check planning is finished in time
                double timing_margin = (sim_current_time - ros::Time::now()).toSec();
                if (timing_margin < 0) {
                    //TODO: use prev sol when timing margin < 0
                    ROS_WARN_STREAM("[MultiSyncSimulator] Planning speed is too slow! Delay time: " << timing_margin);
                }
            } else if (param.multisim_planning_rate > 0) {
                // (For simulation) Slow down iteration speed for visualization
                ros::Rate(param.multisim_planning_rate).sleep();
            }
        }
    }

    bool MultiSyncSimulator::isPlannerReady() {
        if (planner_state == PlannerState::WAIT) {
            ROS_INFO_ONCE("[MultiSyncSimulator] Planner ready, wait for start message");
            return false;
        }

        if (param.world_use_octomap and not has_global_map) {
            ROS_INFO_ONCE("[MultiSyncSimulator] Planner ready, wait for global map");
            return false;
        }

        if (param.multisim_experiment and initial_update) {
            for (const auto &agent: agents) {
                if (not agent->isInitialStateValid()) {
                    ROS_WARN_ONCE("[MultiSyncSimulator] Planner ready, but invalid initial state");
                    planner_state = PlannerState::WAIT;
                    return false;
                }
            }
        }

        return true;
    }

    void MultiSyncSimulator::initializeROSMessage() {
        visualization_msgs::Marker default_marker;
        default_marker.header.frame_id = param.world_frame_id;
        default_marker.type = visualization_msgs::Marker::LINE_STRIP;
        default_marker.action = visualization_msgs::Marker::ADD;
        default_marker.pose.position = defaultPoint();
        default_marker.pose.orientation = defaultQuaternion();

        msg_agent_trajectories.markers.clear();
        msg_agent_trajectories.markers.resize(mission.qn);
        for (int qi = 0; qi < mission.qn; qi++) {
            msg_agent_trajectories.markers[qi] = default_marker;
            msg_agent_trajectories.markers[qi].id = qi;
            msg_agent_trajectories.markers[qi].scale.x = 0.07;
            msg_agent_trajectories.markers[qi].color = mission.color[qi];
            msg_agent_trajectories.markers[qi].color.a = 0.75;
        }

        msg_obstacle_trajectories.markers.clear();
        msg_obstacle_trajectories.markers.resize(mission.on);
        for (int oi = 0; oi < mission.on; oi++) {
            msg_obstacle_trajectories.markers[oi] = default_marker;
            msg_obstacle_trajectories.markers[oi].scale.x = 0.07;
            msg_obstacle_trajectories.markers[oi].id = oi;
            msg_obstacle_trajectories.markers[oi].color = defaultColor();
        }
    }

    void MultiSyncSimulator::initializeSimTime() {
        // Initialize planner timing
        // To match the simulation time and real world time, add one time step to simulation time.
        sim_start_time = ros::Time::now() + ros::Duration(param.multisim_time_step);
        sim_current_time = sim_start_time;
    }

    void MultiSyncSimulator::initializePlannerState() {
        if (param.multisim_experiment) {
            planner_state = PlannerState::WAIT;
        } else {
            // Do not wait for start signal in simulation
            if (param.multisim_patrol) {
                planner_state = PlannerState::PATROL;
            } else {
                planner_state = PlannerState::GOTO;
            }
        }
    }

    void MultiSyncSimulator::initializeAgents() {
        agents.resize(mission.qn);
        for (size_t qi = 0; qi < mission.qn; qi++) {
            agents[qi] = std::make_unique<AgentManager>(nh, param, mission, qi);

            // Set initial state
            State initial_state;
            initial_state.position = mission.agents[qi].start_point;
            agents[qi]->setCurrentState(initial_state);

            // Set global map of the agent
            if (param.world_use_octomap and param.world_use_global_map) {
                agents[qi]->setGlobalMap();
                has_global_map = true;
            }
        }
    }

    void MultiSyncSimulator::initializeObstacles() {
        MAPFAgents mapf_obstacles;
        for (int oi = 0; oi < mission.on; oi++) {
            if (mission.obstacles[oi]->getType() != ObstacleType::DYN_PATROL) {
                continue;
            }

            std::shared_ptr<PatrolObstacle> patrol_obstacle_ptr =
                    std::static_pointer_cast<PatrolObstacle>(mission.obstacles[oi]);
            point3ds waypoints = patrol_obstacle_ptr->getWaypoints();
            mapf_obstacles.emplace_back(oi,
                                        waypoints.front(),
                                        waypoints.front(),
                                        waypoints.front(),
                                        waypoints.back());
        }

        if (not mapf_obstacles.empty()) {
            bool success = grid_based_planner->planMAPF(mapf_obstacles,
                                                        agents[0]->getDistmap(), //TODO: agent 0
                                                        mission.agents[0].radius); //TODO: agent 0

            if (success) {
                int path_idx = 0;
                for (int oi = 0; oi < mission.on; oi++) {
                    if (mission.obstacles[oi]->getType() != ObstacleType::DYN_PATROL) {
                        continue;
                    }

                    point3ds new_waypoints = grid_based_planner->getPath(path_idx++);
                    size_t N_path = new_waypoints.size();
                    for (int i = 1; i < N_path + 1; i++) {
                        new_waypoints.emplace_back(new_waypoints[N_path - i]);
                    }
                    obstacle_generator.setWaypoints(oi, new_waypoints);
                }
            }
        }

        obstacle_generator.update(0);
    }

    void MultiSyncSimulator::doStep() {
        sim_current_time += ros::Duration(param.multisim_time_step);

        for (const auto &agent: agents) {
            agent->doStep(param.multisim_time_step);
        }
    }

    void MultiSyncSimulator::updateCollisionAlert() {
        Obstacles obstacles = obstacle_generator.getObstacles();
        for (int qi = 0; qi < mission.qn; qi++) {
            CollisionAlert collision_alert = agents[qi]->getCollisionAlert();
            collision_alert.agent_position = agents[qi]->getCurrentPosition();

            if (obstacles.empty()) {
                collision_alert.obstacles.clear();
            } else {
                for (auto &collision_alert_obstacle: collision_alert.obstacles) {
                    double min_dist = SP_INFINITY;
                    for (const auto &current_obstacle: obstacles) {
                        double dist = collision_alert_obstacle.position.distance(current_obstacle.position);
                        if (dist < min_dist) {
                            min_dist = dist;
                            collision_alert_obstacle = current_obstacle;
                        }
                    }
                }
            }

            agents[qi]->setCollisionAlert(collision_alert);
        }
    }

    void MultiSyncSimulator::decentralizedMAPP() {
        if (param.goal_mode == GoalMode::GRIDBASEDPLANNER) {
            // Ad-hoc network configuration
            // Find network group
            // If the agents are connected each other, they are in the same group
            groups.clear();
            std::set<size_t> initial_group = {0};
            groups.emplace_back(initial_group);
            for (size_t qi = 1; qi < mission.qn; qi++) {
                int group_cand_idx = -1;
                int gi = 0;
                while (gi < groups.size()) {
                    for (const auto &qj: groups[gi]) {
                        double dist = LInfinityDistance(agents[qi]->getCurrentPosition(),
                                                        agents[qj]->getCurrentPosition());
                        if (param.communication_range < 0 or dist < param.communication_range) {
                            if (group_cand_idx == -1) {
                                groups[gi].insert(qi);
                                group_cand_idx = gi;
                            } else {
                                groups[group_cand_idx].merge(groups[gi]);
                                groups.erase(groups.begin() + gi);
                                gi--;
                            }
                            break;
                        }
                    }
                    gi++;
                }

                if (group_cand_idx == -1) {
                    std::set<size_t> new_group = {qi};
                    groups.emplace_back(new_group);
                }
            }

            // Find next_waypoint using grid based planner
            for (const auto &group: groups) {
                ros::Time mapf_start_time = ros::Time::now();
                std::vector<size_t> group_vector(group.begin(), group.end()); // For index search

                // Set start, current, goal points
                size_t group_size = group.size();

                MAPFAgents mapf_agents;
                for (size_t qi: group) {
                    mapf_agents.emplace_back(qi,
                                             agents[qi]->getCurrentPosition(),
                                             agents[qi]->getStartPoint(),
                                             agents[qi]->getNextWaypoint(),
                                             agents[qi]->getDesiredGoalPoint(),
                                             agents[qi]->getCollisionAlert());
                }

                // Set obstacles
                Obstacles obstacles = obstacle_generator.getObstacles();
                for (auto &obstacle: obstacles) {
                    if (param.multisim_experiment and obstacle.type == ObstacleType::DYN_REAL) {
                        State obs_state = agents[0]->getObservedObstacleState(obstacle.id); //TODO: agent 0
                        obstacle.position = obs_state.position;
                        obstacle.velocity = obs_state.velocity;
                    }
                }

                // Run grid_based_planner
                bool success = grid_based_planner->planMAPF(mapf_agents,
                                                            agents[0]->getDistmap(), //TODO: agent 0
                                                            mission.agents[0].radius, //TODO: agent 0, hetero radius
                                                            obstacles);
                if (success) {
                    point3ds desired_waypoints(group_size);
                    for (size_t qgi = 0; qgi < group_size; qgi++) {
                        point3ds path = grid_based_planner->getPath(qgi);
                        int next_waypoint_idx = std::min(1, (int) path.size() - 1);
                        desired_waypoints[qgi] = path[next_waypoint_idx];
                    }

                    std::set<size_t> update_cand_set;
                    for (size_t qi: group) {
                        int qgi = getIndex(group_vector, qi);

                        bool is_in_communication_range = true;
                        if (param.communication_range > 0) {
                            traj_t traj = agents[qi]->getTraj();
                            double dist;
                            for (int m = 0; m < param.M + 1; m++) {
                                if (traj.empty()) {
                                    dist = LInfinityDistance(desired_waypoints[qgi],
                                                             agents[qi]->getCurrentPosition());
                                } else if (m < param.M) {
                                    dist = LInfinityDistance(desired_waypoints[qgi],
                                                             agents[qi]->getTraj()[m].startPoint());
                                } else {
                                    dist = LInfinityDistance(desired_waypoints[qgi],
                                                             agents[qi]->getTraj().lastPoint());
                                }

                                if (dist > 0.5 * param.communication_range - SP_EPSILON_FLOAT) {
                                    is_in_communication_range = false;
                                    break;
                                }
                            }
                        }


                        Line line(agents[qi]->getNextWaypoint(), desired_waypoints[qgi]);
                        if (is_in_communication_range and
                            (desired_waypoints[qgi] - agents[qi]->getNextWaypoint()).norm() > SP_EPSILON_FLOAT and
                            line.includePoint(agents[qi]->getCurrentGoalPoint())) {
                            update_cand_set.insert(qi);
                        }
                    }

                    // Find valid update
                    bool update = false;
                    while (not update and not update_cand_set.empty() and group_size > 1) {
                        for (const auto &qi: update_cand_set) {
                            int qgi = getIndex(group_vector, qi);
                            for (size_t qj: group) {
                                int qgj = getIndex(group_vector, qj);
                                point3d next_waypoint_j;
                                if (qi == qj) {
                                    continue;
                                } else if (update_cand_set.find(qj) == update_cand_set.end()) {
                                    next_waypoint_j = agents[qj]->getNextWaypoint();
                                } else {
                                    next_waypoint_j = desired_waypoints[qgj];
                                }

                                bool is_occupied = desired_waypoints[qgi].distance(next_waypoint_j) < SP_EPSILON_FLOAT;
                                if (is_occupied) {
                                    update_cand_set.erase(qi);
                                    update = false;
                                    break;
                                } else {
                                    update = true;
                                }
                            }

                            if (not update) {
                                break;
                            }
                        }
                    }

                    // Update next_waypoint
                    for (const auto &qi: update_cand_set) {
                        int qgi = getIndex(group_vector, qi);
                        agents[qi]->setNextWaypoint(desired_waypoints[qgi]);
                    }
                } else {
                    ROS_ERROR("[MultiSyncSimulator] MAPF failed");
                }

                ros::Time mapf_end_time = ros::Time::now();
                planning_time.mapf_time.update((mapf_end_time - mapf_start_time).toSec());
            }
        }
    }

    void MultiSyncSimulator::broadcastMsgs() {
        double t = (sim_current_time - sim_start_time).toSec();
        obstacle_generator.update(t);
        Obstacles obstacles = obstacle_generator.getObstacles();

        // Update obstacle's states for each agent
        for (size_t qi = 0; qi < mission.qn; qi++) {
            // Dynamic obstacles
            Obstacles msg_obstacles;
            for (const auto& obstacle: obstacles) {
                msg_obstacles.emplace_back(obstacle);
            }

            // Other agents
            for (size_t qj = 0; qj < mission.qn; qj++) {
                // communication range
                double dist = LInfinityDistance(agents[qi]->getCurrentPosition(), agents[qj]->getCurrentPosition());
                if (qi == qj) {
                    continue;
                }

                if (param.communication_range > 0 and dist > param.communication_range) {
                    continue;
                }

                Obstacle obstacle = agents[qj]->getAgent();
                msg_obstacles.emplace_back(obstacle);

                // Map merging
                if (not param.world_use_global_map) {
                    agents[qi]->mergeMapCallback(agents[qj]->getOctomapMsg());
                }
            }

            agents[qi]->setPlannerState(planner_state);
            agents[qi]->obstacleCallback(msg_obstacles);

            if (mission_changed) {
                agents[qi]->setStartPosition(mission.agents[qi].start_point);
                agents[qi]->setDesiredGoal(mission.agents[qi].desired_goal_point);
            }
        }

        if (mission_changed) {
            mission_changed = false;
        }
    }

    bool MultiSyncSimulator::plan() {
        // Sequential planning
        PlanningReport result;
        for (size_t qi = 0; qi < mission.qn; qi++) {
            result = agents[qi]->plan(sim_current_time);
            if (result == PlanningReport::QPFAILED) {
                return false;
            }
        }

        // save planning result
        if (planner_state != PlannerState::LAND) {
            saveResult();

            if (param.multisim_save_result) {
                saveResultAsCSV();
            }
        }

        return true;
    }

    void MultiSyncSimulator::publish() {
        if (param.log_vis) {
//            publishGridMap();
        }

        if (param.communication_range > 0) {
            publishCommunicationRange();
            publishCommunicationGroup();
        }
        publishAgentTrajectories();
        publishObstacleTrajectories();
        if (planner_state != PlannerState::LAND) {
            publishDesiredTrajs();
        }
        publishCollisionModel();
        publishStartGoalPoints();
        publishWorldBoundary();
        publishCollisionAlert();
        for (size_t qi = 0; qi < mission.qn; qi++) {
            agents[qi]->publish();
        }
        obstacle_generator.publish(param.world_frame_id);
        publishAgentState();
    }

    bool MultiSyncSimulator::isFinished() {
        if (planner_state == PlannerState::PATROL or planner_state == PlannerState::LAND) {
            return false;
        }

        for (size_t qi = 0; qi < mission.qn; qi++) {
            point3d current_position = agents[qi]->getCurrentPosition();
            double dist_to_goal;
            if (planner_state == PlannerState::GOTO) {
                dist_to_goal = current_position.distance(mission.agents[qi].desired_goal_point);
            } else if (planner_state == PlannerState::GOBACK) {
                dist_to_goal = current_position.distance(mission.agents[qi].start_point);
            }

//            ROS_INFO_STREAM("agent:" << qi << ",dist:"<< dist_to_goal << ",thres:" << param.goal_threshold);

            if (dist_to_goal > param.goal_threshold) {
                return false;
            }
        }

        total_flight_time = (sim_current_time - sim_start_time).toSec();
        return true;
    }

    void MultiSyncSimulator::summarizeResult() {
        // total_flight_time
        ROS_INFO_STREAM("[MultiSyncSimulator] total flight time: " << total_flight_time);

        // total flight distance
        total_distance = getTotalDistance();
        ROS_INFO_STREAM("[MultiSyncSimulator] total distance: " << total_distance);

        // mapf running time
        ROS_INFO_STREAM("[MultiSyncSimulator] mapf running time: " << planning_time.mapf_time.average);

        // average planning time
        ROS_INFO_STREAM("[MultiSyncSimulator] planning time per agent: " << planning_time.total_planning_time.average);

        // safety_ratio_agent
        ROS_INFO_STREAM("[MultiSyncSimulator] safety ratio between agent: " << safety_ratio_agent);

        // safety_ratio_obs
        ROS_INFO_STREAM("[MultiSyncSimulator] safety ratio obstacle: " << safety_ratio_obs);

        if (param.multisim_save_result) {
            saveSummarizedResultAsCSV();
        }
    }

    void MultiSyncSimulator::saveResult() {
        is_collided = false;
        double future_time = 0;
        double record_time_step;
        if (param.multisim_experiment) {
            record_time_step = param.multisim_time_step;
        } else {
            record_time_step = param.multisim_save_time_step;
        }

        while (future_time < param.multisim_time_step - SP_EPSILON_FLOAT) {
            point3ds agent_positions(mission.qn);
            for (int qi = 0; qi < mission.qn; qi++) {
                if (param.multisim_experiment) {
                    agent_positions[qi] = agents[qi]->getObservedAgentPosition();
                } else {
                    agent_positions[qi] = agents[qi]->getFutureState(future_time).position;
                }
            }

            point3ds obs_positions(mission.on);
            for (int oi = 0; oi < mission.on; oi++) {
                if (param.multisim_experiment and mission.obstacles[oi]->getType() == ObstacleType::DYN_REAL) {
                    obs_positions[oi] = agents[0]->getObservedObstaclePosition(oi); //TODO: agent 0
                } else if (obstacle_generator.getNumObs() == mission.on) {
                    Obstacles obstacles = obstacle_generator.getObstacles();
                    obs_positions[oi] = obstacles[oi].position;
                }
            }

            // Trajectory message
            for (int qi = 0; qi < mission.qn; qi++) {
                msg_agent_trajectories.markers[qi].points.emplace_back(point3DToPointMsg(agent_positions[qi]));
            }

            for (int oi = 0; oi < mission.on; oi++) {
                msg_obstacle_trajectories.markers[oi].points.emplace_back(point3DToPointMsg(obs_positions[oi]));
            }

            // minimum distance
            for (size_t qi = 0; qi < mission.qn; qi++) {
                // safety_ratio_agent
                double current_safety_ratio_agent = SP_INFINITY;
                int min_qj = -1;
                for (size_t qj = 0; qj < mission.qn; qj++) {
                    if (qi == qj) {
                        continue;
                    }

                    double downwash = (mission.agents[qi].downwash * mission.agents[qi].radius +
                                       mission.agents[qj].downwash * mission.agents[qj].radius) /
                                      (mission.agents[qi].radius + mission.agents[qj].radius);
                    double dist_to_agent = ellipsoidalDistance(agent_positions[qi],
                                                               agent_positions[qj],
                                                               downwash);
                    double safety_ratio = dist_to_agent / (mission.agents[qi].radius + mission.agents[qj].radius);
                    if (safety_ratio < current_safety_ratio_agent) {
                        current_safety_ratio_agent = safety_ratio;
                        min_qj = qj;
                    }
                    if (safety_ratio < safety_ratio_agent) {
                        safety_ratio_agent = safety_ratio;
                    }
                }
                if (current_safety_ratio_agent < 1
                    and agents[qi]->isAgentPoseUpdated()
                    and agents[min_qj]->isAgentPoseUpdated()) {
                    ROS_ERROR_STREAM("[MultiSyncSimulator] collision with agents, agent_id: (" << qi << "," << min_qj
                                                                                               << "), safety_ratio:"
                                                                                               << current_safety_ratio_agent);
                    is_collided = true;
                }

                // safety_ratio_obs
                double current_safety_ratio_obs = SP_INFINITY;
                for (int oi = 0; oi < mission.on; oi++) {
                    double t = (sim_current_time - sim_start_time).toSec();
                    if(param.obs_duration > 0 and t > param.obs_duration){
                        break;
                    }

                    if (mission.obstacles[oi]->getType() == ObstacleType::DYN_REAL) {
                        continue;
                    }

                    double obs_radius = mission.obstacles[oi]->getRadius();
                    double obs_downwash = mission.obstacles[oi]->getDownwash();
                    double downwash = (obs_radius * obs_downwash +
                                       mission.agents[qi].radius * mission.agents[qi].downwash) /
                                      (mission.agents[qi].radius + obs_radius);
                    double dist_to_obs = ellipsoidalDistance(agent_positions[qi],
                                                             obs_positions[oi],
                                                             downwash);
                    double safety_ratio = dist_to_obs / (mission.agents[qi].radius + obs_radius);
                    if (safety_ratio < current_safety_ratio_obs) {
                        current_safety_ratio_obs = safety_ratio;
                    }
                    if (safety_ratio < safety_ratio_obs) {
                        safety_ratio_obs = safety_ratio;
                    }
                }
                if (current_safety_ratio_obs < 1) {
                    ROS_ERROR_STREAM(
                            "[MultiSyncSimulator] collision with obstacles, agent_id:" << qi
                                                                                       << ", current_safety_ratio:"
                                                                                       << current_safety_ratio_obs
                                                                                       << ", planner_seq:"
                                                                                       << agents[qi]->getPlannerSeq());
                    is_collided = true;
                }
            }

            future_time += record_time_step;
        }

        // planning time
        for (size_t qi = 0; qi < mission.qn; qi++) {
            PlanningTimeStatistics agent_planning_time = agents[qi]->getPlanningStatistics().planning_time;
            planning_time.update(agent_planning_time);
        }
    }

    void MultiSyncSimulator::saveResultAsCSV() {
        std::string file_name =
                param.package_path + "/log/result_" + mission_start_time + "_" + file_name_param + ".csv";
        std::ofstream result_csv;
        result_csv.open(file_name, std::ios_base::app);
        if (sim_current_time == sim_start_time) {
            for (size_t qi = 0; qi < mission.qn; qi++) {
                if (param.multisim_experiment) {
                    result_csv << "id,t,px,py,pz,planning_time";
                } else {
                    result_csv << "id,t,px,py,pz,vx,vy,vz,ax,ay,az,planning_time";
                }

                if (qi < mission.qn - 1 or mission.on != 0) {
                    result_csv << ",";
                } else {
                    result_csv << "\n";
                }
            }

            for (size_t oi = 0; oi < mission.on; oi++) {
                result_csv << "obs_id,t,px,py,pz,size";
                if (oi < mission.on - 1) {
                    result_csv << ",";
                } else {
                    result_csv << "\n";
                }
            }
        }

        double record_time_step;
        if (param.multisim_experiment) {
            record_time_step = param.multisim_time_step;
        } else {
            record_time_step = param.multisim_save_time_step;
        }

        double future_time = 0;
        double t = (sim_current_time - sim_start_time).toSec();
        while (future_time < param.multisim_time_step) {
            if (param.multisim_experiment) {
                for (size_t qi = 0; qi < mission.qn; qi++) {
                    point3d agent_position = agents[qi]->getObservedAgentPosition();
                    PlanningStatistics statistics = agents[qi]->getPlanningStatistics();
                    result_csv << qi << "," << t << ","
                               << agent_position.x() << ","
                               << agent_position.y() << ","
                               << agent_position.z() << ","
                               << statistics.planning_time.total_planning_time.current;

                    if (qi < mission.qn - 1 or mission.on != 0) {
                        result_csv << ",";
                    } else {
                        result_csv << "\n";
                    }
                }

                for (int oi = 0; oi < mission.on; oi++) {
                    point3d obs_position = agents[0]->getObservedObstaclePosition(oi); //TODO: agent 0
                    result_csv << oi << ","
                               << t << ","
                               << obs_position.x() << ","
                               << obs_position.y() << ","
                               << obs_position.z() << ","
                               << mission.obstacles[oi]->getRadius();
                    if (oi < mission.on - 1) {
                        result_csv << ",";
                    } else {
                        result_csv << "\n";
                    }
                }
            } else {
                for (size_t qi = 0; qi < mission.qn; qi++) {
                    State future_state = agents[qi]->getFutureState(future_time);
                    PlanningStatistics statistics = agents[qi]->getPlanningStatistics();
                    result_csv << qi << "," << t << ","
                               << future_state.position.x() << ","
                               << future_state.position.y() << ","
                               << future_state.position.z() << ","
                               << future_state.velocity.x() << ","
                               << future_state.velocity.y() << ","
                               << future_state.velocity.z() << ","
                               << future_state.acceleration.x() << ","
                               << future_state.acceleration.y() << ","
                               << future_state.acceleration.z() << ","
                               << statistics.planning_time.total_planning_time.current;

                    if (qi < mission.qn - 1 or mission.on != 0) {
                        result_csv << ",";
                    } else {
                        result_csv << "\n";
                    }
                }

                Obstacles obstacles = obstacle_generator.getObstacles();
                for (const auto &obstacle: obstacles) {
                    result_csv << obstacle.id << ","
                               << t << ","
                               << obstacle.position.x() << ","
                               << obstacle.position.y() << ","
                               << obstacle.position.z() << ","
                               << obstacle.radius;
                    if (obstacle.id < mission.on - 1) {
                        result_csv << ",";
                    } else {
                        result_csv << "\n";
                    }
                }
            }

            future_time += record_time_step;
            t += record_time_step;
        }

        result_csv.close();
    }

    void MultiSyncSimulator::saveSummarizedResultAsCSV() {
        std::string file_name = param.package_path + "/log/summary_" + file_name_param + ".csv";
        std::ifstream result_csv_in(file_name);
        bool print_description = false;
        if (not result_csv_in or result_csv_in.peek() == std::ifstream::traits_type::eof()) {
            print_description = true;
        }

        std::ofstream result_csv_out;
        result_csv_out.open(file_name, std::ios_base::app);
        if (print_description) {
            result_csv_out << "start_time,total_flight_time,total_flight_distance,"
                           << "safety_ratio_agent,safety_ratio_obs,"
                           << "mapf_time_average,mapf_time_min,mapf_time_max,"
                           << "planning_time_average,planning_time_min,planning_time_max,"
                           << "initial_traj_planning_time,obstacle_prediction_time,goal_planning_time,"
                           << "lsc_generation_time,sfc_generation_time,traj_optimization_time,"
                           << "mission_file_name,world_file_name,"
                           << "planner_mode,goal_mode,mapf_mode,"
                           << "communication_range,world_dimension,M,dt\n";
        }
        result_csv_out << mission_start_time << ","
                       << total_flight_time << ","
                       << total_distance << ","
                       << safety_ratio_agent << ","
                       << safety_ratio_obs << ","
                       << planning_time.mapf_time.average << ","
                       << planning_time.mapf_time.min << ","
                       << planning_time.mapf_time.max << ","
                       << planning_time.total_planning_time.average << ","
                       << planning_time.total_planning_time.min << ","
                       << planning_time.total_planning_time.max << ","
                       << planning_time.initial_traj_planning_time.average << ","
                       << planning_time.obstacle_prediction_time.average << ","
                       << planning_time.goal_planning_time.average << ","
                       << planning_time.lsc_generation_time.average << ","
                       << planning_time.sfc_generation_time.average << ","
                       << planning_time.traj_optimization_time.average << ","
                       << mission.current_mission_file_name << ","
                       << mission.current_world_file_name << ","
                       << param.getPlannerModeStr() << ","
                       << param.getGoalModeStr() << ","
                       << param.getMAPFModeStr() << ","
                       << param.communication_range << ","
                       << param.world_dimension << ","
                       << param.M << ","
                       << param.dt << "\n";
        result_csv_out.close();
    }

    double MultiSyncSimulator::getTotalDistance() {
        double dist_total = 0;
        for (size_t qi = 0; qi < mission.qn; qi++) {
            for (int i = 0; i < (int) msg_agent_trajectories.markers[qi].points.size() - 1; i++) {
                dist_total += (pointMsgToPoint3d(msg_agent_trajectories.markers[qi].points[i + 1]) -
                               pointMsgToPoint3d(msg_agent_trajectories.markers[qi].points[i])).norm();
            }
        }
        return dist_total;
    }

    void MultiSyncSimulator::globalMapCallback(const sensor_msgs::PointCloud2 &global_map) {
        if (has_global_map) {
            return;
        }

        for (size_t qi = 0; qi < mission.qn; qi++) {
            agents[qi]->setGlobalMap(global_map);
        }
        has_global_map = true;
    }

    bool MultiSyncSimulator::startPlanningCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        planner_state = PlannerState::GOTO;
        ROS_INFO("[MultiSyncSimulator] Start planning");
        return true;
    }

    bool MultiSyncSimulator::landCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        planner_state = PlannerState::LAND;
        ROS_INFO("[MultiSyncSimulator] Land");
        return true;
    }

    bool MultiSyncSimulator::patrolCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        planner_state = PlannerState::PATROL;
        ROS_INFO("[MultiSyncSimulator] Patrol");
        return true;
    }

    bool MultiSyncSimulator::stopPatrolCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        planner_state = GOBACK;
        ROS_INFO("[MultiSyncSimulator] Back to start position");
        return true;
    }

    void MultiSyncSimulator::publishCollisionModel() {
        visualization_msgs::MarkerArray msg_collision_model;
        msg_collision_model.markers.clear();

        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        for (size_t qi = 0; qi < mission.qn; qi++) {
            if (agents[qi]->getCollisionAlert().obstacles.empty()) {
                marker.color = mission.color[qi];
                marker.color.a = 0.3;
            } else {
                marker.color = mission.color[qi];
                marker.color.a = 1.0;
            }

            marker.scale.x = 2 * mission.agents[qi].radius;
            marker.scale.y = 2 * mission.agents[qi].radius;
            marker.scale.z = 2 * mission.agents[qi].radius * mission.agents[qi].downwash;

            marker.id = qi;
            State current_state = agents[qi]->getCurrentState();
            marker.pose.position = point3DToPointMsg(current_state.position);
            marker.pose.orientation = defaultQuaternion();

            msg_collision_model.markers.emplace_back(marker);
        }
        pub_collision_model.publish(msg_collision_model);
    }

    void MultiSyncSimulator::publishStartGoalPoints() {
        visualization_msgs::MarkerArray msg_start_goal_points_vis;

        for (int qi = 0; qi < mission.qn; qi++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = param.world_frame_id;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color = mission.color[qi];
            marker.color.a = 0.7;

            marker.ns = "start";
            marker.id = qi;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.pose.position = point3DToPointMsg(mission.agents[qi].start_point);
            marker.pose.orientation = point3DToQuaternionMsg(point3d(0, 0, 0));
            msg_start_goal_points_vis.markers.emplace_back(marker);

            point3d agent_desired_goal = agents[qi]->getDesiredGoalPoint();
            marker.ns = "desired_goal";
            marker.id = qi;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.pose.position = point3DToPointMsg(agent_desired_goal);
            marker.pose.orientation = defaultQuaternion();
            msg_start_goal_points_vis.markers.emplace_back(marker);

            point3d agent_current_goal = agents[qi]->getCurrentGoalPoint();
            marker.ns = "current_goal";
            marker.id = qi;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.pose.position = point3DToPointMsg(agent_current_goal);
            marker.pose.orientation = point3DToQuaternionMsg(point3d(0, 0, 0));
            msg_start_goal_points_vis.markers.emplace_back(marker);

            if (param.goal_mode == GoalMode::GRIDBASEDPLANNER) {
                point3d agent_next_waypoint = agents[qi]->getNextWaypoint();
                marker.ns = "next_waypoint";
                marker.id = qi;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.scale.x = 0.05;
                marker.scale.y = 0.08;
                marker.scale.z = 0.0;
                marker.points.clear();
                marker.points.emplace_back(point3DToPointMsg(agent_current_goal));
                marker.points.emplace_back(point3DToPointMsg(agent_next_waypoint));
                marker.pose.position = defaultPoint();
                marker.pose.orientation = point3DToQuaternionMsg(point3d(0, 0, 0));
                msg_start_goal_points_vis.markers.emplace_back(marker);
            }
        }
        pub_start_goal_points_vis.publish(msg_start_goal_points_vis);
    }

    void MultiSyncSimulator::publishWorldBoundary() {
        visualization_msgs::MarkerArray msg_world_boundary;
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        std::vector<double> world_boundary{mission.world_min.x(), mission.world_min.y(), mission.world_min.z(),
                                           mission.world_max.x(), mission.world_max.y(), mission.world_max.z()};

        marker.pose.position = defaultPoint();
        marker.pose.orientation = defaultQuaternion();

        marker.scale.x = 0.03;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        geometry_msgs::Point point_i, point_j;
        std::vector<int> index = {0, 1, 2,
                                  3, 4, 2,
                                  3, 1, 5,
                                  0, 4, 5};
        int offset = 0;
        for (int iter = 0; iter < 4; iter++) {
            point_i.x = world_boundary[index[offset + 0]];
            point_i.y = world_boundary[index[offset + 1]];
            point_i.z = world_boundary[index[offset + 2]];

            for (int i = 0; i < 3; i++) {
                point_j.x = world_boundary[(index[offset + 0] + 3 * (i == 0)) % 6];
                point_j.y = world_boundary[(index[offset + 1] + 3 * (i == 1)) % 6];
                point_j.z = world_boundary[(index[offset + 2] + 3 * (i == 2)) % 6];

                marker.points.emplace_back(point_i);
                marker.points.emplace_back(point_j);
            }
            offset += 3;
        }
        msg_world_boundary.markers.emplace_back(marker);
        pub_world_boundary.publish(msg_world_boundary);
    }

    void MultiSyncSimulator::publishAgentTrajectories() {
        pub_agent_trajectories.publish(msg_agent_trajectories);
    }

    void MultiSyncSimulator::publishObstacleTrajectories() {
        pub_obstacle_trajectories.publish(msg_obstacle_trajectories);
    }

    void MultiSyncSimulator::publishCollisionAlert() {
        visualization_msgs::MarkerArray msg_collision_alert;
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position = point3DToPointMsg((mission.world_max + mission.world_min) * 0.5);
        marker.pose.orientation = defaultQuaternion();

        marker.scale.x = mission.world_max.x() - mission.world_min.x();
        marker.scale.y = mission.world_max.y() - mission.world_min.y();
        marker.scale.z = mission.world_max.z() - mission.world_min.z();

        marker.color.a = 0.0;
        if (is_collided) {
            marker.color.a = 0.3;
        }
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;

        msg_collision_alert.markers.emplace_back(marker);
        pub_collision_alert.publish(msg_collision_alert);
    }

    void MultiSyncSimulator::publishAgentState() {
        std_msgs::Float64MultiArray msg_agent_velocities_x;
        std_msgs::Float64MultiArray msg_agent_velocities_y;
        std_msgs::Float64MultiArray msg_agent_velocities_z;
        std_msgs::Float64MultiArray msg_agent_acceleration_x;
        std_msgs::Float64MultiArray msg_agent_acceleration_y;
        std_msgs::Float64MultiArray msg_agent_acceleration_z;
        std_msgs::Float64MultiArray msg_agent_vel_limits;
        std_msgs::Float64MultiArray msg_agent_acc_limits;

        for (size_t qi = 0; qi < mission.qn; qi++) {
            State msg_current_state = agents[qi]->getCurrentState();
            msg_agent_velocities_x.data.emplace_back(msg_current_state.velocity.x());
            msg_agent_velocities_y.data.emplace_back(msg_current_state.velocity.y());
            msg_agent_velocities_z.data.emplace_back(msg_current_state.velocity.z());
            msg_agent_acceleration_x.data.emplace_back(msg_current_state.acceleration.x());
            msg_agent_acceleration_y.data.emplace_back(msg_current_state.acceleration.y());
            msg_agent_acceleration_z.data.emplace_back(msg_current_state.acceleration.z());
        }
        msg_agent_vel_limits.data.emplace_back(mission.agents[0].max_vel); //TODO: agent 0
        msg_agent_vel_limits.data.emplace_back(-mission.agents[0].max_vel);
        msg_agent_acc_limits.data.emplace_back(mission.agents[0].max_acc);
        msg_agent_acc_limits.data.emplace_back(-mission.agents[0].max_acc);

        pub_agent_velocities_x.publish(msg_agent_velocities_x);
        pub_agent_velocities_y.publish(msg_agent_velocities_y);
        pub_agent_velocities_z.publish(msg_agent_velocities_z);
        pub_agent_accelerations_x.publish(msg_agent_acceleration_x);
        pub_agent_accelerations_y.publish(msg_agent_acceleration_y);
        pub_agent_accelerations_z.publish(msg_agent_acceleration_z);
        pub_agent_vel_limits.publish(msg_agent_vel_limits);
        pub_agent_acc_limits.publish(msg_agent_acc_limits);
    }

    void MultiSyncSimulator::publishDesiredTrajs() {
        // Vis
        visualization_msgs::MarkerArray msg_desired_trajs_vis;
        for (size_t qi = 0; qi < mission.qn; qi++) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = param.world_frame_id;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "traj";
            marker.id = qi;
            marker.scale.x = 0.05;
            marker.color = mission.color[qi];
            marker.color.a = 0.5;
            marker.pose.orientation = defaultQuaternion();

            double dt = 0.1;
            int n_interval = floor((param.M * param.dt + SP_EPSILON) / dt);
            for (int i = 0; i < n_interval; i++) {
                double future_time = i * dt;
                State traj_point = agents[qi]->getFutureState(future_time);
                marker.points.emplace_back(point3DToPointMsg(traj_point.position));
            }
            msg_desired_trajs_vis.markers.emplace_back(marker);

//            // Last point
//            marker.points.clear();
//            marker.type = visualization_msgs::Marker::SPHERE;
//            marker.color.a = 0.3;
//            marker.scale.x = 2 * mission.agents[qi].radius;
//            marker.scale.y = 2 * mission.agents[qi].radius;
//            marker.scale.z = 2 * mission.agents[qi].radius * mission.agents[qi].downwash;
//            for(int m = 0; m < param.M + 1; m++){
//                if(m < param.M){
//                    marker.ns = std::to_string(m);
//                }
//                else{
//                    marker.ns = "last_point";
//                }
//                marker.id = qi;
//                State traj_point = agents[qi]->getFutureState(m * param.dt);
//                marker.pose.position = point3DToPointMsg(traj_point.position);
//                marker.pose.orientation = defaultQuaternion();
//                msg_desired_trajs_vis.markers.emplace_back(marker);
//            }
        }

        pub_desired_trajs_vis.publish(msg_desired_trajs_vis);
    }

//    void MultiSyncSimulator::publishGridMap() {
//        if(agents[0]->getPlannerSeq() > 2){
//            return;
//        }
//
//        if(not param.world_use_octomap) {
//            return;
//        }
//
//        GridBasedPlanner grid_based_planner(distmap_ptr, mission, param);
//        grid_based_planner.plan(mission.agents[0].start_position, mission.agents[0].start_position, 0,
//                                mission.agents[0].radius, mission.agents[0].downwash);
//        point3ds free_grid_points = grid_based_planner.getFreeGridPoints();
//
//        visualization_msgs::MarkerArray msg_grid_map;
//        visualization_msgs::Marker marker;
//        marker.header.frame_id = param.world_frame_id;
//        marker.type = visualization_msgs::Marker::CUBE;
//        marker.action = visualization_msgs::Marker::ADD;
//
//        size_t marker_id = 0;
//        for(const auto& point : free_grid_points){
//            marker.id = marker_id++;
//
//            marker.pose.position = point3DToPointMsg(point);
//            marker.pose.orientation = defaultQuaternion();
//
//            marker.scale.x = 0.02;
//            marker.scale.y = 0.02;
//            marker.scale.z = 0.02;
//
//            marker.color.r = 0;
//            marker.color.g = 0;
//            marker.color.b = 0;
//            marker.color.a = 0.5;
//
//            msg_grid_map.markers.emplace_back(marker);
//        }
//        pub_grid_map.publish(msg_grid_map);
//    }

    void MultiSyncSimulator::publishCommunicationRange() {
        visualization_msgs::MarkerArray msg_communication_range;
        msg_communication_range.markers.clear();

        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "communication_range";

        for (size_t qi = 0; qi < mission.qn; qi++) {
            marker.color = mission.color[qi];
            marker.color.a = 0.1;

            marker.scale.x = 2 * param.communication_range;
            marker.scale.y = 2 * param.communication_range;
            marker.scale.z = 2 * param.communication_range;

            marker.id = qi;
            State current_state = agents[qi]->getCurrentState();
            marker.pose.position = point3DToPointMsg(current_state.position);
            marker.pose.orientation = defaultQuaternion();

            msg_communication_range.markers.emplace_back(marker);
        }

        marker.ns = "trajectory_bound";
        for (size_t qi = 0; qi < mission.qn; qi++) {
            marker.color = mission.color[qi];
            marker.color.a = 0.1;

            marker.scale.x = param.communication_range;
            marker.scale.y = param.communication_range;
            marker.scale.z = param.communication_range;

            marker.id = qi;
            State current_state = agents[qi]->getCurrentState();
            marker.pose.position = point3DToPointMsg(current_state.position);
            marker.pose.orientation = defaultQuaternion();

            msg_communication_range.markers.emplace_back(marker);
        }

        pub_communication_range.publish(msg_communication_range);
    }

    void MultiSyncSimulator::publishCommunicationGroup() {
        visualization_msgs::MarkerArray msg_communication_group;
        msg_communication_group.markers.clear();

        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "communication_route";
        marker.id = 0;
        marker.pose.orientation = defaultQuaternion();
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.3;
        marker.scale.x = 0.03;

        for (size_t qi = 0; qi < mission.qn; qi++) {
            for (size_t qj = qi + 1; qj < mission.qn; qj++) {
                double dist = LInfinityDistance(agents[qi]->getCurrentPosition(), agents[qj]->getCurrentPosition());
                if (dist < param.communication_range) {
                    marker.points.emplace_back(point3DToPointMsg(agents[qi]->getCurrentPosition()));
                    marker.points.emplace_back(point3DToPointMsg(agents[qj]->getCurrentPosition()));
                }
            }
        }
        msg_communication_group.markers.emplace_back(marker);


//        marker.type = visualization_msgs::Marker::SPHERE;
//        marker.ns = "communication_group";
//        marker.points.clear();
//
//        Colors group_colors = getHSVColorMap(groups.size());
//        int group_idx = 0;
//        int count = 0;
//        for(size_t gi = 0; gi < groups.size(); gi++) {
//            marker.color = group_colors[gi];
//            marker.color.a = 0.7;
//
//            for(const auto& qi : groups[gi]) {
//                marker.id = count++;
//                marker.pose.position = point3DToPointMsg(agents[qi]->getCurrentPosition());
//                marker.pose.orientation = defaultQuaternion();
//                marker.scale.x = 2 * mission.agents[qi].radius;
//                marker.scale.y = 2 * mission.agents[qi].radius;
//                marker.scale.z = 2 * mission.agents[qi].radius * mission.agents[qi].downwash;
//                msg_communication_group.markers.emplace_back(marker);
//            }
//        }
        pub_communication_group.publish(msg_communication_group);
    }
}