#include <traj_planner.hpp>

namespace MATP {
    TrajPlanner::TrajPlanner(const ros::NodeHandle &_nh,
                             const Param &_param,
                             const Mission &_mission,
                             const Agent &_agent)
            : nh(_nh),
              param(_param),
              mission(_mission),
              constraints(_param, _mission, _agent.radius, _agent.max_vel),
              agent(_agent)
    {
        // Initialize useful constants
        buildBernsteinBasis(param.n, B, B_inv);

        // Initialize planner state
        planner_seq = 0;
        is_disturbed = false;
        initialize_sfc = false;
        if (param.planner_mode == PlannerMode::DLSCGC or param.planner_mode == PlannerMode::LSC) {
            initialize_sfc = true;
        }

        // Initialize grid based planner module
        grid_based_planner = std::make_unique<GridBasedPlanner>(param, mission);

        // Initialize trajectory optimization module
        traj_optimizer = std::make_unique<TrajOptimizer>(param, mission, B);

        // Initialize ROS
        initializeROS();
    }

    TrajOptResult TrajPlanner::plan(const Agent &_agent,
                                    const std::shared_ptr <octomap::OcTree> &_octree_ptr,
                                    const std::shared_ptr <DynamicEDTOctomap> &_distmap_ptr,
                                    ros::Time _sim_current_time,
                                    bool _is_disburbed) {
        // Initialize planner
        ros::Time planning_start_time = ros::Time::now();
        agent = _agent;
        sim_current_time = _sim_current_time;
        octree_ptr = _octree_ptr;
        distmap_ptr = _distmap_ptr;
        constraints.setDistmap(distmap_ptr);
        constraints.setOctomap(octree_ptr);

        is_disturbed = _is_disburbed;

        // Start planning
        planner_seq++;
        statistics.planning_seq = planner_seq;
        TrajOptResult result = planImpl();

        // Re-initialization for replanning
        prev_traj = result.desired_traj;

        // Print terminal message
        statistics.planning_time.total_planning_time.update((ros::Time::now() - planning_start_time).toSec());

        return result;
    }

    void TrajPlanner::publish() {
        if(param.log_vis){
//            publishInitialTraj();
//            publishCollisionConstraints();
//            publishGridOccupiedPoints();
            publishFeasibleRegion();
            publishGridPath();
        }
        publishObstaclePrediction();
        publishSFC();
        publishLSC();
    }

    void TrajPlanner::setObstacles(const Obstacles &msg_obstacles) {
        obstacles = msg_obstacles;
    }

    int TrajPlanner::getPlannerSeq() const {
        return planner_seq;
    }

    point3d TrajPlanner::getCurrentGoalPosition() const {
        return agent.current_goal_point;
    }

    PlanningStatistics TrajPlanner::getPlanningStatistics() const {
        return statistics;
    }

    void TrajPlanner::initializeROS() {
        // Initialize ros publisher and subscriber
        std::string prefix = "/mav" + std::to_string(agent.id);
        pub_sfc = nh.advertise<visualization_msgs::MarkerArray>(prefix + "/sfc", 1);
        pub_lsc = nh.advertise<visualization_msgs::MarkerArray>(prefix + "/lsc", 1);
        pub_feasible_region = nh.advertise<visualization_msgs::MarkerArray>("/feasible_region", 1);
        pub_initial_traj_vis = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + "/initial_traj_vis", 1);
        pub_obs_pred_traj_vis = nh.advertise<visualization_msgs::MarkerArray>("/obs_pred_traj_vis", 1);
        pub_grid_path = nh.advertise<visualization_msgs::MarkerArray>("/grid_path_vis", 1);
        pub_grid_occupied_points = nh.advertise<visualization_msgs::MarkerArray>(
                prefix + "/grid_occupied_points", 1);
    }

    TrajOptResult TrajPlanner::planImpl() {
        // Check the current planner mode is valid.
        checkPlannerMode();

        // Plan initial trajectory of the other agents
        obstaclePrediction();

        // Plan initial trajectory of current agent.
        initialTrajPlanning();

        // Construct LSC or BVC
        constructLSC();

        // Construct SFC
        constructSFC();

        // Check the waypoint is trapped by the dynamic obstacle
        checkWaypointTrap();

        // Goal planning
        goalPlanning();

        // Trajectory optimization
        TrajOptResult result = trajOptimization();
        return result;
    }

    void TrajPlanner::checkPlannerMode() {
        // Check planner mode at the first iteration only
        if (planner_seq > 1) {
            return;
        }

        switch (param.planner_mode) {
            case PlannerMode::DLSCGC:
                if (param.multisim_time_step != param.dt) {
                    throw std::invalid_argument("[TrajPlanner] multisim_time_step must be equal to the segment time");
                }
                if (param.prediction_mode != PredictionMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of DLSC_GC must be previous_solution, fix automatically");
                    param.prediction_mode = PredictionMode::PREVIOUSSOLUTION;
                }
                if (param.initial_traj_mode != InitialTrajMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of DLSC_GC must be previous_solution, fix automatically");
                    param.initial_traj_mode = InitialTrajMode::PREVIOUSSOLUTION;
                }
                if(param.slack_mode != SlackMode::NONE) {
                    ROS_WARN("[TrajPlanner] DLSC_GC does not need slack variables, fix to none");
                    param.slack_mode = SlackMode::NONE;
                    traj_optimizer->updateParam(param);
                }
                if(param.goal_mode != GoalMode::GRIDBASEDPLANNER) {
                    ROS_WARN("[TrajPlanner] goal_mode of DLSC_GC must be grid_based_planner, fix automatically");
                    param.goal_mode = GoalMode::GRIDBASEDPLANNER;
                    traj_optimizer->updateParam(param);
                }
                break;
            case PlannerMode::DLSC:
                if (param.multisim_time_step > param.dt) {
                    throw std::invalid_argument("[TrajPlanner] multisim_time_step must be smaller than segment time");
                }
                else if(param.multisim_time_step == param.dt and param.slack_mode != SlackMode::NONE){
                    ROS_WARN("[TrajPlanner] DLSC does not need slack variables when multisim_time_step == segment time, fix SlackMode to none");
                    param.slack_mode = SlackMode::NONE;
                    traj_optimizer->updateParam(param);
                }
                else if(param.multisim_time_step < param.dt and param.slack_mode != SlackMode::CONTINUITY){
                    ROS_WARN("[TrajPlanner] DLSC requires slack variables when multisim_time_step < segment time, fix SlackMode to dynamical_limit");
                    param.slack_mode = SlackMode::CONTINUITY;
                    traj_optimizer->updateParam(param);
                }

                if (param.prediction_mode != PredictionMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of LSC must be previous_solution, auto fixed");
                    param.prediction_mode = PredictionMode::PREVIOUSSOLUTION;
                }
                if (param.initial_traj_mode != InitialTrajMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of LSC must be previous_solution, , auto fixed");
                    param.initial_traj_mode = InitialTrajMode::PREVIOUSSOLUTION;
                }
                break;
            case PlannerMode::LSC:
                if (param.multisim_time_step != param.dt) {
                    throw std::invalid_argument("[TrajPlanner] multisim_time_step must be equal to the segment time");
                }
                if (param.prediction_mode != PredictionMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of LSC must be previous_solution, fix automatically");
                    param.prediction_mode = PredictionMode::PREVIOUSSOLUTION;
                }
                if (param.initial_traj_mode != InitialTrajMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of LSC must be previous_solution, fix automatically");
                    param.initial_traj_mode = InitialTrajMode::PREVIOUSSOLUTION;
                }
                if(param.slack_mode != SlackMode::NONE){
                    ROS_WARN("[TrajPlanner] LSC does not need slack variables, fix to none");
                    param.slack_mode = SlackMode::NONE;
                    traj_optimizer->updateParam(param);
                }
                break;
            case PlannerMode::BVC:
                if (param.prediction_mode != PredictionMode::POSITION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of BVC must be current_position, fix automatically");
                    param.prediction_mode = PredictionMode::POSITION;
                }
                if (param.initial_traj_mode != InitialTrajMode::POSITION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of LSC must be current_position, fix automatically");
                    param.initial_traj_mode = InitialTrajMode::POSITION;
                }
                break;
            case PlannerMode::ORCA:
                break;
            case PlannerMode::RECIPROCALRSFC:
                if (param.prediction_mode == PredictionMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] prediction_mode of Reciprocal RSFC must be current_position, fix to velocity automatically");
                    param.prediction_mode = PredictionMode::VELOCITY;
                }
                if (param.initial_traj_mode == InitialTrajMode::PREVIOUSSOLUTION) {
                    ROS_WARN("[TrajPlanner] init_traj_mode of Reciprocal RSFC must be current_position, fix to ORCA automatically");
                    param.initial_traj_mode = InitialTrajMode::ORCA;
                }
                if (param.slack_mode != SlackMode::COLLISIONCONSTRAINT) {
                    ROS_WARN("[TrajPlanner] Reciprocal RSFC needs slack variables at collision constraints");
                    param.slack_mode = SlackMode::COLLISIONCONSTRAINT;
                    traj_optimizer->updateParam(param);
                }
                break;
        }

        if (param.world_use_octomap and distmap_ptr == nullptr) {
            throw std::invalid_argument("[TrajPlanner] Distmap is not ready");
        }
    }


    void TrajPlanner::obstaclePrediction() {
        // Timer start
        ros::Time obs_pred_start_time = ros::Time::now();

        // Initialize obstacle predicted trajectory
        size_t N_obs = obstacles.size();
        obs_pred_trajs.resize(N_obs);
        obs_pred_sizes.resize(N_obs);

        switch (param.prediction_mode) {
            case PredictionMode::POSITION:
                obstaclePredictionWithCurrPos();
                break;
            case PredictionMode::VELOCITY:
                obstaclePredictionWithCurrVel();
                break;
            case PredictionMode::PREVIOUSSOLUTION:
                obstaclePredictionWithPrevSol();
                break;
            default:
                throw std::invalid_argument("[TrajPlanner] Invalid obstacle prediction mode");
        }

        checkObstacleDisturbance();
        obstacleSizePredictionWithConstAcc();

        // Timer end
        ros::Time obs_pred_end_time = ros::Time::now();
        statistics.planning_time.obstacle_prediction_time.update((obs_pred_end_time - obs_pred_start_time).toSec());
    }

    void TrajPlanner::obstaclePredictionWithCurrPos() {
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            obs_pred_trajs[oi] = Trajectory<point3d>(param.M, param.n, param.dt);
            obs_pred_trajs[oi].planConstVelTraj(obstacles[oi].position, point3d(0, 0, 0));
        }
    }

    void TrajPlanner::obstaclePredictionWithCurrVel() {
        // Obstacle prediction with constant velocity assumption
        // It assumes that correct position and velocity are given
        size_t N_obs = obstacles.size();
        for (size_t oi = 0; oi < N_obs; oi++) {
            obs_pred_trajs[oi] = Trajectory<point3d>(param.M, param.n, param.dt);
            obs_pred_trajs[oi].planConstVelTraj(obstacles[oi].position, obstacles[oi].velocity);
        }
    }

    void TrajPlanner::obstaclePredictionWithPrevSol() {
        // Dynamic obstacle -> constant velocity, Agent -> prev sol
        // Use current velocity to predict the obstacle while the first iteration
        if (planner_seq < 2) {
            obstaclePredictionWithCurrVel();
            return;
        }

        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            obs_pred_trajs[oi] = Trajectory<point3d>(param.M, param.n, param.dt);

            if (obstacles[oi].type != ObstacleType::AGENT) {
                // if the obstacle is not agent, use current velocity to predict trajectory
                obs_pred_trajs[oi].planConstVelTraj(obstacles[oi].position, obstacles[oi].velocity);
            } else if (param.multisim_time_step == param.dt) {
                // feasible LSC: generate C^n-continuous LSC
                for (int m = 0; m < param.M; m++) {
                    if (m == param.M - 1) {
                        for (int i = 0; i < param.n + 1; i++) {
                            obs_pred_trajs[oi][m][i] = obstacles[oi].prev_traj[m][param.n];
                        }
                    } else {
                        obs_pred_trajs[oi][m] = obstacles[oi].prev_traj[m + 1];
                    }
                }
            } else if (param.multisim_time_step < param.dt) {
                // relaxed LSC: generate C^0-continuous LSC
                for (int m = 0; m < param.M; m++) {
                    obs_pred_trajs[oi][m] = obstacles[oi].prev_traj[m];
                    if (m == 0) {
                        obs_pred_trajs[oi][m] = obstacles[oi].prev_traj[m].subSegment(param.multisim_time_step / param.dt, 1, B, B_inv);
                    }
                }
            } else {
                throw std::invalid_argument("[TrajPlanner] obs_pred_prev_sol supports only LSC, LSC2");
            }
        }
    }

    void TrajPlanner::checkObstacleDisturbance() {
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            point3d obs_position = obstacles[oi].position;
            if ((obs_pred_trajs[oi].startPoint() - obs_position).norm() > param.reset_threshold) {
                obs_pred_trajs[oi].planConstVelTraj(obs_position, point3d(0, 0, 0));
            }
        }
    }

    void TrajPlanner::obstacleSizePredictionWithConstAcc() {
        int M_uncertainty = std::min(static_cast<int>((param.obs_uncertainty_horizon + SP_EPSILON) / param.dt), param.M);
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            obs_pred_sizes[oi] = Trajectory<double>(param.M, param.n, param.dt);
            if (param.obs_size_prediction and (param.planner_mode == PlannerMode::RECIPROCALRSFC or
                                               obstacles[oi].type != ObstacleType::AGENT)) {
                // Predict obstacle size using max acc
                double max_acc;
                max_acc = obstacles[oi].max_acc;
                for (int m = 0; m < M_uncertainty; m++) {
                    Eigen::MatrixXd coef = Eigen::MatrixXd::Zero(1, param.n + 1);
                    coef(0, 0) = 0.5 * max_acc * pow(m * param.dt, 2);
                    coef(0, 1) = max_acc * m * param.dt * param.dt;
                    coef(0, 2) = 0.5 * max_acc * pow(param.dt, 2);

                    Eigen::MatrixXd control_points = coef * B_inv;
                    for (int i = 0; i < param.n + 1; i++) {
                        obs_pred_sizes[oi][m][i] = obstacles[oi].radius + control_points(0, i);
                    }
                }
                for (int m = M_uncertainty; m < param.M; m++) {
                    for (int i = 0; i < param.n + 1; i++) {
                        obs_pred_sizes[oi][m][i] = obstacles[oi].radius +
                                                   0.5 * max_acc * pow(M_uncertainty * param.dt, 2);
                    }
                }
            } else {
                obs_pred_sizes[oi].planConstVelTraj(obstacles[oi].radius, 0);
            }
        }
    }

    void TrajPlanner::initialTrajPlanning() {
        // Timer start
        ros::Time init_traj_planning_start_time = ros::Time::now();

        initial_traj = Trajectory<point3d>(param.M, param.n, param.dt);
        switch (param.initial_traj_mode) {
            case InitialTrajMode::POSITION:
                initialTrajPlanningCurrPos();
                break;
            case InitialTrajMode::VELOCITY:
                initialTrajPlanningCurrVel();
                break;
            case InitialTrajMode::PREVIOUSSOLUTION:
                initialTrajPlanningPrevSol();
                break;
            case InitialTrajMode::SKIP:
                // skip
                break;
            default:
                throw std::invalid_argument("[TrajPlanner] Invalid initial traj_curr planner mode");
        }

        // Check whether the agent is disturbed
        initialTrajPlanningCheck();

        // Timer end
        ros::Time init_traj_planning_end_time = ros::Time::now();
        statistics.planning_time.initial_traj_planning_time.update(
                (init_traj_planning_end_time - init_traj_planning_start_time).toSec());
    }

    void TrajPlanner::initialTrajPlanningCurrPos() {
        initial_traj.planConstVelTraj(agent.current_state.position, point3d(0, 0, 0));
    }

    void TrajPlanner::initialTrajPlanningCurrVel() {
        initial_traj.planConstVelTraj(agent.current_state.position, agent.current_state.velocity);
    }

    void TrajPlanner::initialTrajPlanningPrevSol() {
        if (planner_seq < 2) {
            initialTrajPlanningCurrVel();
        } else if (param.multisim_time_step == param.dt) {
            for (int m = 0; m < param.M; m++) {
                if (m == param.M - 1) {
                    for (int i = 0; i < param.n + 1; i++) {
                        initial_traj[m][i] = prev_traj[m][param.n];
                    }
                } else {
                    initial_traj[m] = prev_traj[m + 1];
                }
            }
        } else if (param.multisim_time_step < param.dt) {
            for (int m = 0; m < param.M; m++) {
                if (m == 0) {
                    initial_traj[m] = prev_traj[m].subSegment(param.multisim_time_step / param.dt, 1, B, B_inv);
                    initial_traj[m].segment_time = param.dt;
                }
                else{
                    initial_traj[m] = prev_traj[m];
                }
            }
        }
    }

    void TrajPlanner::initialTrajPlanningCheck() {
        // If the agent is disturbed, consider all agents as dynamic obstacles.
        if (is_disturbed) {
            initialTrajPlanningCurrPos();
            initialize_sfc = true;
        }
    }

    void TrajPlanner::goalPlanning() {
        // Timer start
        ros::Time goal_planning_start_time = ros::Time::now();

        if (is_disturbed) {
            agent.current_goal_point = agent.current_state.position;
            return;
        }

        switch (param.goal_mode) {
            case GoalMode::STATIC:
                goalPlanningWithStaticGoal();
                break;
            case GoalMode::RIGHTHAND:
                goalPlanningWithRightHandRule();
                break;
            case GoalMode::PRIORBASED:
//                goalPlanningWithPriority();
                throw std::invalid_argument("[TrajPlanner] Invalid goal mode");
                break;
            case GoalMode::GRIDBASEDPLANNER:
                goalPlanningWithGridBasedPlanner();
                break;
            default:
                throw std::invalid_argument("[TrajPlanner] Invalid goal mode");
                break;
        }

        // Timer end
        ros::Time goal_planning_end_time = ros::Time::now();
        statistics.planning_time.goal_planning_time.update((goal_planning_end_time - goal_planning_start_time).toSec());
    }

    void TrajPlanner::goalPlanningWithStaticGoal() {
        agent.current_goal_point = agent.desired_goal_point;
    }

    void TrajPlanner::goalPlanningWithRightHandRule() {
        // If the agent detect deadlock, then change the goal point to the right.
        if (isDeadlock()) {
            point3d z_axis(0, 0, 1);
            agent.current_goal_point = agent.current_state.position +
                                       (agent.desired_goal_point - agent.current_state.position).cross(z_axis);
        } else {
            goalPlanningWithStaticGoal();
        }
    }

    void TrajPlanner::goalPlanningWithGridBasedPlanner() {
        //update current_goal_point
        GoalOptimizer goal_optimizer(param, mission);
        agent.current_goal_point = goal_optimizer.solve(agent, constraints,
                                                        agent.current_goal_point, agent.next_waypoint);
    }

    void TrajPlanner::constructLSC() {
        // LSC (or BVC) construction
        ros::Time lsc_start_time = ros::Time::now();
        constraints.initializeLSC(obstacles);
        if (param.planner_mode == PlannerMode::DLSCGC){
            generateDLSCGC();
        } else if (param.planner_mode == PlannerMode::DLSC or param.planner_mode == PlannerMode::LSC) { // RAL 2022
            generateLSC();
        } else if (param.planner_mode == PlannerMode::RECIPROCALRSFC) { // RAL 2021
            generateReciprocalRSFC();
        } else if (param.planner_mode == PlannerMode::BVC) {
            generateBVC();
        } else {
            throw std::invalid_argument("[TrajPlanner] Invalid planner mode");
        }
        ros::Time lsc_end_time = ros::Time::now();
        statistics.planning_time.lsc_generation_time.update((lsc_end_time - lsc_start_time).toSec());
    }

    void TrajPlanner::constructSFC() {
        // SFC construction
        if (param.world_use_octomap) {
            ros::Time sfc_start_time = ros::Time::now();
            generateSFC();
            ros::Time sfc_end_time = ros::Time::now();
            statistics.planning_time.sfc_generation_time.update((sfc_end_time - sfc_start_time).toSec());
        }
    }

    void TrajPlanner::generateReciprocalRSFC() {
        double closest_dist;
        point3d normal_vector;

        for (int oi = 0; oi < obstacles.size(); oi++) {
            double downwash = downwashBetween(oi);
            for (int m = 0; m < param.M; m++) {
                Line obs_path(obs_pred_trajs[oi][m][0], obs_pred_trajs[oi][m][param.n]);
                Line agent_path(initial_traj[m][0], initial_traj[m][param.n]);
                normal_vector = normalVectorBetweenLines(obs_path, agent_path, closest_dist);
                normal_vector.z() = normal_vector.z() / (downwash * downwash);

                for (int i = 0; i < param.n + 1; i++) {
                    double d;
                    if (obstacles[oi].type == ObstacleType::AGENT and
                        closest_dist < obs_pred_sizes[oi][m][i] + agent.radius) {
                        d = 0.5 * (obs_pred_sizes[oi][m][i] + agent.radius + closest_dist);
                    } else {
                        d = obs_pred_sizes[oi][m][i] + agent.radius;
                    }

                    LSC lsc(obs_pred_trajs[oi][m][i], normal_vector, d);
                    constraints.setLSC(oi, m, i, lsc);
                }
            }
        }
    }

    void TrajPlanner::generateLSC() {
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            // Coordinate transformation
            double downwash = downwashBetween(oi);
            traj_t initial_traj_trans = initial_traj.coordinateTransform(downwash);
            traj_t obs_pred_traj_trans = obs_pred_trajs[oi].coordinateTransform(downwash);
            double collision_dist = obstacles[oi].radius + agent.radius;

            // Normal vector planning
            // Compute normal vector of LSC
            for (int m = 0; m < param.M; m++) {
                point3d normal_vector_trans;
                if (obstacles[oi].type == ObstacleType::AGENT) {
                    normal_vector_trans = normalVectorBetweenPolys(oi, m, initial_traj_trans, obs_pred_traj_trans);
                    if (normal_vector_trans.norm() < SP_EPSILON_FLOAT) {
                        if (obstacles[oi].type == ObstacleType::AGENT) {
                            ROS_WARN("[TrajPlanner] normal_vector is 0");
                        }

                        point3d vector_obs_to_agent = coordinateTransform(
                                agent.current_goal_point - obstacles[oi].position, downwash);
                        normal_vector_trans = vector_obs_to_agent.normalized();
                    }
                } else {
                    normal_vector_trans = normalVectorDynamicObs(oi, m, downwash);
                }

                point3d normal_vector = normal_vector_trans;
                normal_vector.z() = normal_vector.z() / downwash;

                // Compute safety margin
                for(int i = 0; i < param.n + 1; i++) {
                    double d;
                    if (obstacles[oi].type == ObstacleType::AGENT and not constraints.isDynamicObstacle(oi)) {
                        d = 0.5 * (collision_dist +
                                   (initial_traj_trans[m][i] - obs_pred_traj_trans[m][i]).dot(normal_vector_trans));
                    } else {
                        d = obs_pred_sizes[oi][m][i] + agent.radius;
                    }


                    LSC lsc(obs_pred_trajs[oi][m][i], normal_vector, d);
                    constraints.setLSC(oi, m, i, lsc);
                }
            }
        }
    }

    void TrajPlanner::generateDLSCGC() {
        for (int oi = 0; oi < obstacles.size(); oi++) {
            double collision_dist = obstacles[oi].radius + agent.radius;

            // Coordinate transformation
            double downwash = downwashBetween(oi);
            traj_t initial_traj_trans = initial_traj.coordinateTransform(downwash);
            traj_t obs_pred_traj_trans = obs_pred_trajs[oi].coordinateTransform(downwash);
            point3d obs_goal_trans = coordinateTransform(obstacles[oi].goal_point, downwash);
            point3d agent_goal_trans = coordinateTransform(agent.current_goal_point, downwash);

            // Generate LSC
            for (int m = 0; m < param.M; m++) {
                if(obstacles[oi].type != ObstacleType::AGENT){
                    point3d normal_vector_trans = normalVectorDynamicObs(oi, m, downwash);
                    point3d normal_vector(normal_vector_trans.x(),
                                          normal_vector_trans.y(),
                                          normal_vector_trans.z() / downwash);

                    // Compute safety margin
                    for (int i = 0; i < param.n + 1; i++) {
                        double d = obs_pred_sizes[oi][m][i] + agent.radius;
                        LSC lsc(obs_pred_trajs[oi][m][i], normal_vector, d);
                        constraints.setLSC(oi, m, i, lsc);
                    }
                } else if (m < param.M - 1) {
                    point3d normal_vector_trans = normalVectorBetweenPolys(oi, m, initial_traj_trans, obs_pred_traj_trans);
                    point3d normal_vector(normal_vector_trans.x(),
                                          normal_vector_trans.y(),
                                          normal_vector_trans.z() / downwash);

                    // Compute safety margin
                    for (int i = 0; i < param.n + 1; i++) {
                        double d = 0.5 * (collision_dist +
                                      (initial_traj_trans[m][i] - obs_pred_traj_trans[m][i]).dot(normal_vector_trans));
                        LSC lsc(obs_pred_trajs[oi][m][i], normal_vector, d);
                        constraints.setLSC(oi, m, i, lsc);
                    }
                } else {
                    Line line1(obs_pred_traj_trans.lastPoint(), obs_goal_trans);
                    Line line2(initial_traj_trans.lastPoint(), agent_goal_trans);
                    ClosestPoints closest_points = closestPointsBetweenLineSegments(line1, line2);
                    point3d normal_vector_trans =
                            (closest_points.closest_point2 - closest_points.closest_point1).normalized();
                    point3d obs_control_point_trans = closest_points.closest_point1;

                    // Compute safety margin
                    double d = 0.5 * (collision_dist + closest_points.dist);

                    // Return to original coordination
                    point3d normal_vector(normal_vector_trans.x(),
                                          normal_vector_trans.y(),
                                          normal_vector_trans.z() / downwash);
                    point3d obs_control_point = obs_control_point_trans;
                    obs_control_point.z() = obs_control_point.z() * downwash;

                    LSC lsc(obs_control_point, normal_vector, d);
                    for (int i = 0; i < param.n + 1; i++) {
                        constraints.setLSC(oi, m, i, lsc);
                    }
                }
            }
        }
    }

    void TrajPlanner::generateBVC() {
        // Since the original BVC does not consider downwash, we conduct coordinate transformation to consider downwash.
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            // Coordinate transformation
            double downwash = downwashBetween(oi);
            double collision_dist = obstacles[oi].radius + agent.radius;
            traj_t initial_traj_trans = initial_traj.coordinateTransform(downwash);
            traj_t obs_pred_traj_trans = obs_pred_trajs[oi].coordinateTransform(downwash);
            point3d normal_vector_trans = (initial_traj_trans.startPoint() - obs_pred_traj_trans.startPoint()).normalized();
            point3d normal_vector = normal_vector_trans;
            normal_vector.z() = normal_vector.z() / downwash;

            for(int m = 0; m < param.M; m++) {
                for (int i = 0; i < param.n + 1; i++) {
                    double d = 0.5 * (collision_dist +
                                      (initial_traj_trans.startPoint() - obs_pred_traj_trans.startPoint()).dot(
                                              normal_vector_trans));
                    LSC lsc(obs_pred_trajs[oi][m][i], normal_vector, d);
                    constraints.setLSC(oi, m, i, lsc);
                }
            }
        }
    }

    void TrajPlanner::generateSFC() {
        if (initialize_sfc) {
            constraints.initializeSFC(agent.current_state.position);
            initialize_sfc = false;
        } else {
            if(param.goal_mode == GoalMode::GRIDBASEDPLANNER) {
                constraints.constructSFCFromInitialTraj(initial_traj,
                                                        agent.current_goal_point,
                                                        agent.next_waypoint);
                constraints.constructCommunicationRange(agent.next_waypoint);
            } else {
                constraints.constructSFCFromPoint(initial_traj.lastPoint(), agent.current_goal_point);
            }
        }
    }

    void TrajPlanner::checkWaypointTrap() {
        if(param.planner_mode != PlannerMode::DLSCGC or param.goal_mode != GoalMode::GRIDBASEDPLANNER or obstacles.empty()) {
            return;
        }

        // If the waypoint is trapped, ignore dynamic obstacles
        bool is_waypoint_trapped = !(constraints.isPointInFeasibleRegion(agent.current_goal_point, param.M - 1, param.n)
                                     and constraints.isPointInFeasibleRegion(agent.next_waypoint, param.M - 1, param.n));
        if (is_waypoint_trapped) {
            for (int oi = 0; oi < obstacles.size(); oi++) {
                if (obstacles[oi].type == ObstacleType::AGENT) {
                    continue;
                }
                bool is_waypoint_collided = obstacles[oi].isCollided(agent.next_waypoint,
                                                                     agent.radius,
                                                                     param.M * param.dt,
                                                                     param.obs_uncertainty_horizon);
                if (is_waypoint_collided) {
                    for (int m = 0; m < param.M; m++) {
                        for (int i = 0; i < param.n + 1; i++) {
                            LSC lsc;
                            constraints.setLSC(oi, m, i, lsc); // default lsc is ignored at the traj optimization step.
                        }
                    }
                }
            }
        }
    }

    TrajOptResult TrajPlanner::trajOptimization() {
        Timer timer;
        TrajOptResult result;

        // Solve QP problem using CPLEX
        timer.reset();
        try {
            result = traj_optimizer->solve(agent, constraints, initial_traj, true);
            if (param.planner_mode == PlannerMode::DLSC and not isSolValid(result)) {
                ROS_WARN("[TrajPlanner] Rerun the solver with default algorithm");
                result = traj_optimizer->solve(agent, constraints, initial_traj, false);
            }
        } catch (...) {
            // Debug
            for (int m = 0; m < param.M; m++) {
                if(param.world_use_octomap) {
                    Box sfc = constraints.getSFC(m);
                    for (const auto &control_point: initial_traj[m].control_points) {
                        bool sfc_check = sfc.isPointInBox(control_point);
                        if (not sfc_check) {
                            ROS_ERROR_STREAM("[TrajPlanner] SFC constraint is not feasible. m: " << m);
                        }
                    }
                }
                for (size_t oi = 0; oi < obstacles.size(); oi++) {
                    for (int i = 0; i < param.n + 1; i++) {
                        LSC lsc = constraints.getLSC(oi, m, i);
                        bool lsc_check = lsc.isPointInLSC(initial_traj[m][i]);
                        if (not lsc_check) {
                            ROS_ERROR_STREAM("[TrajPlanner] LSC constraint is not feasible." <<
                                                                                             " oi: " << oi <<
                                                                                             ", m: " << m <<
                                                                                             ", i: " << i);
                        }
                    }
                }
            }

            //Failsafe
            result.desired_traj = initial_traj;
        }

        timer.stop();
        statistics.planning_time.traj_optimization_time.update(timer.elapsedSeconds());

        return result;
    }

    void TrajPlanner::publishSFC(){
        visualization_msgs::MarkerArray msg_sfc;
        msg_sfc = constraints.convertSFCsToMarkerArrayMsg(mission.color[agent.id]);
        pub_sfc.publish(msg_sfc);
    }

    void TrajPlanner::publishLSC(){
        visualization_msgs::MarkerArray msg_lsc;
        msg_lsc = constraints.convertLSCsToMarkerArrayMsg(obstacles, mission.color);
        pub_lsc.publish(msg_lsc);
    }

    void TrajPlanner::publishFeasibleRegion(){
        visualization_msgs::MarkerArray msg_feasible_region;
//        ros::Time start_time = ros::Time::now();
        if(param.multisim_show_feasible_region) {
            msg_feasible_region = constraints.feasibleRegionToMarkerArrayMsg(agent.id,
                                                                             mission.color[agent.id]);
        }
//        ros::Time end_time = ros::Time::now();
//        ROS_INFO_STREAM("[TrajPlanner] feasible region gen time: " << (end_time - start_time).toSec());
        pub_feasible_region.publish(msg_feasible_region);
    }

    void TrajPlanner::publishGridPath() {
        visualization_msgs::MarkerArray msg_delete_all = msgDeleteAll();
        pub_grid_path.publish(msg_delete_all);

        visualization_msgs::MarkerArray msg_grid_path_vis =
                grid_based_planner->pathToMarkerMsg(agent.id,
                                                    param.world_frame_id,
                                                    mission.color[agent.id]);
        pub_grid_path.publish(msg_grid_path_vis);
    }

    void TrajPlanner::publishObstaclePrediction() {
        // obstacle prediction vis
        visualization_msgs::MarkerArray msg_obs_pred_traj_vis;
        msg_obs_pred_traj_vis.markers.clear();

        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
//        marker.ns = std::to_string(agent.id);

        marker.color.a = 0.07;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        int count = 0;
        for (size_t oi = 0; oi < obstacles.size(); oi++) {
            if (obstacles[oi].type == ObstacleType::AGENT) {
                continue;
            }

            double sample_dt = 0.2;
            double N_sample = floor(param.M * param.dt / sample_dt);
            for (int i = 0; i < N_sample; i++) {
                double sample_time = i * sample_dt;
                double obs_pred_size;
                if(i == 0){
                    obs_pred_size = obs_pred_sizes[oi].startPoint();
                }
                else{
                    obs_pred_size = obs_pred_sizes[oi].getPointAt(sample_time);
                }

                marker.scale.x = 2 * obs_pred_size;
                marker.scale.y = 2 * obs_pred_size;
                marker.scale.z = 2 * obs_pred_size * obstacles[oi].downwash;

                if(isnan(obs_pred_size)){
                    ROS_ERROR_STREAM("nan!" << sample_time);
                }

                marker.id = count;
                marker.pose.position = point3DToPointMsg(obs_pred_trajs[oi].getPointAt(sample_time));
                marker.pose.orientation = defaultQuaternion();
                msg_obs_pred_traj_vis.markers.emplace_back(marker);
                count++;
            }
        }

        if(count == 0){
            visualization_msgs::Marker marker_delete_all;
            marker_delete_all.action = visualization_msgs::Marker::DELETEALL;
            msg_obs_pred_traj_vis.markers.emplace_back(marker_delete_all);
        }

        pub_obs_pred_traj_vis.publish(msg_obs_pred_traj_vis);
    }

//    void TrajPlanner::publishGridOccupiedPoints() {
//        if (not param.world_use_octomap) {
//            return;
//        }
//
//        visualization_msgs::MarkerArray msg_delete_all = msgDeleteAll();
//        pub_grid_occupied_points.publish(msg_delete_all);
//
//        visualization_msgs::MarkerArray msg_occupied_points =
//                grid_based_planner->occupiedPointsToMsg(param.world_frame_id);
//        pub_grid_occupied_points.publish(msg_occupied_points);
//    }

    bool TrajPlanner::isDeadlock() {
//        double min_safety_ratio = SP_INFINITY;
//        for (int oi = 0; oi < obstacles.size(); oi++) {
//            // Coordinate transformation
//            double downwash = downwashBetween(oi);
//            traj_t initial_traj_trans = initial_traj.coordinateTransform(downwash);
//            traj_t obs_pred_traj_trans = obs_pred_trajs[oi].coordinateTransform(downwash);
//            double safety_ratio = (initial_traj_trans.startPoint() - obs_pred_traj_trans.startPoint()).norm() / (agent.radius + obstacles[oi].radius);
//            if(min_safety_ratio > safety_ratio){
//                min_safety_ratio = safety_ratio;
//            }
//        }

        //If agent's velocity is lower than some threshold, then it determines agent is in deadlock
        double dist_to_goal = (agent.current_state.position - agent.desired_goal_point).norm();
        return planner_seq > param.deadlock_seq_threshold and
               agent.current_state.velocity.norm() < param.deadlock_velocity_threshold and
               dist_to_goal > 0.2;
//        return min_safety_ratio < 1.1 and dist_to_goal > 0.2;
    }

    bool TrajPlanner::isSolValid(const TrajOptResult& result) const {
        // Check SFC
        if(param.world_use_octomap){
            for(int m = 0; m < param.M; m++){
                Box sfc = constraints.getSFC(m);
                if(m == 0){
                    for(int i = param.phi; i < param.n + 1; i++){
                        if(not sfc.isPointInBox(result.desired_traj[m][i])) {
                            ROS_WARN("[TrajPlanner] solution is not valid due to SFC");
                            return false;
                        }
                    }
                } else {
                    if(not sfc.isSegmentInBox(result.desired_traj[m])) {
                        ROS_WARN("[TrajPlanner] solution not valid due to SFC");
                        return false;
                    }
                }
            }
        }


        // Check LSC
//        for(int oi = 0; oi < obstacles.size(); oi++) {
//            for (int m = 0; m < param.M; m++) {
//                for (int i = 0; i < param.n + 1; i++) {
//                    if (m == 0 and i < param.phi) {
//                        continue;
//                    }
//
//                    LSC lsc = constraints.getLSC(oi, m, i);
//                    if (lsc.normal_vector.dot(result.desired_traj[m][i] - lsc.obs_control_point) - lsc.d <
//                        -SP_EPSILON_FLOAT) {
//                        ROS_WARN("[TrajPlanner] solution is not valid due to LSC");
//                        return false;
//                    }
//                }
//            }
//        }

        // Check dynamical limit
        double dyn_err_tol_ratio = 0.01;
        State state = result.desired_traj.getStateAt(param.multisim_time_step);
        for(int k = 0; k < param.world_dimension; k++){
            if(abs(state.velocity(k)) > agent.max_vel * (1 + dyn_err_tol_ratio)) {
                ROS_WARN("[TrajPlanner] solution is not valid due to max_vel");
                return false;
            }
            if(abs(state.acceleration(k)) > agent.max_acc * (1 + dyn_err_tol_ratio)) {
                ROS_WARN("[TrajPlanner] solution is not valid due to max_acc");
                return false;
            }
        }

        return true;
    }

    int TrajPlanner::findObstacleIdxByObsId(int obs_id) const {
        int oi = -1;
        for (size_t i = 0; i < obstacles.size(); i++) {
            if (obstacles[i].id == obs_id) {
                oi = i;
            }
        }
        return oi;
    }

    double TrajPlanner::distanceToGoalByObsId(int obs_id) const {
        int obs_idx = findObstacleIdxByObsId(obs_id);
        return distanceToGoalByObsIdx(obs_idx);
    }

    double TrajPlanner::distanceToGoalByObsIdx(int obs_idx) const {
        return obstacles[obs_idx].goal_point.distance(obstacles[obs_idx].position);
    }

    double TrajPlanner::computeCollisionTimeToDistmap(const point3d &start_position,
                                                      const point3d &goal_position,
                                                      double agent_radius,
                                                      double time_horizon) {
        double collision_time = 0;
        bool isCollided = distmap_ptr->getDistance(start_position) < agent_radius;
        if (goal_position == start_position) {
            if (isCollided) {
                collision_time = 0;
            } else {
                collision_time = SP_INFINITY;
            }
            return collision_time;
        }

        double search_time_step = 0.1;
        double current_time = 0;
        point3d current_search_point;
        while (!isCollided and current_time < time_horizon) {
            current_time += search_time_step;
            current_search_point = start_position + (goal_position - start_position) * (current_time / time_horizon);
            isCollided = distmap_ptr->getDistance(current_search_point) < agent_radius;
        }

        if (isCollided) {
            collision_time = current_time;
        } else {
            collision_time = SP_INFINITY;
        }

        return collision_time;
    }

//    double TrajPlanner::computeMinCollisionTime() {
//        double collision_time, min_collision_time = SP_INFINITY;
//        double total_time_horizon = M * param.dt;
//
//        if (param.world_use_octomap) {
//            collision_time = computeCollisionTimeToDistmap(initial_traj.startPoint(),
//                                                           initial_traj.lastPoint(),
//                                                           agent.radius,
//                                                           total_time_horizon);
//            if (min_collision_time > collision_time) {
//                min_collision_time = collision_time;
//            }
//        }
//
//        size_t N_obs = obstacles.size();
//        for (int oi = 0; oi < N_obs; oi++) {
//            collision_time = computeCollisionTime(obs_pred_trajs[oi].startPoint(),
//                                                  obs_pred_trajs[oi].lastPoint(),
//                                                  initial_traj.startPoint(),
//                                                  initial_traj.lastPoint(),
//                                                  obstacles[oi].radius + agent.radius,
//                                                  total_time_horizon);
//
//            if (min_collision_time > collision_time) {
//                min_collision_time = collision_time;
//            }
//
//            if (param.world_use_octomap) {
//                collision_time = computeCollisionTimeToDistmap(obs_pred_trajs[oi].startPoint(),
//                                                               obs_pred_trajs[oi].lastPoint(),
//                                                               obstacles[oi].radius,
//                                                               total_time_horizon);
//                if (min_collision_time > collision_time) {
//                    min_collision_time = collision_time;
//                }
//            }
//
//            for (int oj = 0; oj < N_obs; oj++) {
//                if (oj > oi) {
//                    collision_time = computeCollisionTime(obs_pred_trajs[oi].startPoint(),
//                                                          obs_pred_trajs[oi].lastPoint(),
//                                                          obs_pred_trajs[oj].startPoint(),
//                                                          obs_pred_trajs[oj].lastPoint(),
//                                                          obstacles[oi].radius + obstacles[oj].radius,
//                                                          total_time_horizon);
//                } else {
//                    continue;
//                }
//
//                if (min_collision_time > collision_time) {
//                    min_collision_time = collision_time;
//                }
//            }
//        }
//
//        return min_collision_time;
//    }

    point3d TrajPlanner::normalVectorBetweenLines(const Line &obs_path, const Line &agent_path, double &closest_dist) {
        ClosestPoints closest_points;
        closest_points = closestPointsBetweenLinePaths(obs_path, agent_path);
        closest_dist = closest_points.dist;

        point3d delta, normal_vector;
        delta = closest_points.closest_point2 - closest_points.closest_point1;
        normal_vector = delta.normalized();
        if (normal_vector.norm() == 0) {
            ROS_WARN("[Util] heuristic method was used to get normal vector");
            point3d a, b;
            a = agent_path.start_point - obs_path.start_point;
            b = agent_path.end_point - obs_path.end_point;
            if (a.norm() == 0 and b.norm() == 0) {
                normal_vector = point3d(1, 0, 0);
            } else {
                normal_vector = (b - a).cross(point3d(0, 0, 1));
            }
        }
        return normal_vector;
    }

    point3d TrajPlanner::normalVectorBetweenPolys(int oi, int m,
                                                  const traj_t &initial_traj_trans,
                                                  const traj_t &obs_pred_traj_trans) {
        size_t n_control_points = param.n + 1;
        point3ds control_points_rel(n_control_points);
        for (size_t i = 0; i < n_control_points; i++) {
            control_points_rel[i] = initial_traj_trans[m][i] - obs_pred_traj_trans[m][i];

            if (obstacles[oi].type != ObstacleType::AGENT and
                obstacles[oi].downwash > param.obs_downwash_threshold) {
                control_points_rel[i].z() = 0;
            }
        }

        ClosestPoints closest_points = closestPointsBetweenPointAndConvexHull(point3d(0, 0, 0),
                                                                              control_points_rel);
        point3d normal_vector = closest_points.closest_point2.normalized();

        if (obstacles[oi].type == AGENT and closest_points.dist < agent.radius + obstacles[oi].radius - 0.001) {
            ROS_WARN_STREAM("[TrajPlanner] invalid normal_vector: " << normal_vector
                                                                    << ", dist: " << closest_points.dist
                                                                    << ", agent_id: " << agent.id << ", obs_id: "
                                                                    << obstacles[oi].id);
        }
        return normal_vector;
    }

    point3d TrajPlanner::normalVectorDynamicObs(int oi, int m, double downwash) {
        point3d normal_vector;

        //Coordinate transformation
        point3d vector_obs_to_goal = coordinateTransform(agent.current_goal_point - obstacles[oi].position,
                                                         downwash);
        point3d vector_obs_to_agent = coordinateTransform(agent.current_state.position - obstacles[oi].position,
                                                          downwash);
        // Ignore z axis if obstacle's downwash is too big
        if (obstacles[oi].downwash > param.obs_downwash_threshold) {
            vector_obs_to_goal.z() = 0;
            vector_obs_to_agent.z() = 0;
        }

        Line line_obs(obs_pred_trajs[oi][m][0], obs_pred_trajs[oi][m][param.n]);
        Line line_agent(initial_traj[m][0], initial_traj[m][param.n]);
        double closest_dist = 0;
        normal_vector = normalVectorBetweenLines(line_obs, line_agent, closest_dist);
        return normal_vector;
    }

    double TrajPlanner::downwashBetween(int oi) const {
        double downwash = 1;
        if (obstacles[oi].type == ObstacleType::AGENT) {
            downwash = (agent.downwash * agent.radius + obstacles[oi].downwash * obstacles[oi].radius) /
                       (agent.radius + obstacles[oi].radius);
        } else {
            downwash = (agent.radius + obstacles[oi].downwash * obstacles[oi].radius) /
                       (agent.radius + obstacles[oi].radius);
        }

        return downwash;
    }

    double TrajPlanner::downwashBetween(int oi, int oj) const {
        double downwash = 1;

        if (obstacles[oi].type != obstacles[oj].type) {
            if (obstacles[oi].type == ObstacleType::AGENT) {
                downwash = (obstacles[oi].radius + obstacles[oj].downwash * obstacles[oj].radius) /
                           (obstacles[oi].radius + obstacles[oj].radius);
            } else if (obstacles[oj].type == ObstacleType::AGENT) {
                downwash = (obstacles[oi].downwash * obstacles[oi].radius + obstacles[oj].radius) /
                           (obstacles[oi].radius + obstacles[oj].radius);
            }
        } else {
            downwash = (obstacles[oi].downwash * obstacles[oi].radius +
                        obstacles[oj].downwash * obstacles[oj].radius) /
                       (obstacles[oi].radius + obstacles[oj].radius);
        }

        return downwash;
    }

    point3d TrajPlanner::coordinateTransform(const point3d &point, double downwash) {
        point3d point_trans = point;
        point_trans.z() = point_trans.z() / downwash;
        return point_trans;
    }

    double TrajPlanner::computePriority(double dist_to_goal, double goal_threshold){
        double priority;
        if(dist_to_goal < goal_threshold){
            priority = dist_to_goal;
        }
        else{
            priority = goal_threshold + 1/dist_to_goal;
        }

        return priority;
    }

    void TrajPlanner::updateCurrentGoal(){
        for(int m = param.M - 2; m >= 0; m--){
            Box sfc_next = constraints.getSFC(m + 1);
            Box sfc_curr = constraints.getSFC(m);
            Box inter_sfc = sfc_next.intersection(sfc_curr);
            Segment<point3d> target_segment = initial_traj[m];
            point3d last_point = target_segment.lastPoint();
            if(inter_sfc.isPointInBox(last_point) and
               not sfc_curr.isPointInBoxStrictly(last_point) and
               not inter_sfc.isSegmentInBox(target_segment)){
                agent.current_goal_point = target_segment.lastPoint();
            }
        }
    }
}