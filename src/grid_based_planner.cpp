#include <grid_based_planner.hpp>
#pragma GCC push_options
#pragma GCC optimize ("O0")

namespace MATP {
    void PlanResult::initialize() {
        paths.clear();
        agent_ids.clear();
    }

    int PlanResult::getMakespan() {
        if(paths.empty()){
            return -1;
        }

        int max_size = 0;
        for(const auto& path: paths){
            if(path.empty()){
                return -1;
            } else if(max_size < path.size()) {
                max_size = (int)path.size() - 1;
            }
        }

        return max_size;
    }

    GridBasedPlanner::GridBasedPlanner(const MATP::Param &_param,
                                       const MATP::Mission &_mission)
            : mission(_mission), param(_param) {
        agent_downwash = mission.agents[0].downwash; //TODO: agent 0

        for (int i = 0; i < 3; i++) {
            double grid_resolution = getGridResolution(i);
            grid_min[i] = -floor((-mission.world_min(i) + SP_EPSILON) / grid_resolution) * grid_resolution;
            grid_max[i] = floor((mission.world_max(i) + SP_EPSILON) / grid_resolution) * grid_resolution;
        }
        if (param.world_dimension == 2) {
            grid_min[2] = param.world_z_2d;
            grid_max[2] = param.world_z_2d;
        }

        for (int i = 0; i < param.world_dimension; i++) {
            double grid_resolution = getGridResolution(i);
            dim[i] = (int) round((grid_max[i] - grid_min[i]) / grid_resolution) + 1;
        }
        if (param.world_dimension == 2) {
            dim[2] = 1;
        }
    }

//    bool GridBasedPlanner::planSAPF(const Agent &agent,
//                                    const std::shared_ptr<DynamicEDTOctomap> &_distmap_ptr,
//                                    const Obstacles &obstacles,
//                                    const std::set<int> &grid_obstacles) {
//        distmap_ptr = _distmap_ptr;
//        checkMission(agent.current_state.position, agent.desired_goal_point);
//        updateGridMap(agent.radius, obstacles, grid_obstacles);
//
//        bool success = runSAPF();
//        return success;
//    }

    bool GridBasedPlanner::planMAPF(const MAPFAgents &_mapf_agents,
                                    const std::shared_ptr<DynamicEDTOctomap> &_distmap_ptr,
                                    double agent_radius,
                                    const Obstacles &obstacles) {
        MAPFAgents mapf_agents = _mapf_agents;
        distmap_ptr = _distmap_ptr;
        n_agents = mapf_agents.size();

        // Update grid map
        updateGridMap(agent_radius, obstacles);
        bool valid_grid_map = isMissionValid(mapf_agents);
        if(not valid_grid_map) {
            updateGridMap(agent_radius);
        }

        // Set DOI and update desired goal point
        updateDistanceTables(obstacles);
        updateDOI(mapf_agents, obstacles, agent_radius);
        updateGoal(mapf_agents);

        bool success = runMAPF(mapf_agents);
        if(not success and not obstacles.empty()){
            // If failed, retry it without dynamic obstacles
            updateGridMap(agent_radius);
            success = runMAPF(mapf_agents);
        }

        return success;
    }

    void GridBasedPlanner::updateGridMap(double agent_radius,
                                         const Obstacles &obstacles) {
        int width = dim[0];
        int depth = dim[1];
        int height = dim[2];

        // create nodes
        Nodes V = Nodes(width * depth * height, nullptr);
        point3d delta = point3d(1, 1, 1) * 0.5 * param.world_resolution;
        std::set<int> warning_node_ids;
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < depth; y++) {
                for (int z = 0; z < height; z++) {
                    int id = width * depth * z + width * y + x;
                    point3d search_point = posToPoint3D(Pos(x, y, z));

                    // Static obstacles
                    if (distmap_ptr != nullptr) {
                        float dist;
                        point3d closest_point;
                        distmap_ptr->getDistanceAndClosestObstacle(search_point, dist, closest_point);
                        Box closest_cell(closest_point - delta, closest_point + delta);
                        closest_point = closest_cell.closestPoint(search_point);

                        // Due to numerical error of getDistance function, explicitly compute distance to obstacle
                        double dist_to_obs = LInfinityDistance(search_point, closest_point);
                        if (dist_to_obs < agent_radius - SP_EPSILON_FLOAT) {
                            continue;
                        }
                    }

                    // Detour when real obstacles stop
                    //TODO: [CAUTION] It assumes that real obstacle is cylinder-shaped
                    bool skip = false;
                    for(const auto& obstacle: obstacles){
                        if(obstacle.type == ObstacleType::DYN_REAL and
                           obstacle.velocity.norm() < param.obs_velocity_threshold and
                           search_point.distanceXY(obstacle.position) < agent_radius + obstacle.radius) {
                            skip = true;
                            break;
                        }
                    }
                    if(skip) {
                        continue;
                    }

                    bool warning = false;
                    // Warning_node: the node which is in the reachable region of dynamic obstacles
                    for(const auto& obstacle: obstacles) {
                        if(obstacle.type == ObstacleType::DYN_REAL){
                            continue;
                        }

                        bool is_point_collided = obstacle.isCollided(search_point,
                                                                     agent_radius,
                                                                     param.M * param.dt,
                                                                     param.obs_uncertainty_horizon);
                        if(is_point_collided) {
                            warning = true;
                        }
                    }

                    // Set new node
                    Node *v = new Node(id, x, y, z, warning);
                    V[id] = v;
                }
            }
        }

        grid_map = new Grid(V, width, depth, height);
    }

    void GridBasedPlanner::updateDistanceTables(const Obstacles &obstacles) {
        obs_distance_tables.clear();
        std::vector<int> obs_ids;
        for (const auto &obstacle: obstacles) {
            Node *obs_node = point3DToClosestNode(obstacle.position);
            obs_ids.emplace_back(obs_node->id);
        }
        obs_distance_tables = createDistanceTable(obs_ids);
    }

    bool GridBasedPlanner::isMissionValid(const MAPFAgents &mapf_agents) {
        for (const auto& mapf_agent: mapf_agents) {
            if (isOccupied(mapf_agent.current_waypoint)) {
                ROS_ERROR_STREAM("[GridBasedPlanner] Current point (" << mapf_agent.current_waypoint << ") is occluded");
                return false;
            }

            if (isOccupied(mapf_agent.goal_point)) {
                ROS_ERROR_STREAM("[GridBasedPlanner] Goal point (" << mapf_agent.goal_point << ") is occluded");
                return false;
            }
        }

        return true;
    }

    void GridBasedPlanner::updateDOI(MAPFAgents &mapf_agents, const Obstacles &obstacles, double agent_radius) {
        for (auto& mapf_agent: mapf_agents) {
            DOI dyn_obs_interest;
            dyn_obs_interest.agent_position = mapf_agent.current_agent_position;

            // Find the candidates of dyn_obs_interest
            Obstacles doi_cands;
            Node *g = point3DToClosestNode(mapf_agent.current_waypoint);
            if(mapf_agent.collision_alert.obstacles.empty()){
                for (int oi = 0; oi < obstacles.size(); oi++) {
                    // If the velocity of the obstacle is too slow, consider it as a static obstacle
                    if (obstacles[oi].type == ObstacleType::DYN_REAL and
                        obstacles[oi].velocity.norm() < param.obs_velocity_threshold) {
                        continue;
                    }

                    // If the waypoint is in the reachable region of the obstacle,
                    // assign the obstacle as doi_candidate.
                    bool is_waypoint_collided = obstacles[oi].isCollided(mapf_agent.current_waypoint,
                                                                         agent_radius,
                                                                         param.M * param.dt,
                                                                         param.obs_uncertainty_horizon);
                    if (is_waypoint_collided) {
                        doi_cands.emplace_back(obstacles[oi]);
                    }
                }
            } else {
                for(const auto& obstacle: mapf_agent.collision_alert.obstacles) {
                    // If the velocity of the obstacle is too slow, consider it as a static obstacle
                    if(obstacle.type == ObstacleType::DYN_REAL and
                       obstacle.velocity.norm() < param.obs_velocity_threshold){
                        continue;
                    }

                    doi_cands.emplace_back(obstacle);
                }
            }

            // Assign the closest obstacle among the doi_cands to dyn_obs_interest
            double min_dist = SP_INFINITY;
            for (const auto &obstacle: doi_cands) {
                dyn_obs_interest.doi_cand_ids.insert(obstacle.id);

                double dist = obstacle.position.distance(mapf_agent.collision_alert.agent_position);
                if (dist < min_dist) {
                    min_dist = dist;
                    dyn_obs_interest.closest_obs_point = obstacle.position;
                    dyn_obs_interest.closest_obs_dist = min_dist;
                    dyn_obs_interest.closest_obs_id = obstacle.id;
                }
            }

            // Update dyn_obs_interest to mapf_agent
            mapf_agent.dyn_obs_interest = dyn_obs_interest;
        }
    }


    void GridBasedPlanner::updateGoal(MAPFAgents &mapf_agents) {
        for (auto& mapf_agent: mapf_agents) {
            if (not mapf_agent.dyn_obs_interest.exist()) {
                continue;
            }

            Node *n = point3DToClosestNode(mapf_agent.dyn_obs_interest.agent_position);
            Node *g = point3DToClosestNode(mapf_agent.current_waypoint);
            point3d new_goal_point = nodeToPoint3D(n);
            double min_cost = SP_INFINITY;
            bool restart_search = false;

            std::queue<Node *> OPEN;
            OPEN.push(n);
            while (!OPEN.empty()) {
                n = OPEN.front();
                OPEN.pop();

                if (!restart_search and n->id == g->id) {
                    // restart BFS at the goal
                    std::queue<Node *> empty;
                    std::swap(OPEN, empty);
                    OPEN.push(g);
                    min_cost = SP_INFINITY;
                    new_goal_point = nodeToPoint3D(g);
                    restart_search = true;
                    continue;
                }

                const double c_n = getObsCost(mapf_agent.dyn_obs_interest.doi_cand_ids, n->id);
                for (const auto &m: n->neighbor) {
                    const double c_m = getObsCost(mapf_agent.dyn_obs_interest.doi_cand_ids, m->id);
                    if (c_n < c_m + SP_EPSILON_FLOAT) {
                        continue;
                    }
                    if (c_m < min_cost) {
                        min_cost = c_m;
                        new_goal_point = nodeToPoint3D(m);
                    }
                    OPEN.push(m);
                }

                if (min_cost < 0.01) { //TODO: param
                    break;
                }
            }

            mapf_agent.goal_point = new_goal_point;
        }
    }

    void GridBasedPlanner::updatePlanResult(const MAPF::Plan &plan, const MAPFAgents &mapf_agents) {
        // Plan initial path
        PlanResult prev_plan_result = planInitialPath(plan_result, mapf_agents);

        // Initialize plan_result
        plan_result.initialize();

        // Delete the repeated path and update path
        int repeat_start_idx = 0;
        for (int i = 1; i < plan.size(); i++) {
            bool repeat = true;
            for (int qi = 0; qi < n_agents; qi++) {
                if (plan.get(0, qi) != plan.get(i, qi)) {
                    repeat = false;
                    break;
                }
            }
            if (repeat) {
                repeat_start_idx = i;
            }
        }
        plan_result.paths.resize(n_agents);
        for (int qi = 0; qi < n_agents; qi++) {
            plan_result.agent_ids.insert(mapf_agents[qi].id);
            for (int i = repeat_start_idx; i < plan.size(); i++) {
                plan_result.paths[qi].emplace_back(nodeToPoint3D(plan.get(i, qi)));
            }
        }

        // Check the solution is valid
        bool valid_sol_found = isSolutionValid(plan_result, mapf_agents);
        bool valid_prev_sol_found = isSolutionValid(prev_plan_result, mapf_agents);

        // Check DOI exists
        bool doi_exist = false;
        for (const auto &mapf_agent: mapf_agents) {
            if (mapf_agent.dyn_obs_interest.exist()) {
                doi_exist = true;
                break;
            }
        }

        // Check new agent is added
        bool new_agent_added = plan_result.agent_ids != prev_plan_result.agent_ids;

        // Check new solution is better than the previous one
        bool better_solution_found = plan_result.getMakespan() < prev_plan_result.getMakespan();

        // If no DOI exist, no additional agent, no better solution found, use the previous plan result
        if (!doi_exist && !new_agent_added && (!valid_sol_found || (!better_solution_found && valid_prev_sol_found))) {
            plan_result = prev_plan_result;
        }
    }

    bool GridBasedPlanner::isValid(const point3d &point) {
        for (int i = 0; i < param.world_dimension; i++) {
            if (point(i) < grid_min[i] || point(i) > grid_max[i]) {
                return false;
            }
        }

        return true;
    }

    bool GridBasedPlanner::isOccupied(const point3d &point) {
        return (not isValid(point)) or (not grid_map->existNode(point3DToID(point)));
    }

    bool GridBasedPlanner::isSolutionValid(const PlanResult &plan_result, const MAPFAgents &mapf_agents) {
        if(plan_result.paths.empty()){
            return false;
        }

        for (int qi = 0; qi < n_agents; qi++) {
            if ((plan_result.paths[qi].back() - mapf_agents[qi].goal_point).norm() > SP_EPSILON_FLOAT) {
                return false;
            }
        }

        return true;
    }

    PlanResult GridBasedPlanner::planInitialPath(const PlanResult &prev_plan_result, const MAPFAgents &mapf_agents) {
        std::set<size_t> agent_ids;
        for (const auto &mapf_agent: mapf_agents) {
            agent_ids.insert(mapf_agent.id);
        }
        if (prev_plan_result.agent_ids.empty()
            or prev_plan_result.agent_ids.size() != n_agents
            or agent_ids != prev_plan_result.agent_ids) {
            return prev_plan_result;
        }

        // Find the agents that updates the waypoint
        std::set<size_t> updated_agents;
        for (int qi = 0; qi < n_agents; qi++) {
            if (prev_plan_result.paths[qi].size() < 2
                or (prev_plan_result.paths[qi][1] - mapf_agents[qi].current_waypoint).norm() < SP_EPSILON_FLOAT) {
                updated_agents.insert(qi);
            }
        }

        // Update initial path
        PlanResult init_plan_result = prev_plan_result;
        if (updated_agents.size() == n_agents) {
            // If waypoints of all agents are updated, makespan -= 1
            for (auto &path: init_plan_result.paths) {
                if(path.size() > 1){
                    path.erase(path.begin());
                }
            }
        } else {
            // makespan preserved
            for (int qi = 0; qi < n_agents; qi++) {
                if (prev_plan_result.paths[qi].size() > 1 and updated_agents.find(qi) != updated_agents.end()) {
                    init_plan_result.paths[qi][0] = prev_plan_result.paths[qi][1];
                }
            }
        }

        return init_plan_result;
    }

    bool GridBasedPlanner::runMAPF(const MAPFAgents &mapf_agents) {
        // Initialize start, current, goal points
        ProblemAgents agents;
        for (const auto &mapf_agent: mapf_agents) {
            agents.emplace_back(point3DToNode(mapf_agent.start_point),
                                point3DToNode(mapf_agent.current_waypoint),
                                point3DToNode(mapf_agent.goal_point),
                                point3DToClosestNode(mapf_agent.dyn_obs_interest.closest_obs_point),
                                mapf_agent.dyn_obs_interest.closest_obs_dist);
        }

        // Run MAPF
        MAPF::Problem P = MAPF::Problem(grid_map, n_agents, agents);
        std::unique_ptr<MAPF::Solver> solver;
        if (param.mapf_mode == MAPFMode::PIBT) {
            solver = std::make_unique<MAPF::PIBT>(&P);
        } else if (param.mapf_mode == MAPFMode::ECBS) {
            //TODO: ECBS is not supported yet.
            solver = std::make_unique<MAPF::ECBS>(&P);
        } else {
            throw std::invalid_argument("[GridBasedPlanner] Invalid MAPF mode");
        }

        solver->solve();

        MAPF::Plan plan = solver->getSolution();
        updatePlanResult(plan, mapf_agents);

        return not plan.empty();
    }

    Nodes GridBasedPlanner::pointsToNodes(const point3ds &points) const {
        Nodes nodes;
        for (const auto &point: points) {
            nodes.emplace_back(point3DToNode(point));
        }
        return nodes;
    }

    Nodes GridBasedPlanner::pointsToClosestNodes(const point3ds &points) const {
        Nodes nodes;
        for (const auto &point: points) {
            nodes.emplace_back(point3DToClosestNode(point));
        }
        return nodes;
    }

    Node *GridBasedPlanner::point3DToNode(const point3d &point) const {
        int id = point3DToID(point);
        if (not grid_map->existNode(id)) {
            return nullptr;
        }

        return grid_map->getNode(id);
    }

    Node *GridBasedPlanner::point3DToClosestNode(const point3d &point) const {
        Node *node = point3DToNode(point);
        if (node != nullptr) {
            return node;
        }

        // find the closest unoccupied node
        // near node search
        Pos pos = point3DToPos(point);
        Nodes node_cands;
        if (grid_map->existNode(pos.x + 1, pos.y, pos.z)) {
            node_cands.emplace_back(grid_map->getNode(pos.x + 1, pos.y, pos.z));
        }
        if (grid_map->existNode(pos.x - 1, pos.y, pos.z)) {
            node_cands.emplace_back(grid_map->getNode(pos.x - 1, pos.y, pos.z));
        }
        if (grid_map->existNode(pos.x, pos.y + 1, pos.z)) {
            node_cands.emplace_back(grid_map->getNode(pos.x, pos.y + 1, pos.z));
        }
        if (grid_map->existNode(pos.x, pos.y - 1, pos.z)) {
            node_cands.emplace_back(grid_map->getNode(pos.x, pos.y - 1, pos.z));
        }
        if (grid_map->existNode(pos.x, pos.y, pos.z + 1)) {
            node_cands.emplace_back(grid_map->getNode(pos.x, pos.y, pos.z + 1));
        }
        if (grid_map->existNode(pos.x, pos.y, pos.z - 1)) {
            node_cands.emplace_back(grid_map->getNode(pos.x, pos.y, pos.z - 1));
        }
        if (not node_cands.empty()) {
            double min_dist = SP_INFINITY;
            for (const auto &node_cand: node_cands) {
                double dist = point.distance(nodeToPoint3D(node_cand));
                if (dist < min_dist) {
                    min_dist = dist;
                    node = node_cand;
                }
            }
            return node;
        }

        // naive search
        double min_dist = SP_INFINITY;
        for (int id = 0; id < grid_map->getNodesSize(); id++) {
            if (not grid_map->existNode(id)) {
                continue;
            }

            Node *node_cand = grid_map->getNode(id);
            double dist = point.distance(nodeToPoint3D(node_cand));
            if (dist < min_dist) {
                min_dist = dist;
                node = node_cand;
            }
        }
        return node;
    }

    point3d GridBasedPlanner::posToPoint3D(const Pos &pos) const {
        point3d point;
        point.x() = grid_min[0] + pos.x * getGridResolution(0);
        point.y() = grid_min[1] + pos.y * getGridResolution(1);
        if (param.world_dimension == 2) {
            point.z() = param.world_z_2d;
        } else {
            point.z() = grid_min[2] + pos.z * getGridResolution(2);
        }

        return point;
    }

    point3d GridBasedPlanner::nodeToPoint3D(Node *node) const {
        return posToPoint3D(node->pos);
    }

    Pos GridBasedPlanner::point3DToPos(const point3d &point) const {
        std::array<int, 3> grid_position = {0, 0, 0};
        for (int i = 0; i < param.world_dimension; i++) {
            double grid_resolution = getGridResolution(i);
            grid_position[i] = (int) round((point(i) - grid_min[i]) / grid_resolution);

            // clamping
            if (grid_position[i] < 0) {
                grid_position[i] = 0;
            } else if (grid_position[i] >= dim[i]) {
                grid_position[i] = dim[i] - 1;
            }
        }

        Pos pos(grid_position[0], grid_position[1], grid_position[2]);
        return pos;
    }

    int GridBasedPlanner::point3DToID(const point3d &point) const {
        Pos pos = point3DToPos(point);
        int id = grid_map->getWidth() * grid_map->getDepth() * pos.z + grid_map->getWidth() * pos.y + pos.x;
        return id;
    }

    point3ds GridBasedPlanner::getPath(size_t i) const {
        return plan_result.paths[i];
    }

    visualization_msgs::MarkerArray GridBasedPlanner::pathToMarkerMsg(int agent_id,
                                                                      const std::string &frame_id,
                                                                      std_msgs::ColorRGBA color) const {
        visualization_msgs::MarkerArray msg_grid_path_vis;
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.color = color;
        marker.color.a = 0.2;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        if (plan_result.paths.empty()) {
            return msg_grid_path_vis;
        }

        int marker_id = 0;
        for (const auto &point: plan_result.paths[0]) {
            marker.ns = std::to_string(agent_id);
            marker.id = marker_id++;
            marker.pose.position = point3DToPointMsg(point);
            marker.pose.orientation = defaultQuaternion();
            msg_grid_path_vis.markers.emplace_back(marker);
        }

        return msg_grid_path_vis;
    }

    double GridBasedPlanner::getGridResolution(int i) const {
        if (i < 2) {
            return param.grid_resolution;
        } else {
            return param.grid_resolution * agent_downwash;
        }
    }

    DistanceTable GridBasedPlanner::createDistanceTable(const point3ds &points) {
        DistanceTable distance_table(points.size(),
                                     std::vector<int>(grid_map->getNodesSize(), SP_INFINITY));
        for (int i = 0; i < points.size(); ++i) {
            // breadth first search
            std::queue<Node *> OPEN;
            Node *n = point3DToClosestNode(points[i]);
            OPEN.push(n);
            distance_table[i][n->id] = 0;
            while (!OPEN.empty()) {
                n = OPEN.front();
                OPEN.pop();
                const int d_n = distance_table[i][n->id];
                for (auto m: n->neighbor) {
                    const int d_m = distance_table[i][m->id];
                    if (d_n + 1 >= d_m) continue;
                    distance_table[i][m->id] = d_n + 1;
                    OPEN.push(m);
                }
            }
        }

        return distance_table;
    }

    DistanceTable GridBasedPlanner::createDistanceTable(const std::vector<int> &ids) {
        DistanceTable distance_table(ids.size(),
                                     std::vector<int>(grid_map->getNodesSize(), SP_INFINITY));
        for (int i = 0; i < ids.size(); ++i) {
            // breadth first search
            std::queue<Node *> OPEN;
            Node *n = grid_map->getNode(ids[i]);
            OPEN.push(n);
            distance_table[i][n->id] = 0;
            while (!OPEN.empty()) {
                n = OPEN.front();
                OPEN.pop();
                const int d_n = distance_table[i][n->id];
                for (auto m: n->neighbor) {
                    const int d_m = distance_table[i][m->id];
                    if (d_n + 1 >= d_m) continue;
                    distance_table[i][m->id] = d_n + 1;
                    OPEN.push(m);
                }
            }
        }

        return distance_table;
    }

    int GridBasedPlanner::getObsId(const Obstacles &obstacles, const point3d &obs_point) {
        int obs_id = -1;
        double min_dist = SP_INFINITY;
        for(int oi = 0; oi < obstacles.size(); oi++){
            double dist = obs_point.distance(obstacles[oi].position);
            if(dist < min_dist) {
                min_dist = dist;
                obs_id = oi;
            }
        }

        return obs_id;
    }

    double GridBasedPlanner::getObsCost(const std::set<int> &obs_ids, int agent_id) {
        double cost = 0;
        for(const auto& obs_id: obs_ids){
            int dist = obs_distance_tables[obs_id][agent_id];
            if(dist == 0){
                cost += SP_INFINITY;
            } else {
                cost += 1.0 / (dist * dist);
            }
        }

        return cost;
    }
}

#pragma GCC pop_options
